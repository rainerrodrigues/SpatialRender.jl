# MCAP container format (https://mcap.dev/spec), the default storage of
# ROS 2 bags since Iron. Native reader (memory-mapped, chunk compression
# none / zstd / lz4) and a small writer for recording and test fixtures.

const MCAP_MAGIC = UInt8[0x89, 0x4d, 0x43, 0x41, 0x50, 0x30, 0x0d, 0x0a]   # "\x89MCAP0\r\n"

module Op
const HEADER = 0x01; const FOOTER = 0x02; const SCHEMA = 0x03; const CHANNEL = 0x04
const MESSAGE = 0x05; const CHUNK = 0x06; const DATA_END = 0x0f
end

struct MCAPSchema
    id::UInt16
    name::String          # e.g. "sensor_msgs/msg/PointCloud2"
    encoding::String      # e.g. "ros2msg"
    data::Vector{UInt8}
end

struct MCAPChannel
    id::UInt16
    schema_id::UInt16
    topic::String
    message_encoding::String   # e.g. "cdr"
    metadata::Dict{String, String}
end

"Where one message lives: file `file`, at top level (`chunk == 0`) or in a decompressed chunk."
struct MessageRef
    log_time::Int64
    publish_time::Int64
    channel::UInt16
    sequence::UInt32
    file::Int32
    chunk::Int32
    offset::Int          # 0-based offset of the payload in the file / chunk
    len::Int
end

struct ChunkInfo
    file::Int32
    offset::Int          # 0-based file offset of the (possibly compressed) records
    len::Int
    compression::String
    uncompressed_size::Int
end

"""
    MCAPFile

An opened MCAP recording: schemas, channels and a time-sorted index of every
message. A split rosbag2 recording (several `.mcap` files) is one `MCAPFile`
with one timeline. Payloads are read lazily ([`message_data`](@ref));
compressed chunks are decompressed on demand and a few are cached.
"""
mutable struct MCAPFile
    path::String                      # the file, or the rosbag2 directory
    files::Vector{String}
    data::Vector{Vector{UInt8}}       # memory map of each file
    schemas::Dict{UInt16, MCAPSchema}
    channels::Dict{UInt16, MCAPChannel}
    messages::Vector{MessageRef}
    chunks::Vector{ChunkInfo}
    cache::Vector{Pair{Int32, Vector{UInt8}}}     # most recently used last
    lock::ReentrantLock
end

# --- little-endian field readers over a byte buffer -----------------------------

mutable struct ByteCursor{V <: AbstractVector{UInt8}}
    d::V
    pos::Int          # 0-based
end
@inline function rd(c::ByteCursor, ::Type{T}) where {T}
    v = GC.@preserve c unsafe_load(Ptr{T}(pointer(c.d) + c.pos))
    c.pos += sizeof(T)
    return ltoh(v)
end
function rdstr(c::ByteCursor)
    n = Int(rd(c, UInt32))
    s = String(c.d[c.pos+1:c.pos+n]); c.pos += n
    return s
end
function rdbytes32(c::ByteCursor)
    n = Int(rd(c, UInt32))
    b = c.d[c.pos+1:c.pos+n]; c.pos += n
    return b
end
function rdmap(c::ByteCursor)
    n = Int(rd(c, UInt32)); stop = c.pos + n
    m = Dict{String, String}()
    while c.pos < stop
        k = rdstr(c); m[k] = rdstr(c)
    end
    return m
end

"""
    open_mcap(path) -> MCAPFile

Open an `.mcap` file, or a rosbag2 directory. A directory may hold a split
recording (several `.mcap` files, as `ros2 bag record --max-bag-size` writes);
all of them are read, in the order `metadata.yaml` lists them (else by name),
into one time-sorted index.
"""
function open_mcap(path::AbstractString)
    files = isdir(path) ? _bag_files(path) : [String(path)]
    f = MCAPFile(String(path), files, Vector{UInt8}[], Dict{UInt16, MCAPSchema}(), Dict{UInt16, MCAPChannel}(),
                 MessageRef[], ChunkInfo[], Pair{Int32, Vector{UInt8}}[], ReentrantLock())
    schema_ids = Dict{Tuple{String, String, Vector{UInt8}}, UInt16}()
    channel_ids = Dict{Tuple{String, String, UInt16}, UInt16}()
    for (k, file) in enumerate(files)
        data = Mmap.mmap(file)
        length(data) >= 16 && data[1:8] == MCAP_MAGIC || error("$file is not an MCAP file")
        push!(f.data, data)
        # scan this file with its own id tables, then merge them by content:
        # each file of a split bag is a complete MCAP and may number differently
        local_sc = Dict{UInt16, MCAPSchema}(); local_ch = Dict{UInt16, MCAPChannel}()
        first_msg = length(f.messages) + 1
        _scan_records!(f, local_sc, local_ch, data, 8, length(data), Int32(0), Int32(k))
        if length(files) == 1                       # a single file keeps its own ids
            merge!(f.schemas, local_sc); merge!(f.channels, local_ch)
            continue
        end
        scmap = Dict{UInt16, UInt16}(UInt16(0) => UInt16(0))     # 0 = schemaless
        for (id, sc) in local_sc
            scmap[id] = get!(schema_ids, (sc.name, sc.encoding, sc.data)) do
                g = UInt16(length(f.schemas) + 1)
                f.schemas[g] = MCAPSchema(g, sc.name, sc.encoding, sc.data); g
            end
        end
        chmap = Dict{UInt16, UInt16}()
        for (id, ch) in local_ch
            sid = get(scmap, ch.schema_id, UInt16(0))
            chmap[id] = get!(channel_ids, (ch.topic, ch.message_encoding, sid)) do
                g = UInt16(length(f.channels) + 1)
                f.channels[g] = MCAPChannel(g, sid, ch.topic, ch.message_encoding, ch.metadata); g
            end
        end
        for j in first_msg:length(f.messages)
            m = f.messages[j]
            f.messages[j] = MessageRef(m.log_time, m.publish_time, get(chmap, m.channel, m.channel),
                                       m.sequence, m.file, m.chunk, m.offset, m.len)
        end
    end
    sort!(f.messages; by = m -> m.log_time, alg = MergeSort)       # stable: keeps file order on ties
    return f
end

# The data files of a rosbag2 directory, in recording order.
function _bag_files(dir)
    mcaps = sort(filter(f -> endswith(f, ".mcap"), readdir(dir)))
    isempty(mcaps) && error("$dir contains no .mcap file " *
        "(rosbag2 SQLite `.db3` bags are not supported: convert with `ros2 bag convert` to MCAP)")
    meta = joinpath(dir, "metadata.yaml")
    if isfile(meta)                                          # relative_file_paths: - a.mcap - b.mcap
        listed = String[]; inlist = false
        for l in eachline(meta)
            if occursin(r"^\s*relative_file_paths:", l)
                inlist = true
            elseif inlist
                m = match(r"^\s*-\s*(\S+\.mcap)\s*$", l)
                m === nothing ? (inlist = false) : push!(listed, m[1])
            end
        end
        listed = filter(x -> x in mcaps, listed)
        !isempty(listed) && return [joinpath(dir, x) for x in listed]
    end
    return [joinpath(dir, x) for x in mcaps]
end

# Walk a record stream: top level of file `file` (chunk 0) or the inside of a chunk.
function _scan_records!(f::MCAPFile, schemas, channels, d, start, stop, chunk::Int32, file::Int32)
    c = ByteCursor(d, start)
    while c.pos + 9 <= stop
        op = d[c.pos+1]; c.pos += 1
        len = Int(rd(c, UInt64))
        body = c.pos
        c.pos + len > stop && break                                  # truncated file: keep what we have
        if op == Op.SCHEMA
            id = rd(c, UInt16); name = rdstr(c); enc = rdstr(c); sd = rdbytes32(c)
            schemas[id] = MCAPSchema(id, name, enc, sd)
        elseif op == Op.CHANNEL
            id = rd(c, UInt16); sid = rd(c, UInt16); topic = rdstr(c); enc = rdstr(c); md = rdmap(c)
            channels[id] = MCAPChannel(id, sid, topic, enc, md)
        elseif op == Op.MESSAGE
            ch = rd(c, UInt16); seq = rd(c, UInt32); lt = rd(c, UInt64); pt = rd(c, UInt64)
            push!(f.messages, MessageRef(Int64(lt), Int64(pt), ch, seq, file, chunk, c.pos, body + len - c.pos))
        elseif op == Op.CHUNK && chunk == 0
            rd(c, UInt64); rd(c, UInt64)                              # start / end time
            usize = Int(rd(c, UInt64)); rd(c, UInt32)                 # uncompressed size, crc
            comp = rdstr(c); rlen = Int(rd(c, UInt64))
            push!(f.chunks, ChunkInfo(file, c.pos, rlen, comp, usize))
            id = Int32(length(f.chunks))
            records = _chunk_records(f, id)
            _scan_records!(f, schemas, channels, records, 0, length(records), id, file)
        elseif op == Op.DATA_END || op == Op.FOOTER
            break
        end
        c.pos = body + len
    end
end

function _decompress(info::ChunkInfo, raw)
    info.compression == "" && return Vector{UInt8}(raw)
    info.compression == "zstd" && return zstd_decompress(raw, info.uncompressed_size)
    info.compression == "lz4" && return lz4_decompress(raw, info.uncompressed_size)
    error("unsupported MCAP chunk compression $(repr(info.compression))")
end

# Decompressed records of chunk `id`, through a small LRU cache (thread-safe).
function _chunk_records(f::MCAPFile, id::Int32)
    lock(f.lock) do
        k = findfirst(p -> p.first == id, f.cache)
        if k !== nothing
            p = f.cache[k]; deleteat!(f.cache, k); push!(f.cache, p)
            return p.second
        end
        info = f.chunks[id]
        out = _decompress(info, view(f.data[info.file], info.offset+1:info.offset+info.len))
        push!(f.cache, id => out)
        length(f.cache) > 6 && popfirst!(f.cache)
        return out
    end
end

"""
    message_data(f, ref) -> AbstractVector{UInt8}

Payload bytes of one indexed message (a view; copy it to keep it).
"""
function message_data(f::MCAPFile, m::MessageRef)
    buf = m.chunk == 0 ? f.data[m.file] : _chunk_records(f, m.chunk)
    return view(buf, m.offset+1:m.offset+m.len)
end

"Schema name of a channel (e.g. `\"nav_msgs/msg/Odometry\"`), or `\"\"`."
schema_name(f::MCAPFile, ch::UInt16) = (c = f.channels[ch]; s = get(f.schemas, c.schema_id, nothing);
                                        s === nothing ? "" : s.name)

"Topics with their schema names and message counts."
function topics(f::MCAPFile)
    counts = Dict{UInt16, Int}()
    for m in f.messages
        counts[m.channel] = get(counts, m.channel, 0) + 1
    end
    sort([(topic=c.topic, type=schema_name(f, id), count=get(counts, id, 0), channel=id)
          for (id, c) in f.channels]; by = x -> x.topic)
end

time_range(f::MCAPFile) = isempty(f.messages) ? (0, 0) : (f.messages[1].log_time, f.messages[end].log_time)

function Base.show(io::IO, ::MIME"text/plain", f::MCAPFile)
    t0, t1 = time_range(f)
    nf = length(f.files) > 1 ? " ($(length(f.files)) files)" : ""
    println(io, "MCAPFile $(basename(rstrip(f.path, '/')))$nf: $(length(f.messages)) messages, ",
            @sprintf("%.2f s", (t1 - t0) / 1e9), ", $(length(f.chunks)) chunks")
    for t in topics(f)
        println(io, "  ", rpad(t.topic, 28), rpad(t.type, 34), t.count)
    end
end

# --- writer -----------------------------------------------------------------------

"""
    MCAPWriter(path; profile="ros2", compression="zstd", chunk_size=1 << 20)

Write an MCAP file: [`add_schema!`](@ref), [`add_channel!`](@ref),
[`write_message!`](@ref), then `close`. Messages are grouped into chunks
compressed with `compression` (`"zstd"`, `"lz4"` or `""`).
"""
mutable struct MCAPWriter
    io::IO
    compression::String
    chunk_size::Int
    chunk::IOBuffer
    chunk_start::Int64
    chunk_end::Int64
    nschemas::Int
    nchannels::Int
end

function _record(io::IO, op::UInt8, body::Vector{UInt8})
    write(io, op, htol(UInt64(length(body))), body)
end
_wstr(io, s) = (b = codeunits(String(s)); write(io, htol(UInt32(length(b))), b))

function MCAPWriter(path::AbstractString; profile::AbstractString="ros2", compression::AbstractString="zstd",
                    chunk_size::Integer=1 << 20)
    compression in ("", "zstd", "lz4") || throw(ArgumentError("compression must be \"\", \"zstd\" or \"lz4\""))
    io = open(path, "w")
    write(io, MCAP_MAGIC)
    b = IOBuffer(); _wstr(b, profile); _wstr(b, "SpatialRender.jl")
    _record(io, Op.HEADER, take!(b))
    MCAPWriter(io, String(compression), Int(chunk_size), IOBuffer(), typemax(Int64), 0, 0, 0)
end

"Register a schema (e.g. `\"sensor_msgs/msg/PointCloud2\"`, encoding `\"ros2msg\"`); returns its id."
function add_schema!(w::MCAPWriter, name::AbstractString, encoding::AbstractString="ros2msg",
                     data::AbstractVector{UInt8}=UInt8[])
    w.nschemas += 1
    b = IOBuffer(); write(b, htol(UInt16(w.nschemas))); _wstr(b, name); _wstr(b, encoding)
    write(b, htol(UInt32(length(data))), data)
    _record(w.io, Op.SCHEMA, take!(b))
    return UInt16(w.nschemas)
end

"Register a channel on `topic` using schema `schema_id`; returns its id."
function add_channel!(w::MCAPWriter, topic::AbstractString, schema_id::Integer;
                      message_encoding::AbstractString="cdr", metadata=Dict{String, String}())
    w.nchannels += 1
    b = IOBuffer(); write(b, htol(UInt16(w.nchannels)), htol(UInt16(schema_id)))
    _wstr(b, topic); _wstr(b, message_encoding)
    m = IOBuffer(); for (k, v) in metadata; _wstr(m, k); _wstr(m, v); end
    mb = take!(m); write(b, htol(UInt32(length(mb))), mb)
    _record(w.io, Op.CHANNEL, take!(b))
    return UInt16(w.nchannels)
end

"Append a message (log time in nanoseconds)."
function write_message!(w::MCAPWriter, channel::Integer, log_time::Integer, data::AbstractVector{UInt8};
                        publish_time::Integer=log_time, sequence::Integer=0)
    b = IOBuffer()
    write(b, htol(UInt16(channel)), htol(UInt32(sequence)), htol(UInt64(log_time)), htol(UInt64(publish_time)), data)
    _record(w.chunk, Op.MESSAGE, take!(b))
    w.chunk_start = min(w.chunk_start, Int64(log_time)); w.chunk_end = max(w.chunk_end, Int64(log_time))
    position(w.chunk) >= w.chunk_size && _flush_chunk!(w)
    return w
end

function _flush_chunk!(w::MCAPWriter)
    position(w.chunk) == 0 && return
    raw = take!(w.chunk)
    comp = w.compression == "zstd" ? zstd_compress(raw) :
           w.compression == "lz4"  ? lz4_compress(raw) : raw
    b = IOBuffer()
    write(b, htol(UInt64(w.chunk_start)), htol(UInt64(w.chunk_end)), htol(UInt64(length(raw))),
          htol(UInt32(0)))                                       # crc 0 = not computed (allowed)
    _wstr(b, w.compression)
    write(b, htol(UInt64(length(comp))), comp)
    _record(w.io, Op.CHUNK, take!(b))
    w.chunk_start = typemax(Int64); w.chunk_end = 0
end

function Base.close(w::MCAPWriter)
    _flush_chunk!(w)
    _record(w.io, Op.DATA_END, collect(reinterpret(UInt8, [htol(UInt32(0))])))
    _record(w.io, Op.FOOTER, zeros(UInt8, 20))                    # no summary section
    write(w.io, MCAP_MAGIC)
    close(w.io)
end
