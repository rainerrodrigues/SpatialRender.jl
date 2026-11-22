# PNG writer (zlib from Julia's own Zlib_jll; no image package).

const _CRC_TABLE = let t = Vector{UInt32}(undef, 256)
    for n in 0:255
        c = UInt32(n)
        for _ in 1:8
            c = (c & 1) != 0 ? 0xedb88320 ⊻ (c >> 1) : c >> 1
        end
        t[n+1] = c
    end
    t
end
function _crc32(data)
    c = 0xffffffff
    for b in data
        c = _CRC_TABLE[((c ⊻ b) & 0xff)+1] ⊻ (c >> 8)
    end
    return c ⊻ 0xffffffff
end

"""
    write_png(path, img::Matrix{NTuple{3,UInt8}}) -> path
"""
function write_png(path::AbstractString, img::AbstractMatrix{NTuple{3, UInt8}})
    h, w = size(img)
    raw = Vector{UInt8}(undef, h * (1 + 3w))
    k = 0
    for y in 1:h
        # filter "Up" (byte minus the byte above) for all rows but the first:
        # rendered images have large flat areas, which then compress well
        raw[k += 1] = y == 1 ? 0x00 : 0x02
        for x in 1:w, c in 1:3
            v = img[y, x][c]
            raw[k += 1] = y == 1 ? v : v - img[y-1, x][c]
        end
    end
    idat = zlib_compress(raw; level=6)
    chunk(io, typ, data) = (write(io, hton(UInt32(length(data)))); td = vcat(Vector{UInt8}(typ), data);
                            write(io, td); write(io, hton(_crc32(td))))
    open(path, "w") do io
        write(io, UInt8[0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a])
        ihdr = IOBuffer()
        write(ihdr, hton(UInt32(w)), hton(UInt32(h)), UInt8(8), UInt8(2), UInt8(0), UInt8(0), UInt8(0))
        chunk(io, "IHDR", take!(ihdr))
        chunk(io, "IDAT", idat)
        chunk(io, "IEND", UInt8[])
    end
    return path
end
