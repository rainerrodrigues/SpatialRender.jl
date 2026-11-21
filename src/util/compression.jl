# One-shot (de)compression through the C libraries directly: zstd and lz4
# (frame format) for MCAP chunks, zlib for PNG. No codec wrapper packages.

using Zstd_jll: libzstd
using Lz4_jll: liblz4
using Zlib_jll: libz

function zstd_compress(src::AbstractVector{UInt8}; level::Integer=3)
    cap = ccall((:ZSTD_compressBound, libzstd), Csize_t, (Csize_t,), length(src))
    dst = Vector{UInt8}(undef, cap)
    n = GC.@preserve src dst ccall((:ZSTD_compress, libzstd), Csize_t,
        (Ptr{UInt8}, Csize_t, Ptr{UInt8}, Csize_t, Cint), dst, cap, src, length(src), level)
    _zstd_check(n)
    return resize!(dst, n)
end

"Decompress a zstd frame whose decompressed size is known (MCAP records it)."
function zstd_decompress(src::AbstractVector{UInt8}, size::Integer)
    dst = Vector{UInt8}(undef, size)
    n = GC.@preserve src dst ccall((:ZSTD_decompress, libzstd), Csize_t,
        (Ptr{UInt8}, Csize_t, Ptr{UInt8}, Csize_t), dst, size, src, length(src))
    _zstd_check(n)
    n == size || error("zstd: decompressed $n bytes, expected $size")
    return dst
end

function _zstd_check(code)
    if ccall((:ZSTD_isError, libzstd), Cuint, (Csize_t,), code) != 0
        error("zstd: " * unsafe_string(ccall((:ZSTD_getErrorName, libzstd), Cstring, (Csize_t,), code)))
    end
end

function lz4_compress(src::AbstractVector{UInt8})
    cap = ccall((:LZ4F_compressFrameBound, liblz4), Csize_t, (Csize_t, Ptr{Cvoid}), length(src), C_NULL)
    dst = Vector{UInt8}(undef, cap)
    n = GC.@preserve src dst ccall((:LZ4F_compressFrame, liblz4), Csize_t,
        (Ptr{UInt8}, Csize_t, Ptr{UInt8}, Csize_t, Ptr{Cvoid}), dst, cap, src, length(src), C_NULL)
    _lz4_check(n)
    return resize!(dst, n)
end

"Decompress an LZ4 frame whose decompressed size is known."
function lz4_decompress(src::AbstractVector{UInt8}, size::Integer)
    ctx = Ref{Ptr{Cvoid}}(C_NULL)
    _lz4_check(ccall((:LZ4F_createDecompressionContext, liblz4), Csize_t, (Ref{Ptr{Cvoid}}, Cuint), ctx, 100))
    dst = Vector{UInt8}(undef, size)
    try
        inpos = 0; outpos = 0
        GC.@preserve src dst while inpos < length(src)
            dlen = Ref{Csize_t}(size - outpos); slen = Ref{Csize_t}(length(src) - inpos)
            hint = ccall((:LZ4F_decompress, liblz4), Csize_t,
                (Ptr{Cvoid}, Ptr{UInt8}, Ref{Csize_t}, Ptr{UInt8}, Ref{Csize_t}, Ptr{Cvoid}),
                ctx[], pointer(dst) + outpos, dlen, pointer(src) + inpos, slen, C_NULL)
            _lz4_check(hint)
            inpos += slen[]; outpos += dlen[]
            hint == 0 && break                                   # frame complete
            slen[] == 0 && dlen[] == 0 && error("lz4: no progress (truncated frame?)")
        end
        outpos == size || error("lz4: decompressed $outpos bytes, expected $size")
    finally
        ccall((:LZ4F_freeDecompressionContext, liblz4), Csize_t, (Ptr{Cvoid},), ctx[])
    end
    return dst
end

function _lz4_check(code)
    if ccall((:LZ4F_isError, liblz4), Cuint, (Csize_t,), code) != 0
        error("lz4: " * unsafe_string(ccall((:LZ4F_getErrorName, liblz4), Cstring, (Csize_t,), code)))
    end
end

"zlib stream (header, deflate, Adler-32), as PNG's IDAT chunk expects."
function zlib_compress(src::AbstractVector{UInt8}; level::Integer=6)
    cap = Ref{Culong}(ccall((:compressBound, libz), Culong, (Culong,), length(src)))
    dst = Vector{UInt8}(undef, cap[])
    rc = GC.@preserve src dst ccall((:compress2, libz), Cint,
        (Ptr{UInt8}, Ref{Culong}, Ptr{UInt8}, Culong, Cint), dst, cap, src, length(src), level)
    rc == 0 || error("zlib compress2 failed ($rc)")
    return resize!(dst, cap[])
end
