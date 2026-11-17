# Local Cartesian frames.
#
# GPUs work in Float32, which has ~7 significant digits: a UTM easting of
# 500_000 m only resolves to ~3 cm, and ECEF coordinates (~6.4e6 m) to ~0.5 m.
# Every layer is therefore moved into a local frame near the sensor origin in
# Float64 on the CPU, and only the small local offsets are stored as Float32.

"""
A mapping from source coordinates (Float64) to the scene's local Cartesian
frame (metres, Z up).
"""
abstract type LocalFrame end

"""
    IdentityFrame()

Coordinates are already local (CAD parts, simulated data).
"""
struct IdentityFrame <: LocalFrame end
to_local(::IdentityFrame, p) = Vec3d(p[1], p[2], length(p) > 2 ? p[3] : 0.0)
to_world(::IdentityFrame, p) = Vec3d(p)

"""
    OffsetFrame(origin)

For projected coordinates (UTM, state plane, LAS files): subtract `origin`.
"""
struct OffsetFrame <: LocalFrame
    origin::Vec3d
end
OffsetFrame(x, y, z=0.0) = OffsetFrame(Vec3d(x, y, z))
to_local(f::OffsetFrame, p) = Vec3d(p[1], p[2], length(p) > 2 ? p[3] : 0.0) - f.origin
to_world(f::OffsetFrame, p) = Vec3d(p) + f.origin

# WGS84 ellipsoid
const WGS84_A = 6378137.0
const WGS84_F = 1 / 298.257223563
const WGS84_E2 = WGS84_F * (2 - WGS84_F)

"""
    geodetic_to_ecef(lat, lon, h) -> Vec3d

WGS84 latitude / longitude in degrees and ellipsoidal height in metres to
Earth-centred, Earth-fixed coordinates.
"""
function geodetic_to_ecef(lat, lon, h=0.0)
    φ, λ = deg2rad(lat), deg2rad(lon)
    sφ, cφ = sincos(φ)
    sλ, cλ = sincos(λ)
    N = WGS84_A / sqrt(1 - WGS84_E2 * sφ^2)
    return Vec3d((N + h) * cφ * cλ, (N + h) * cφ * sλ, (N * (1 - WGS84_E2) + h) * sφ)
end

"""
    ecef_to_geodetic(p) -> (lat, lon, h)

Inverse of [`geodetic_to_ecef`](@ref) (Bowring's method with two refinement
iterations; sub-millimetre near the Earth's surface).
"""
function ecef_to_geodetic(p)
    x, y, z = p
    λ = atan(y, x)
    r = hypot(x, y)
    b = WGS84_A * (1 - WGS84_F)
    ep2 = (WGS84_A^2 - b^2) / b^2
    θ = atan(z * WGS84_A, r * b)
    φ = atan(z + ep2 * b * sin(θ)^3, r - WGS84_E2 * WGS84_A * cos(θ)^3)
    local N
    for _ in 1:2
        N = WGS84_A / sqrt(1 - WGS84_E2 * sin(φ)^2)
        h = r / cos(φ) - N
        φ = atan(z, r * (1 - WGS84_E2 * N / (N + h)))
    end
    N = WGS84_A / sqrt(1 - WGS84_E2 * sin(φ)^2)
    h = abs(φ) < deg2rad(89) ? r / cos(φ) - N : z / sin(φ) - N * (1 - WGS84_E2)
    return (rad2deg(φ), rad2deg(λ), h)
end

"Rotation from ECEF into East-North-Up at (lat, lon) in degrees."
function enu_rotation(lat, lon)
    sφ, cφ = sincosd(lat)
    sλ, cλ = sincosd(lon)
    return SMatrix{3, 3, Float64, 9}(-sλ, -sφ * cλ, cφ * cλ,
                                      cλ, -sφ * sλ, cφ * sλ,
                                      0.0, cφ, sφ)
end

"""
    ENUFrame(lat0, lon0, h0=0)

Local East-North-Up tangent frame at a geodetic origin (degrees, metres).
Source points are `(lon, lat[, h])`, the GeoJSON axis order.
"""
struct ENUFrame <: LocalFrame
    lat0::Float64
    lon0::Float64
    h0::Float64
    origin_ecef::Vec3d
    R::SMatrix{3, 3, Float64, 9}
end
ENUFrame(lat0, lon0, h0=0.0) =
    ENUFrame(lat0, lon0, h0, geodetic_to_ecef(lat0, lon0, h0), enu_rotation(lat0, lon0))

ecef_to_enu(f::ENUFrame, p) = f.R * (Vec3d(p) - f.origin_ecef)
enu_to_ecef(f::ENUFrame, e) = f.R' * Vec3d(e) + f.origin_ecef
to_local(f::ENUFrame, p) = ecef_to_enu(f, geodetic_to_ecef(p[2], p[1], length(p) > 2 ? p[3] : 0.0))
function to_world(f::ENUFrame, e)
    lat, lon, h = ecef_to_geodetic(enu_to_ecef(f, e))
    return Vec3d(lon, lat, h)
end

to_local32(f::LocalFrame, p) = Vec3f(to_local(f, p))
