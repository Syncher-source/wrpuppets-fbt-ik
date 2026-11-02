#pragma once

// Vectors and quaternions for the full-body IK (src/Fbt): plain structs, no game or engine types, so the same code
// builds into any engine and into offline tools. Quaternions are (x, y, z, w), unit, and rotate
// a vector as q * v * conj(q). Model space is the avatar's: x right, y forward, z up, metres.
#include <cmath>

namespace wrpuppets::fbt {

struct Vec3 {
    float x = 0, y = 0, z = 0;
};
struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float Length(Vec3 a) { return std::sqrt(Dot(a, a)); }
inline Vec3 Normalize(Vec3 a) {
    const float l = Length(a);
    return l > 1e-9f ? a * (1.0f / l) : Vec3{};
}

inline Quat operator*(Quat a, Quat b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quat Conjugate(Quat q) { return {-q.x, -q.y, -q.z, q.w}; }
inline Quat Normalize(Quat q) {
    const float l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return l > 1e-9f ? Quat{q.x / l, q.y / l, q.z / l, q.w / l} : Quat{};
}
inline Vec3 Rotate(Quat q, Vec3 v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = Cross(u, v) * 2.0f;
    return v + t * q.w + Cross(u, t);
}
// The rotation whose matrix has these columns (an orthonormal, right-handed basis).
inline Quat FromBasis(Vec3 x, Vec3 y, Vec3 z) {
    const float trace = x.x + y.y + z.z;
    Quat q;
    if (trace > 0) {
        const float s = std::sqrt(trace + 1.0f) * 2;
        q = {(y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s, 0.25f * s};
    } else if (x.x > y.y && x.x > z.z) {
        const float s = std::sqrt(1.0f + x.x - y.y - z.z) * 2;
        q = {0.25f * s, (y.x + x.y) / s, (z.x + x.z) / s, (y.z - z.y) / s};
    } else if (y.y > z.z) {
        const float s = std::sqrt(1.0f + y.y - x.x - z.z) * 2;
        q = {(y.x + x.y) / s, 0.25f * s, (z.y + y.z) / s, (z.x - x.z) / s};
    } else {
        const float s = std::sqrt(1.0f + z.z - x.x - y.y) * 2;
        q = {(z.x + x.z) / s, (z.y + y.z) / s, 0.25f * s, (x.y - y.x) / s};
    }
    return Normalize(q);
}
// The rotation taking two directions `along` and `side` (side made perpendicular to along) of one frame onto
// `alongTo` and `sideTo` of another: aims a bone and sets its roll in one go.
inline Quat AimRoll(Vec3 along, Vec3 side, Vec3 alongTo, Vec3 sideTo) {
    auto frame = [](Vec3 a, Vec3 s) {
        const Vec3 x = Normalize(a), z = Normalize(Cross(x, s)), y = Cross(z, x);
        return FromBasis(x, y, z);
    };
    const Quat from = frame(along, side), to = frame(alongTo, sideTo);
    return Normalize(to * Quat{-from.x, -from.y, -from.z, from.w});
}

// The shortest rotation taking direction a onto direction b.
inline Quat FromTo(Vec3 a, Vec3 b) {
    a = Normalize(a);
    b = Normalize(b);
    const float d = Dot(a, b);
    if (d < -0.999999f) {   // opposite: any axis perpendicular to a
        Vec3 axis = Cross({1, 0, 0}, a);
        if (Length(axis) < 1e-3f) axis = Cross({0, 1, 0}, a);
        axis = Normalize(axis);
        return {axis.x, axis.y, axis.z, 0};
    }
    const Vec3 c = Cross(a, b);
    return Normalize(Quat{c.x, c.y, c.z, 1.0f + d});
}

inline Quat AxisAngle(Vec3 axis, float radians) {
    const Vec3 a = Normalize(axis);
    const float s = std::sin(radians * 0.5f);
    return {a.x * s, a.y * s, a.z * s, std::cos(radians * 0.5f)};
}

}  // namespace wrpuppets::fbt
