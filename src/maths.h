#pragma once

#include "api.h"
#include "assertion.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace vtrz
{

struct Constants
{
    static constexpr float pi = 3.14159265359f;
    static constexpr float twoPi = 2.0f * pi;
    static constexpr float halfPi = 0.5f * pi;
    static constexpr float quarterPi = 0.5f * halfPi;
    static constexpr float invPi = 1.0f / pi;
    static constexpr float invTwoPi = 0.5f * invPi;
    static constexpr float sqrt2 = 1.41421356237f;
    static constexpr float inf = std::numeric_limits<float>::infinity();
};

constexpr float degToRad(float deg) {
    return deg / 180.0f * Constants::pi;
}

constexpr float radToDeg(float rad) {
    return rad * Constants::invPi * 180.0f;
}

template <typename T>
constexpr T clamp(const T &x, const T &min, const T &max) {
    return std::max(min, std::min(x, max));
}

template <typename T>
constexpr T saturate(const T &x) {
    return clamp(x, T(0.0), T(1.0));
}

template <typename T, typename U>
constexpr T lerp(const T &a, const T &b, const U &t) {
    return a * (U(1.0) - t) + b * t;
}

template <typename T>
constexpr T smoothstep(const T &min, const T &max, const T &value) {
    T t = saturate((value - min) / (max - min));
    return t * t * (3.0f - 2.0f * t);
}

template <typename T>
inline T square(const T &x) {
    return x * x;
}

inline float frac(float s) {
    return s - std::floor(s);
}

constexpr bool isPowerOf2(uint32_t x) {
    return x && !(x & (x - 1));
}

constexpr uint32_t nextPowerOf2U32(uint32_t x) {
    --x;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    ++x;
    return x;
}

inline uint32_t log2U32(uint32_t x) {
    uint32_t r; // result of log2(v) will go here
    uint32_t shift;

    r = (x > 0xFFFF) << 4; x >>= r;
    shift = (x > 0xFF) << 3; x >>= shift; r |= shift;
    shift = (x > 0xF) << 2; x >>= shift; r |= shift;
    shift = (x > 0x3) << 1; x >>= shift; r |= shift;
    r |= (x >> 1);

    return r;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

struct VECTORIZER_API Vector2
{
    float x, y;

    static constexpr uint32_t kDim = 2;

    constexpr Vector2() = default;
    explicit constexpr Vector2(float val) : x(val), y(val) {}
    explicit constexpr Vector2(float x, float y) : x(x), y(y) {}

    float operator[](uint32_t dim) const {
        ASSERT(dim < kDim);
        return *(reinterpret_cast<const float *>(this) + dim);
    }

    float &operator[](uint32_t dim) {
        ASSERT(dim < kDim);
        return *(reinterpret_cast<float *>(this) + dim);
    }

    bool operator==(const Vector2 &other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vector2 &other) const {
        return !(*this == other);
    }

    Vector2 &operator+=(float s) {
        x += s;
        y += s;
        return *this;
    }

    Vector2 &operator+=(const Vector2 &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2 &operator-=(float s) {
        x -= s;
        y -= s;
        return *this;
    }

    Vector2 &operator-=(const Vector2 &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2 &operator*=(float s) {
        x *= s;
        y *= s;
        return *this;
    }

    Vector2 &operator*=(const Vector2 &other) {
        x *= other.x;
        y *= other.y;
        return *this;
    }

    Vector2 &operator/=(float s) {
        x /= s;
        y /= s;
        return *this;
    }

    Vector2 &operator/=(const Vector2 &v) {
        x /= v.x;
        y /= v.y;
        return *this;
    }
};

inline Vector2 operator+(Vector2 v, float s) {
    v += s;
    return v;
}

inline Vector2 operator+(Vector2 v0, const Vector2 &v1) {
    v0 += v1;
    return v0;
}

inline Vector2 operator-(Vector2 v, float s) {
    v -= s;
    return v;
}

inline Vector2 operator-(Vector2 v0, const Vector2 &v1) {
    v0 -= v1;
    return v0;
}

inline Vector2 operator*(Vector2 v, float s) {
    v *= s;
    return v;
}

inline Vector2 operator*(float s, const Vector2 &v) {
    return v * s;
}

inline Vector2 operator*(Vector2 v0, const Vector2 &v1) {
    v0 *= v1;
    return v0;
}

inline Vector2 operator/(Vector2 v, float s) {
    v /= s;
    return v;
}

inline Vector2 operator/(Vector2 v0, const Vector2 &v1) {
    v0 /= v1;
    return v0;
}

inline float dot(const Vector2 &v0, const Vector2 &v1) {
    return v0.x * v1.x + v0.y * v1.y;
}

inline float cross(const Vector2 &v0, const Vector2 &v1) {
    return v0.x * v1.y - v0.y * v1.x;
}

inline float length(const Vector2 &v) {
    return std::sqrt(dot(v, v));
}

inline float length2(const Vector2 &v) {
    return dot(v, v);
}

inline Vector2 clamp(const Vector2 &x, const Vector2 &min, const Vector2 &max) {
    Vector2 ret;
    ret.x = std::max(min.x, std::min(x.x, max.x));
    ret.y = std::max(min.y, std::min(x.y, max.y));
    return ret;
}

inline Vector2 saturate(const Vector2 &x) {
    Vector2 ret;
    ret.x = saturate(x.x);
    ret.y = saturate(x.y);
    return ret;
}

inline Vector2 floor(const Vector2 &v) {
    return Vector2(std::floor(v.x), std::floor(v.y));
}

inline Vector2 frac(const Vector2 &v) {
    return v - floor(v);
}

inline bool isNan(const Vector2 &v) {
    return std::isnan(v.x) || std::isnan(v.y);
}

inline Vector2 min(const Vector2 &v0, const Vector2 &v1) {
    Vector2 ret;
    ret.x = std::min(v0.x, v1.x);
    ret.y = std::min(v0.y, v1.y);
    return ret;
}

inline Vector2 max(const Vector2 &v0, const Vector2 &v1) {
    Vector2 ret;
    ret.x = std::max(v0.x, v1.x);
    ret.y = std::max(v0.y, v1.y);
    return ret;
}

struct VECTORIZER_API Vector3
{
    float x, y, z;

    static constexpr uint32_t kDim = 3;

    constexpr Vector3() = default;
    explicit constexpr Vector3(float val) : x(val), y(val), z(val) {}
    explicit constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    float operator[](uint32_t dim) const {
        ASSERT(dim < kDim);
        return *(reinterpret_cast<const float *>(this) + dim);
    }

    float &operator[](uint32_t dim) {
        ASSERT(dim < kDim);
        return *(reinterpret_cast<float *>(this) + dim);
    }

    bool operator==(const Vector3 &other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const Vector3 &other) const {
        return !(*this == other);
    }

    Vector3 &operator+=(const float s) {
        x += s;
        y += s;
        z += s;
        return *this;
    }

    Vector3 &operator-=(const float s) {
        x -= s;
        y -= s;
        z -= s;
        return *this;
    }

    Vector3 &operator+=(const Vector3 &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3 operator-() const {
        return Vector3(-x, -y, -z);
    }

    Vector3 &operator-=(const Vector3 &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3 &operator*=(const float s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vector3 &operator*=(const Vector3 &v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }

    Vector3 &operator/=(const float s) {
        float invS = 1.0f / s;
        return *this *= invS;
    }

    Vector3 &operator/=(const Vector3 &v) {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        return *this;
    }

    float maxComponent() const {
        return std::max(x, std::max(y, z));
    }

    explicit operator Vector2() const {
        return Vector2(x, y);
    }
};

inline Vector3 operator+(Vector3 v0, const Vector3 &v1) {
    v0 += v1;
    return v0;
}

inline Vector3 operator+(Vector3 v, float s) {
    v += s;
    return v;
}

inline Vector3 operator-(Vector3 v0, const Vector3 &v1) {
    v0 -= v1;
    return v0;
}

inline Vector3 operator-(Vector3 v, float s) {
    v -= s;
    return v;
}

inline Vector3 operator+(float s, Vector3 v) {
    return v + s;
}

inline Vector3 operator*(Vector3 v, float s) {
    v *= s;
    return v;
}

inline Vector3 operator*(Vector3 v0, const Vector3 &v1) {
    v0 *= v1;
    return v0;
}

inline Vector3 operator*(float s, const Vector3 &v) {
    return v * s;
}

inline Vector3 operator/(Vector3 v, const float s) {
    v /= s;
    return v;
}

inline Vector3 operator/(Vector3 v0, const Vector3 &v1) {
    v0 /= v1;
    return v0;
}

inline Vector3 operator/(float s, const Vector3 &v) {
    return Vector3(s) / v;
}

inline float dot(const Vector3 &v0, const Vector3 &v1) {
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

inline float absDot(const Vector3 &v0, const Vector3 &v1) {
    return std::abs(dot(v0, v1));
}

inline Vector3 cross(const Vector3 &v0, const Vector3 &v1) {
    return Vector3(
        v0.y * v1.z - v0.z * v1.y,
        v0.z * v1.x - v0.x * v1.z,
        v0.x * v1.y - v0.y * v1.x
    );
}

inline float length(const Vector3 &v) {
    return std::sqrt(dot(v, v));
}

inline float length2(const Vector3 &v) {
    return dot(v, v);
}

inline Vector3 normalize(const Vector3 &v) {
    float len = length(v);
    ASSERT(len > 0.0f);
    return v * (1.0f / len);
}

inline float unitAngle(const Vector3 &u, const Vector3 &v) {
    if (dot(u, v) < 0.0f)
        return Constants::pi - 2.0f * std::asin(0.5f * length(v + u));
    else
        return 2.0f * std::asin(0.5f * length(v - u));
}

inline Vector3 min(const Vector3 &v0, const Vector3 &v1) {
    Vector3 ret;
    ret.x = std::min(v0.x, v1.x);
    ret.y = std::min(v0.y, v1.y);
    ret.z = std::min(v0.z, v1.z);
    return ret;
}

inline Vector3 max(const Vector3 &v0, const Vector3 &v1) {
    Vector3 ret;
    ret.x = std::max(v0.x, v1.x);
    ret.y = std::max(v0.y, v1.y);
    ret.z = std::max(v0.z, v1.z);
    return ret;
}

inline Vector3 clamp(const Vector3 &x, const Vector3 &min, const Vector3 &max) {
    Vector3 ret;
    ret.x = std::max(min.x, std::min(x.x, max.x));
    ret.y = std::max(min.y, std::min(x.y, max.y));
    ret.z = std::max(min.z, std::min(x.z, max.z));
    return ret;
}

inline Vector3 saturate(const Vector3 &x) {
    Vector3 ret;
    ret.x = saturate(x.x);
    ret.y = saturate(x.y);
    ret.z = saturate(x.z);
    return ret;
}

inline Vector3 floor(const Vector3 &v) {
    return Vector3(std::floor(v.x), std::floor(v.y), std::floor(v.z));
}

inline Vector3 frac(const Vector3 &v) {
    return v - floor(v);
}

inline Vector3 abs(const Vector3 &v) {
    return Vector3(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

inline Vector3 sqrt(const Vector3 &v) {
    return Vector3(std::sqrt(v.x), std::sqrt(v.y), std::sqrt(v.z));
}

inline Vector3 reflect(const Vector3 &wo)
{
    return Vector3(-wo.x, -wo.y, wo.z);
}

inline Vector3 reflect(const Vector3 &v, const Vector3 &n) {
    return  2.0f * dot(v, n) * n - v;
}

inline void orthonormalBasis(const Vector3 &N, Vector3 &X, Vector3 &Y)
{
    if (N.z < -0.9999999f) {
        X = Vector3(0.0f, -1.0f, 0.0f);
        Y = Vector3(-1.0f, 0.0f, 0.0f);
    } else {
        const float a = 1.0f / (1.0f + N.z);
        const float b = -N.x * N.y * a;
        X = Vector3(1.0f - N.x * N.x * a, b, -N.x);
        Y = Vector3(b, 1.0f - N.y * N.y * a, -N.y);
    }
}

inline bool isNan(const Vector3 &v)
{
    return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z);
}

inline bool isFinite(const Vector3 &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool isNonNegativeFinite(const Vector3 &v)
{
    return isFinite(v) && v.x >= 0.0f && v.y >= 0.0f && v.z >= 0.0f;
}

inline float srgbToLinear(float x)
{
    if (x < 0.04045f) {
        return x / 12.92f;
    } else {
        return std::pow((x + 0.055f) / 1.055f, 2.4f);
    }
}

inline Vector3 srgbToLinear(const Vector3 &v)
{
    return Vector3(srgbToLinear(v.x), srgbToLinear(v.y), srgbToLinear(v.z));
}

static inline float linearToSrgb(float x)
{
    if (x > 0.0031308f) {
        return 1.055f * (std::pow(x, (1.0f / 2.4f))) - 0.055f;
    } else {
        return 12.92f * x;
    }
}

inline Vector3 linearToSrgb(const Vector3 &v)
{
    return Vector3(linearToSrgb(v.x), linearToSrgb(v.y), linearToSrgb(v.z));
}

struct VECTORIZER_API Vector4
{
    float x, y, z, w;

    Vector4() = default;
    explicit Vector4(float val) : x(val), y(val), z(val), w(val) {}
    Vector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vector4(const Vector3 &v3, float w) : x(v3.x), y(v3.y), z(v3.z), w(w) {}


    float operator[](uint32_t dim) const {
        ASSERT(dim <= 3);
        return *(reinterpret_cast<const float *>(this) + dim);
    }

    float &operator[](uint32_t dim) {
        ASSERT(dim <= 3);
        return *(reinterpret_cast<float *>(this) + dim);
    }

    bool operator==(const Vector4 &other) const {
        return x == other.x && y == other.y && z == other.z && w == other.w;
    }

    bool operator!=(const Vector4 &other) const {
        return !(*this == other);
    }

    Vector4 &operator+=(float s) {
        x += s;
        y += s;
        z += s;
        w += s;
        return *this;
    }

    Vector4 &operator+=(const Vector4 &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }

    Vector4 &operator*=(const float s) {
        x *= s;
        y *= s;
        z *= s;
        w *= s;
        return *this;
    }

    Vector4 &operator/=(const float s) {
        float invS = 1.0f / s;
        return *this *= invS;
    }

    explicit operator Vector3() const {
        return Vector3(x, y, z);
    }
};

inline Vector4 operator+(Vector4 v, const float s) {
    v += s;
    return v;
}

inline Vector4 operator+(const float s, const Vector4 &v) {
    return v + s;
}

inline Vector4 operator+(Vector4 v0, const Vector4 &v1) {
    v0 += v1;
    return v0;
}

inline Vector4 operator*(Vector4 v, const float s) {
    v *= s;
    return v;
}

inline Vector4 operator*(const float s, const Vector4 &v) {
    return v * s;
}

inline Vector4 operator/(Vector4 v, const float s) {
    v /= s;
    return v;
}

inline float dot(const Vector4 &v0, const Vector4 &v1) {
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z + v0.w * v1.w;
}

struct VECTORIZER_API Matrix2x2
{
    // Row-major storage.
    union
    {
        Vector2 rows[2];
        float arr[2][2];
        struct {
            float
                m00, m01,
                m10, m11;
        } m;
    };

    explicit Matrix2x2(float diag = 1.0f) {
        for (uint32_t i = 0; i < 2; ++i)
            for (uint32_t j = 0; j < 2; ++j)
                arr[i][j] = i == j ? diag : 0.0f;
    }

    Matrix2x2(float m00, float m01, float m10, float m11) {
        m.m00 = m00; m.m01 = m01; m.m10 = m10; m.m11 = m11;
    }

    inline Matrix2x2 &operator*=(float other)
    {
        rows[0] *= other; rows[1] *= other;
        return *this;
    }

    float determinant() const
    {
        return m.m00 * m.m11 - m.m01 * m.m10;
    }

    Matrix2x2 inverse() const
    {
        // Assume the matrix is invertible.
        float det = determinant();
        Matrix2x2 ret = Matrix2x2(
            m.m11, -m.m01,
            -m.m10, m.m00);
        ret *= (1.0f / det);
        return ret;
    }

};

struct VECTORIZER_API Matrix3x3
{
    // Row-major storage.
    union
    {
        Vector3 rows[3];
        float arr[3][3];
        struct {
            float
                m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22;
        } m;
    };

    explicit Matrix3x3(float diag = 1.0f) {
        for (uint32_t i = 0; i < 3; ++i)
            for (uint32_t j = 0; j < 3; ++j)
                arr[i][j] = i == j ? diag : 0.0f;
    }

    Matrix3x3(const Vector3 &row0, const Vector3 &row1, const Vector3 &row2) {
        rows[0] = row0; rows[1] = row1; rows[2] = row2;
    }

    explicit Matrix3x3(const Vector3 *rs) {
        memcpy(rows, rs, sizeof(Vector3) * 3);
    }

    explicit Matrix3x3(const float *a) {
        memcpy(arr, a, sizeof(float) * 9);
    }

    Matrix3x3(
        float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22) {
        m.m00 = m00; m.m01 = m01; m.m02 = m02;
        m.m10 = m10; m.m11 = m11; m.m12 = m12;
        m.m20 = m20; m.m21 = m21; m.m22 = m22;
    }
};

struct VECTORIZER_API Matrix4x4
{
    // Row-major storage.
    union
    {
        Vector4 rows[4];
        float arr[4][4];
        struct {
            float
                m00, m01, m02, m03,
                m10, m11, m12, m13,
                m20, m21, m22, m23,
                m30, m31, m32, m33;
        } m;
    };

    explicit Matrix4x4(float diag = 1.0f) {
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                arr[i][j] = i == j ? diag : 0.0f;
    }

    Matrix4x4(const Vector4 &row0, const Vector4 &row1, const Vector4 &row2, const Vector4 &row3) {
        rows[0] = row0; rows[1] = row1; rows[2] = row2; rows[3] = row3;
    }

    explicit Matrix4x4(const Vector4 *rs) {
        memcpy(rows, rs, sizeof(Vector4) * 4);
    }

    explicit Matrix4x4(const float *a) {
        memcpy(arr, a, sizeof(float) * 16);
    }

    Matrix4x4(
        float m00, float m01, float m02, float m03,
        float m10, float m11, float m12, float m13,
        float m20, float m21, float m22, float m23,
        float m30, float m31, float m32, float m33) {
        m.m00 = m00; m.m01 = m01; m.m02 = m02; m.m03 = m03;
        m.m10 = m10; m.m11 = m11; m.m12 = m12; m.m13 = m13;
        m.m20 = m20; m.m21 = m21; m.m22 = m22; m.m23 = m23;
        m.m30 = m30; m.m31 = m31; m.m32 = m32; m.m33 = m33;
    }

    Vector3 translation() const {
        return Vector3(m.m03, m.m13, m.m23);
    }

    Matrix4x4 transpose() const {
        Matrix4x4 t;
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                t.arr[i][j] = arr[j][i];
        return t;
    }

    Matrix4x4 inverse() const {
        // Closed-form solution: https://github.com/willnode/N-Matrix-Programmer.git
        // Assume the matrix is invertible.

        float A2323 = m.m22 * m.m33 - m.m23 * m.m32;
        float A1323 = m.m21 * m.m33 - m.m23 * m.m31;
        float A1223 = m.m21 * m.m32 - m.m22 * m.m31;
        float A0323 = m.m20 * m.m33 - m.m23 * m.m30;
        float A0223 = m.m20 * m.m32 - m.m22 * m.m30;
        float A0123 = m.m20 * m.m31 - m.m21 * m.m30;
        float A2313 = m.m12 * m.m33 - m.m13 * m.m32;
        float A1313 = m.m11 * m.m33 - m.m13 * m.m31;
        float A1213 = m.m11 * m.m32 - m.m12 * m.m31;
        float A2312 = m.m12 * m.m23 - m.m13 * m.m22;
        float A1312 = m.m11 * m.m23 - m.m13 * m.m21;
        float A1212 = m.m11 * m.m22 - m.m12 * m.m21;
        float A0313 = m.m10 * m.m33 - m.m13 * m.m30;
        float A0213 = m.m10 * m.m32 - m.m12 * m.m30;
        float A0312 = m.m10 * m.m23 - m.m13 * m.m20;
        float A0212 = m.m10 * m.m22 - m.m12 * m.m20;
        float A0113 = m.m10 * m.m31 - m.m11 * m.m30;
        float A0112 = m.m10 * m.m21 - m.m11 * m.m20;

        float invdet =
            m.m00 * (m.m11 * A2323 - m.m12 * A1323 + m.m13 * A1223)
            - m.m01 * (m.m10 * A2323 - m.m12 * A0323 + m.m13 * A0223)
            + m.m02 * (m.m10 * A1323 - m.m11 * A0323 + m.m13 * A0123)
            - m.m03 * (m.m10 * A1223 - m.m11 * A0223 + m.m12 * A0123);
        invdet = 1 / invdet;

        Matrix4x4 inv;
        inv.m.m00 = invdet * (m.m11 * A2323 - m.m12 * A1323 + m.m13 * A1223);
        inv.m.m01 = invdet * -(m.m01 * A2323 - m.m02 * A1323 + m.m03 * A1223);
        inv.m.m02 = invdet * (m.m01 * A2313 - m.m02 * A1313 + m.m03 * A1213);
        inv.m.m03 = invdet * -(m.m01 * A2312 - m.m02 * A1312 + m.m03 * A1212);
        inv.m.m10 = invdet * -(m.m10 * A2323 - m.m12 * A0323 + m.m13 * A0223);
        inv.m.m11 = invdet * (m.m00 * A2323 - m.m02 * A0323 + m.m03 * A0223);
        inv.m.m12 = invdet * -(m.m00 * A2313 - m.m02 * A0313 + m.m03 * A0213);
        inv.m.m13 = invdet * (m.m00 * A2312 - m.m02 * A0312 + m.m03 * A0212);
        inv.m.m20 = invdet * (m.m10 * A1323 - m.m11 * A0323 + m.m13 * A0123);
        inv.m.m21 = invdet * -(m.m00 * A1323 - m.m01 * A0323 + m.m03 * A0123);
        inv.m.m22 = invdet * (m.m00 * A1313 - m.m01 * A0313 + m.m03 * A0113);
        inv.m.m23 = invdet * -(m.m00 * A1312 - m.m01 * A0312 + m.m03 * A0112);
        inv.m.m30 = invdet * -(m.m10 * A1223 - m.m11 * A0223 + m.m12 * A0123);
        inv.m.m31 = invdet * (m.m00 * A1223 - m.m01 * A0223 + m.m02 * A0123);
        inv.m.m32 = invdet * -(m.m00 * A1213 - m.m01 * A0213 + m.m02 * A0113);
        inv.m.m33 = invdet * (m.m00 * A1212 - m.m01 * A0212 + m.m02 * A0112);

        return inv;
    }

    Matrix4x4 &operator+=(const Matrix4x4 &other) {
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                arr[i][j] += other.arr[i][j];

        return *this;
    }
};

inline Vector4 operator*(const Matrix4x4 &m, const Vector4 &v) {
    Vector4 ret;
    for (uint32_t i = 0; i < 4; ++i) {
        ret[i] = dot(m.rows[i], v);
    }
    return ret;
}

inline Matrix4x4 operator*(const Matrix4x4 &m1, const Matrix4x4 &m2) {
    Matrix4x4 ret;
    for (uint32_t i = 0; i < 4; ++i) {
        for (uint32_t j = 0; j < 4; ++j) {
            ret.arr[i][j] = 0;
            for (uint32_t k = 0; k < 4; ++k)
                ret.arr[i][j] += m1.arr[i][k] * m2.arr[k][j];
        }
    }
    return ret;
}

inline Matrix4x4 makePerspective(float near, float far, float aspect, float vFov)
{
    float invTanHalfVFov = 1.0f / std::tan(0.5f * vFov);
    const float n = near;
    const float f = far;
    float elements[16] = {
        invTanHalfVFov / aspect, 0.0f, 0.0f, 0.0f,
        0.0f, invTanHalfVFov, 0.0f, 0.0f,
        0.0f, 0.0f, f / (n - f), n * f / (n - f),
        0.0f, 0.0f, -1.0f, 0.0f,
    };
    Matrix4x4 proj(elements);
    return proj;
}

inline Matrix4x4 makeLookup(const Vector3 &position, Vector3 view, Vector3 up) {
    view = normalize(view);
    Vector3 right = cross(up, -view);
    if (length2(right) < 1e-8f) {
        orthonormalBasis(-view, right, up);
    } else {
        right = normalize(right);
        up = cross(-view, right);
    }
    return Matrix4x4(
        right.x, up.x, -view.x, position.x,
        right.y, up.y, -view.y, position.y,
        right.z, up.z, -view.z, position.z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

inline Matrix4x4 makeLookupView(const Vector3 &position, Vector3 view, Vector3 up) {
    view = normalize(view);
    Vector3 right = cross(up, -view);
    if (length2(right) < 1e-8f) {
        orthonormalBasis(-view, right, up);
    } else {
        right = normalize(right);
        up = cross(-view, right);
    }
    Vector3 t(dot(right, position), dot(up, position), dot(-view, position));
    return Matrix4x4(
        right.x, right.y, right.z, -t.x,
        up.x, up.y, up.z, -t.y,
        -view.x, -view.y, -view.z, -t.z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

inline Matrix4x4 makeTranslate(float tx, float ty, float tz) {
    Matrix4x4 translation(1.0f);
    translation.arr[0][3] = tx;
    translation.arr[1][3] = ty;
    translation.arr[2][3] = tz;
    return translation;
}

inline Matrix4x4 makeRotate(float angle, Vector3 axis) {
    Matrix4x4 rotation(1.0f);
    axis = normalize(axis);
    float sine = std::sin(angle);
    float cosine = std::cos(angle);

    rotation.arr[0][0] = axis.x * axis.x + (1.0f - axis.x * axis.x) * cosine;
    rotation.arr[0][1] = axis.x * axis.y * (1.0f - cosine) - axis.z * sine;
    rotation.arr[0][2] = axis.x * axis.z * (1.0f - cosine) + axis.y * sine;

    rotation.arr[1][0] = axis.x * axis.y * (1.0f - cosine) + axis.z * sine;
    rotation.arr[1][1] = axis.y * axis.y + (1.0f - axis.y * axis.y) * cosine;
    rotation.arr[1][2] = axis.y * axis.z * (1.0f - cosine) - axis.x * sine;

    rotation.arr[2][0] = axis.x * axis.z * (1.0f - cosine) - axis.y * sine;
    rotation.arr[2][1] = axis.y * axis.z * (1.0f - cosine) + axis.x * sine;
    rotation.arr[2][2] = axis.z * axis.z + (1.0f - axis.z * axis.z) * cosine;

    return rotation;
}

inline Matrix4x4 makeScale(float sx, float sy, float sz) {
    Matrix4x4 scale(1.0f);
    scale.arr[0][0] = sx;
    scale.arr[1][1] = sy;
    scale.arr[2][2] = sz;
    return scale;
}

struct VECTORIZER_API AABB2
{
    AABB2() = default;
    AABB2(const Vector2 &min, const Vector2 &max) : min(min), max(max) {}

    void expand(const Vector2 &point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
    }

    void expand(const AABB2 &aabb) {
        min.x = std::min(min.x, aabb.min.x);
        min.y = std::min(min.y, aabb.min.y);

        max.x = std::max(max.x, aabb.max.x);
        max.y = std::max(max.y, aabb.max.y);
    }

    bool contain(const Vector2 &point) const {
        return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y;
    }

    bool contain(const AABB2 &bound) const {
        return
            bound.min.x >= min.x && bound.max.x <= max.x &&
            bound.min.y >= min.y && bound.max.y <= max.y;
    }

    bool isEmpty() const {
        return min.x > max.x || min.y > max.y;
    }

    Vector2 center() const {
        return 0.5f * (min + max);
    }

    Vector2 extents() const {
        if (isEmpty()) {
            return Vector2(0.0f);
        }
        return max - min;
    }

    float offset(const Vector2 &point, uint32_t dim) const {
        float ext = extents()[dim];
        if (ext == 0.0) {
            return 0.0;
        }
        return (point[dim] - min[dim]) / ext;
    }

    uint32_t largestAxis() const {
        Vector2 exts = extents();
        if (exts.x >= exts.y) return 0;
        else return 1;
    }

    float area() const {
        Vector2 exts = extents();
        return exts.x * exts.y;
    }

    const Vector2 &operator[](uint32_t index) const {
        ASSERT(index <= 1);
        return (reinterpret_cast<const Vector2 *>(this))[index];
    }

    Vector2 &operator[](uint32_t index) {
        ASSERT(index <= 1);
        return (reinterpret_cast<Vector2 *>(this))[index];
    }

    Vector2 min = Vector2(Constants::inf);
    Vector2 max = Vector2(-Constants::inf);
};

constexpr bool intersectBool(const AABB2 &b0, const AABB2 &b1) {
    return
        !(b0.min.x > b1.max.x || b1.min.x > b0.max.x ||
            b0.min.y > b1.max.y || b1.min.y > b0.max.y);
}

inline AABB2 intersect(const AABB2 &b0, const AABB2 &b1) {
    AABB2 ret;
    ret.min = max(b0.min, b1.min);
    ret.max = min(b0.max, b1.max);
    if (ret.min.x > ret.max.x ||
        ret.min.y > ret.max.y) {
        return AABB2();
    }
    return ret;
}

inline AABB2 join(const AABB2 &b0, const AABB2 &b1) {
    return AABB2(min(b0.min, b1.min), max(b0.max, b1.max));
}

struct VECTORIZER_API AABB3
{
    AABB3() = default;
    AABB3(const Vector3 &min, const Vector3 &max) : min(min), max(max) {}

    void expand(const Vector3& point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);

        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }

    void expand(const AABB3& aabb) {
        min.x = std::min(min.x, aabb.min.x);
        min.y = std::min(min.y, aabb.min.y);
        min.z = std::min(min.z, aabb.min.z);

        max.x = std::max(max.x, aabb.max.x);
        max.y = std::max(max.y, aabb.max.y);
        max.z = std::max(max.z, aabb.max.z);
    }

    bool isEmpty() const {
        return min.x > max.x || min.y > max.y || min.z > max.z;
    }

    Vector3 center() const {
        return 0.5f * (min + max);
    }

    Vector3 extents() const {
        if (isEmpty()) {
            return Vector3(0.0f);
        }
        return max - min;
    }

    float offset(const Vector3 &point, uint32_t dim) const {
        float ext = extents()[dim];
        if (ext == 0.0) {
            return 0.0;
        }
        return (point[dim] - min[dim]) / ext;
    }

    uint32_t largestAxis() const {
        Vector3 exts = extents();
        if (exts.x >= exts.y && exts.x >= exts.z) return 0;
        else if (exts.y >= exts.x && exts.y >= exts.z) return 1;
        else return 2;
    }

    float surfaceArea() const {
        Vector3 exts = extents();
        return 2.0f * (exts.x * exts.y + exts.y * exts.z + exts.x * exts.z);
    }

    float volume() const {
        Vector3 exts = extents();
        return exts.x * exts.y * exts.z;
    }

    const Vector3 &operator[](uint32_t index) const {
        ASSERT(index <= 1);
        return (reinterpret_cast<const Vector3 *>(this))[index];
    }

    Vector3 &operator[](uint32_t index) {
        ASSERT(index <= 1);
        return (reinterpret_cast<Vector3 *>(this))[index];
    }

    Vector3 min = Vector3(Constants::inf);
    Vector3 max = Vector3(-Constants::inf);
};

inline bool intersectBool(const AABB3 &b0, const AABB3 &b1) {
    return !(
        b0.min.x > b1.max.x || b1.min.x > b0.max.x ||
        b0.min.y > b1.max.y || b1.min.y > b0.max.y ||
        b0.min.x > b1.max.z || b1.min.z > b0.max.z);
}

inline AABB3 intersect(const AABB3 &b0, const AABB3 &b1) {
    AABB3 ret;
    ret.min = max(b0.min, b1.min);
    ret.max = min(b0.max, b1.max);
    if (ret.min.x > ret.max.x ||
        ret.min.y > ret.max.y ||
        ret.min.z > ret.max.z) {
        return AABB3();
    }
    return ret;
}

inline AABB3 join(const AABB3 &b0, const AABB3 &b1) {
    return AABB3(min(b0.min, b1.min), max(b0.max, b1.max));
}

//////////////////////////////////////////////////////////////////////////

struct VECTORIZER_API Vector2I64
{
    int64_t x, y;

    constexpr Vector2I64() = default;
    explicit constexpr Vector2I64(int64_t val) : x(val), y(val) {}
    explicit constexpr Vector2I64(int64_t x, int64_t y) : x(x), y(y) {}

    int64_t operator[](uint32_t dim) const {
        ASSERT(dim <= 1);
        return *(reinterpret_cast<const int64_t *>(this) + dim);
    }

    int64_t &operator[](uint32_t dim) {
        ASSERT(dim <= 1);
        return *(reinterpret_cast<int64_t *>(this) + dim);
    }

    bool operator==(const Vector2I64 &other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vector2I64 &other) const {
        return !(*this == other);
    }

    Vector2I64 &operator+=(const Vector2I64 &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2I64 operator-() const {
        return Vector2I64(-x, -y);
    }

    Vector2I64 &operator-=(int64_t s) {
        x -= s;
        y -= s;
        return *this;
    }

    Vector2I64 &operator-=(const Vector2I64 &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2I64 &operator*=(const int64_t &s) {
        x *= s;
        y *= s;
        return *this;
    }

    Vector2I64 &operator*=(const Vector2I64 &other) {
        x *= other.x;
        y *= other.y;
        return *this;
    }
};

inline Vector2I64 operator+(Vector2I64 v0, const Vector2I64 &v1) {
    v0 += v1;
    return v0;
}

inline Vector2I64 operator-(Vector2I64 v, int64_t s) {
    v -= s;
    return v;
}

inline Vector2I64 operator-(Vector2I64 v0, const Vector2I64 &v1) {
    v0 -= v1;
    return v0;
}

inline Vector2I64 operator*(Vector2I64 v, int64_t s) {
    v *= s;
    return v;
}

inline Vector2I64 operator*(int64_t s, const Vector2I64 &v) {
    return v * s;
}

inline Vector2I64 operator*(Vector2I64 v0, const Vector2I64 &v1) {
    v0 *= v1;
    return v0;
}

inline int64_t dot(const Vector2I64 &v0, const Vector2I64 &v1) {
    return v0.x * v1.x + v0.y * v1.y;
}

inline int64_t cross(const Vector2I64 &v0, const Vector2I64 &v1) {
    return v0.x * v1.y - v0.y * v1.x;
}

inline int64_t lerpRound(int64_t a, int64_t b, double t) {
    return (int64_t)(a + (double)(b - a) * t);
}

inline Vector2I64 lerpRound(const Vector2I64 &a, const Vector2I64 &b, double t) {
    double x = a.x * (1.0 - t) + b.x * t;
    double y = a.y * (1.0 - t) + b.y * t;
    return Vector2I64((int64_t)x, (int64_t)y);
}

struct Vector2I
{
    int x, y;

    constexpr Vector2I() = default;
    explicit constexpr Vector2I(int val) : x(val), y(val) {}
    constexpr Vector2I(int x, int y) : x(x), y(y) {}

    int operator[](int dim) const {
        ASSERT(dim <= 1);
        return *(reinterpret_cast<const int *>(this) + dim);
    }

    int &operator[](int dim) {
        ASSERT(dim <= 1);
        return *(reinterpret_cast<int *>(this) + dim);
    }

    bool operator==(const Vector2I &other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vector2I &other) const {
        return !(*this == other);
    }
};

using Tri2 = std::array<Vector2, 3>;
using Tri3 = std::array<Vector3, 3>;

// This version assumes valid input (point in tri, tri well defined), and uses
// std::abs() and saturate() to always produce a valid coord.
inline Vector3 barycentricCoord(const Vector2 &point, const Tri2 &tri) {
    Vector3 coord;

    // {0, 1, 2} -> {a, b, c}
    Vector2 eAb = tri[1] - tri[0];
    Vector2 eAc = tri[2] - tri[0];
    float invAreaAbc = 1.0f / std::abs(cross(eAb, eAc));

    Vector2 d = point - tri[0];
    float areaAbp = std::abs(cross(d, eAb));
    coord.z = areaAbp * invAreaAbc;

    float areaApc = std::abs(cross(d, eAc));
    coord.y = areaApc * invAreaAbc;

    coord.x = saturate(1.0f - coord.y - coord.z);

    return coord;
}

// This version accepts arbitrary input and can generate out-of-bound coord.
inline Vector3 barycentricCoordNoClamp(const Vector2 &point, const Tri2 &tri) {
    Vector3 coord;

    // {0, 1, 2} -> {a, b, c}
    Vector2 eAb = tri[1] - tri[0];
    Vector2 eAc = tri[2] - tri[0];
    float invAreaAbc = 1.0f / cross(eAb, eAc);

    Vector2 d = point - tri[0];
    float areaAbp = cross(eAb, d);
    coord.z = areaAbp * invAreaAbc;

    float areaApc = cross(d, eAc);
    coord.y = areaApc * invAreaAbc;

    coord.x = 1.0f - coord.y - coord.z;

    return coord;
}

inline Vector3 barycentricCoord(const Vector2I64 &point, const Vector2I64 tri[3]) {
    Vector3 coord;

    // {0, 1, 2} -> {a, b, c}
    Vector2I64 eAB = tri[1] - tri[0];
    Vector2I64 eAC = tri[2] - tri[0];
    // float invAreaAbc = 1.0f / std::abs(cross(eAb, eAc));

    int64_t areaABC = cross(eAB, eAC);

    Vector2I64 d = point - tri[0];
    int64_t areaAbp = cross(eAB, d);
    coord.z = (float)((double)areaAbp / (double)areaABC);

    int64_t areaAPC = cross(d, eAC);
    coord.y = (float)((double)areaAPC / (double)areaABC);

    coord.x = (float)((double)(areaABC - areaAbp - areaAPC) / (double)areaABC);

    coord.x = saturate(coord.x);
    coord.y = saturate(coord.y);
    coord.z = saturate(coord.z);

    return coord;
}

inline Vector3 barycentricCoordSafe(const Vector2 &point, const Tri2 &tri) {
    Vector3 coord;

    // {0, 1, 2} -> {a, b, c}
    Vector2 eAB = tri[1] - tri[0];
    Vector2 eAC = tri[2] - tri[0];
    // float invAreaAbc = 1.0f / std::abs(cross(eAb, eAc));

    float areaABC = cross(eAB, eAC);
    Vector2 d = point - tri[0];
    float areaABP = cross(eAB, d);
    float areaCAP = cross(d, eAC);
    float areaBCP = areaABC - areaABP - areaCAP;
    if (areaABP < 0.0f && areaCAP < 0.0f) {
        return Vector3(1.0f, 0.0f, 0.0f);
    }
    if (areaABP < 0.0f && areaBCP < 0.0f) {
        return Vector3(0.0f, 1.0f, 0.0f);
    }
    if (areaBCP < 0.0f && areaCAP < 0.0f) {
        return Vector3(0.0f, 0.0f, 1.0f);
    }

    if (areaABP < 0.0f) {
        areaABP = 0.0f;
        areaABC = areaBCP + areaCAP;
    }
    if (areaBCP < 0.0f) {
        areaBCP = 0.0f;
        areaABC = areaABP + areaCAP;
    }
    if (areaCAP < 0.0f) {
        areaCAP = 0.0f;
        areaABC = areaABP + areaBCP;
    }

    coord.x = (float)((double)areaBCP / (double)areaABC);
    coord.y = (float)((double)areaCAP / (double)areaABC);
    coord.z = (float)((double)areaABP / (double)areaABC);

    ASSERT(coord.x >= 0.0f && coord.x <= 1.0f);
    ASSERT(coord.y >= 0.0f && coord.y <= 1.0f);
    ASSERT(coord.z >= 0.0f && coord.z <= 1.0f);
    // ASSERT(coord.x + coord.y + coord.z >= 0.0f && coord.x + coord.y + coord.z <= 1.0f);

    return coord;
}

// https://en.wikipedia.org/wiki/Kahan_summation_algorithm.
template <typename TFloat>
struct NeumaierSum
{
    TFloat sum = TFloat(0.0);
    TFloat correct = TFloat(0.0);

    inline void reset() {
        sum = TFloat(0.0);
        correct = TFloat(0.0);
    }

    inline void add(TFloat item) {
        TFloat t = sum + item;
        if (std::abs(sum) > std::abs(item)) {
            correct += (sum - t) + item;
        } else {
            correct += (item - t) + sum;
        }
        sum = t;
    }

    inline TFloat result() const {
        return sum + correct;
    }

    inline NeumaierSum &operator+=(TFloat item) {
        add(item);
        return *this;
    }

    inline explicit operator TFloat() const {
        return result();
    }
};

}