#pragma once

#include "assertion.h"
#include "maths.h"
#include "cast.h"
#include <array>

namespace vtrz
{

#pragma warning( push )
#pragma warning( disable: 4251 )

constexpr uint32_t kMaxGradDim = 1;
using Gradient = std::array<float, kMaxGradDim>;

template <typename T>
struct dreal
{
    T value;
    Gradient grad;

    inline dreal(T value = T(0)) : value(value) {
        zeroGrad();
    }

    inline dreal(T value, const Gradient &grad) : value(value), grad(grad) { }

    inline dreal(T value, uint32_t unitIndex) : value(value) {
        zeroGrad();
        ASSERT(unitIndex < kMaxGradDim);
        grad[unitIndex] = 1.0f;
    }

    inline dreal(const dreal &other) : value(other.value), grad(other.grad) {}

    inline void zeroGrad() {
        for (uint32_t i = 0; i < kMaxGradDim; ++i) {
            grad[i] = 0.0f;
        }
    }

    template <typename U>
    explicit operator dreal<U>() const {
        return dreal<U>((U)value, grad);
    }

    inline dreal &operator+=(const dreal &other) {
        value += other.value;
        for (uint32_t i = 0; i < kMaxGradDim; ++i) {
            grad[i] += other.grad[i];
        }
        return *this;
    }

    inline dreal &operator+=(T other) {
        value += other;
        return *this;
    }

    inline dreal &operator-=(const dreal &other) {
        value -= other.value;
        for (uint32_t i = 0; i < kMaxGradDim; ++i) {
            grad[i] -= other.grad[i];
        }
        return *this;
    }

    inline dreal &operator-=(T other) {
        value -= other;
        return *this;
    }

    inline dreal &operator*=(const dreal &other) {
        for (uint32_t i = 0; i < kMaxGradDim; ++i) {
            grad[i] = other.grad[i] * (float)value + grad[i] * (float)other.value;
        }
        value *= other.value;
        return *this;
    }

    inline dreal &operator*=(T other) {
        value *= other;
        for (uint32_t i = 0; i < kMaxGradDim; ++i) {
            grad[i] *= (float)other;
        }
        return *this;
    }

    inline dreal &operator/=(const dreal &other) {
        float invDenom = 1.0f / (float)(other.value * other.value);
        for (uint32_t i = 0; i < kMaxGradDim; ++i) {
            grad[i] = (grad[i] * (float)other.value - (float)value * other.grad[i]) * invDenom;
        }
        value /= other.value;
        return *this;
    }

    inline dreal &operator/=(T other) {
        value /= other;
        float invOther = 1.0f / (float)other;
        for (uint32_t i = 0; i < kMaxGradDim; ++i) {
            grad[i] *= invOther;
        }
        return *this;
    }
};

template <typename T>
inline dreal<T> operator+(dreal<T> x, const dreal<T> &y) {
    x += y;
    return x;
}

template <typename T>
inline dreal<T> operator+(dreal<T> x, T y) {
    x += y;
    return x;
}

template <typename T>
inline dreal<T> operator+(T x, const dreal<T> &y) {
    return y + x;
}

template <typename T>
inline dreal<T> operator-(dreal<T> x, const dreal<T> &y) {
    x -= y;
    return x;
}

template <typename T>
inline dreal<T> operator-(dreal<T> x, T y) {
    x -= y;
    return x;
}

template <typename T>
inline dreal<T> operator-(T x, const dreal<T> &y) {
    return dreal<T>(x) - y;
}

template <typename T>
inline dreal<T> operator-(const dreal<T> &x) {
    dreal<T> neg;
    neg.value = -x.value;
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        neg.grad[i] = -x.grad[i];
    }
    return neg;
}

template <typename T>
inline dreal<T> operator*(dreal<T> x, const dreal<T> &y) {
    x *= y;
    return x;
}

template <typename T>
inline dreal<T> operator*(dreal<T> x, T y) {
    x *= y;
    return x;
}

template <typename T>
inline dreal<T> operator*(T x, const dreal<T> &y) {
    return y * x;
}

template <typename T>
inline dreal<T> operator/(dreal<T> x, const dreal<T> &y) {
    x /= y;
    return x;
}

template <typename T>
inline dreal<T> operator/(dreal<T> x, T y) {
    x /= y;
    return x;
}

template <typename T>
inline dreal<T> operator/(T x, const dreal<T> &y) {
    return dreal<T>(x) / y;
}

template <typename T>
inline dreal<T> abs(const dreal<T> &x) {
    if (x.value >= T(0)) return x;
    return -x;
}

template <typename T>
inline dreal<T> sin(const dreal<T> &x) {
    dreal<T> y;
    y.value = std::sin(x.value);
    float cosine = std::cos(x.value);
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        y.grad[i] = cosine * x.grad[i];
    }
    return y;
}

template <typename T>
inline dreal<T> cos(const dreal<T> &x) {
    dreal<T> y;
    y.value = std::cos(x.value);
    float sine = std::sin(x.value);
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        y.grad[i] = -sine * x.grad[i];
    }
    return y;
}

template <typename T>
inline dreal<T> asin(const dreal<T> &x) {
    dreal<T> y;
    y.value = std::asin(x.value);
    float invDenom = 1.0f / std::sqrt(1.0f - x.value * x.value);
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        y.grad[i] = x.grad[i] * invDenom;
    }
}

template <typename T>
inline dreal<T> asinSafe(const dreal<T> &x) {
    float val = clamp(x.value, -1.0f, 1.0f);

    dreal<T> y;
    y.value = std::asin(val);
    float invDenom = 1.0f / std::sqrt(1.0f - val * val);
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        y.grad[i] = x.grad[i] * invDenom;
    }
    return y;
}

template <typename T>
inline dreal<T> acos(const dreal<T> &x) {
    dreal<T> y;
    y.value = std::acos(x.value);
    float invDenom = -1.0f / std::sqrt(1.0f - x.value * x.value);
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        y.grad[i] = x.grad[i] * invDenom;
    }
    return y;
}

template <typename T>
inline dreal<T> acosSafe(const dreal<T> &x) {
    float val = clamp(x.value, -1.0f, 1.0f);

    dreal<T> y;
    y.value = std::acos(val);
    float invDenom = -1.0f / std::sqrt(1.0f - val * val);
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        y.grad[i] = x.grad[i] * invDenom;
    }
    return y;
}

template <typename T>
inline dreal<T> sqrt(const dreal<T> &x) {
    dreal<T> y;
    T sqrtVal = std::sqrt(x.value);
    y.value = sqrtVal;
    // Need to avoid divide by 0 (sqrt is not differentiable at 0).
    sqrtVal = std::max(sqrtVal, 1e-6f);
    T coef = T(1) / (T(2) * sqrtVal);
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        y.grad[i] = coef * x.grad[i];
    }
    return y;
}

template <typename T>
inline dreal<T> sqrtSafe(const dreal<T> &x) {
    T sqrtVal = std::sqrt(std::max(x.value, 0.0f));
    T coef = T(1) / (T(2) * sqrtVal);
    dreal<T> y;
    y.value = sqrtVal;
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        y.grad[i] = coef * x.grad[i];
    }
    return y;
}

template <typename T>
inline bool isNan(const dreal<T> &x) {
    if (std::isnan(x.value)) return true;
    for (uint32_t i = 0; i < kMaxGradDim; ++i) {
        if (std::isnan(x.grad[i])) return true;
    }
    return false;
}

using dfloat = dreal<float>;
using ddouble = dreal<double>;

struct VECTORIZER_API DVector2
{
    dfloat x, y;
    inline DVector2() = default;
    explicit inline DVector2(const dfloat &val) : x(val), y(val) {}
    explicit inline DVector2(const Vector2 &v) : x(v.x), y(v.y) {}
    inline DVector2(const dfloat &x, const dfloat &y) : x(x), y(y) {}

    explicit operator Vector2() const {
        return Vector2(x.value, y.value);
    }

    const dfloat &operator[](uint32_t dim) const {
        ASSERT(dim <= 1);
        return *(reinterpret_cast<const dfloat *>(this) + dim);
    }

    dfloat &operator[](uint32_t dim) {
        ASSERT(dim <= 1);
        return *(reinterpret_cast<dfloat *>(this) + dim);
    }

    void setValue(const Vector2 &val) {
        x.value = val.x;
        y.value = val.y;
    }

    Vector2 value() const {
        return Vector2(x.value, y.value);
    }

    DVector2 &operator+=(const dfloat &s) {
        x += s;
        y += s;
        return *this;
    }

    DVector2 &operator+=(const DVector2 &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    DVector2 &operator-=(const dfloat &s) {
        x -= s;
        y -= s;
        return *this;
    }

    DVector2 &operator-=(const DVector2 &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    DVector2 &operator*=(const dfloat &s) {
        x *= s;
        y *= s;
        return *this;
    }

    DVector2 &operator*=(const DVector2 &other) {
        x *= other.x;
        y *= other.y;
        return *this;
    }

    DVector2 &operator/=(const dfloat &s) {
        x /= s;
        y /= s;
        return *this;
    }

    DVector2 &operator/=(const DVector2 &v) {
        x /= v.x;
        y /= v.y;
        return *this;
    }
};

inline DVector2 operator+(DVector2 v, const dfloat &s) {
    v += s;
    return v;
}

inline DVector2 operator+(const dfloat &s, DVector2 v) {
    v += s;
    return v;
}

inline DVector2 operator+(DVector2 v0, const DVector2 &v1) {
    v0 += v1;
    return v0;
}

inline DVector2 operator-(DVector2 v, const dfloat &s) {
    v -= s;
    return v;
}

inline DVector2 operator-(DVector2 v0, const DVector2 &v1) {
    v0 -= v1;
    return v0;
}

inline DVector2 operator-(const dfloat &s, const DVector2 &v) {
    return DVector2(s) - v;
}

inline DVector2 operator-(const DVector2 &v) {
    DVector2 neg;
    neg.x = -v.x;
    neg.y = -v.y;
    return neg;
}

inline DVector2 operator*(DVector2 v, const dfloat &s) {
    v *= s;
    return v;
}

inline DVector2 operator*(dfloat s, const DVector2 &v) {
    return v * s;
}

inline DVector2 operator*(DVector2 v0, const DVector2 &v1) {
    v0 *= v1;
    return v0;
}

inline DVector2 operator/(DVector2 v, const dfloat &s) {
    v /= s;
    return v;
}

inline DVector2 operator/(DVector2 v0, const DVector2 &v1) {
    v0 /= v1;
    return v0;
}

inline DVector2 operator/(const dfloat &s, const DVector2 &v) {
    return DVector2(s) / v;
}

inline dfloat dot(const DVector2 &v0, const DVector2 &v1) {
    return v0.x * v1.x + v0.y * v1.y;
}

inline dfloat cross(const DVector2 &v0, const DVector2 &v1) {
    return v0.x * v1.y - v0.y * v1.x;
}

inline bool isNan(const DVector2 &v) {
    return isNan(v.x) || isNan(v.y);
}

struct VECTORIZER_API DVector3
{
    dfloat x, y, z;
    inline DVector3() = default;
    explicit inline DVector3(const dfloat &val) : x(val), y(val), z(val) {}
    explicit inline DVector3(const Vector3 &v) : x(v.x), y(v.y), z(v.z) {}
    inline DVector3(const dfloat &x, const dfloat &y, const dfloat &z) : x(x), y(y), z(z) {}

    explicit operator Vector3() const {
        return Vector3(x.value, y.value, z.value);
    }

    const dfloat &operator[](uint32_t dim) const {
        ASSERT(dim <= 2);
        return *(reinterpret_cast<const dfloat *>(this) + dim);
    }

    dfloat &operator[](uint32_t dim) {
        ASSERT(dim <= 2);
        return *(reinterpret_cast<dfloat *>(this) + dim);
    }

    void setValue(const Vector3 &val) {
        x.value = val.x;
        y.value = val.y;
        z.value = val.z;
    }

    Vector3 value() const {
        return Vector3(x.value, y.value, z.value);
    }

    DVector3 &operator+=(const dfloat &s) {
        x += s;
        y += s;
        z += s;
        return *this;
    }

    DVector3 &operator+=(const DVector3 &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    DVector3 &operator-=(const dfloat &s) {
        x -= s;
        y -= s;
        z -= s;
        return *this;
    }

    DVector3 &operator-=(const DVector3 &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    DVector3 &operator*=(const dfloat &s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    DVector3 &operator*=(const DVector3 &other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        return *this;
    }

    DVector3 &operator/=(const dfloat &s) {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    DVector3 &operator/=(const DVector3 &v) {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        return *this;
    }

    explicit operator DVector2() const {
        return DVector2(x, y);
    }
};

inline DVector3 operator+(DVector3 v, const dfloat &s) {
    v += s;
    return v;
}

inline DVector3 operator+(const dfloat &s, DVector3 v) {
    v += s;
    return v;
}

inline DVector3 operator+(DVector3 v0, const DVector3 &v1) {
    v0 += v1;
    return v0;
}

inline DVector3 operator-(DVector3 v, const dfloat &s) {
    v -= s;
    return v;
}

inline DVector3 operator-(DVector3 v0, const DVector3 &v1) {
    v0 -= v1;
    return v0;
}

inline DVector3 operator-(const dfloat &s, const DVector3 &v) {
    return DVector3(s) - v;
}

inline DVector3 operator-(const DVector3 &v) {
    DVector3 neg;
    neg.x = -v.x;
    neg.y = -v.y;
    neg.z = -v.z;
    return neg;
}

inline DVector3 operator*(DVector3 v, const dfloat &s) {
    v *= s;
    return v;
}

inline DVector3 operator*(dfloat s, const DVector3 &v) {
    return v * s;
}

inline DVector3 operator*(DVector3 v0, const DVector3 &v1) {
    v0 *= v1;
    return v0;
}

inline DVector3 operator/(DVector3 v, const dfloat &s) {
    v /= s;
    return v;
}

inline DVector3 operator/(DVector3 v0, const DVector3 &v1) {
    v0 /= v1;
    return v0;
}

inline DVector3 operator/(const dfloat &s, const DVector3 &v) {
    return DVector3(s) / v;
}

inline dfloat dot(const DVector3 &v0, const DVector3 &v1) {
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

inline DVector3 cross(const DVector3 &v0, const DVector3 &v1) {
    return DVector3(
        v0.y * v1.z - v0.z * v1.y,
        v0.z * v1.x - v0.x * v1.z,
        v0.x * v1.y - v0.y * v1.x
    );
}

inline dfloat length(const DVector3 &v) {
    return sqrt(dot(v, v));
}

inline dfloat length2(const DVector3 &v) {
    return dot(v, v);
}

inline DVector3 normalize(const DVector3 &v) {
    dfloat len = length(v);
    ASSERT(len.value > 0.0f);
    return v * (1.0f / len);
}

inline dfloat unitAngle(const DVector3 &u, const DVector3 &v) {
    if (dot(u, v).value < 0.0f)
        return Constants::pi - 2.0f * asinSafe(0.5f * length(v + u));
    else
        return 2.0f * asinSafe(0.5f * length(v - u));
}

inline bool isNan(const DVector3 &v) {
    return isNan(v.x) || isNan(v.y) || isNan(v.z);
}

struct VECTORIZER_API DVector4
{
    dfloat x, y, z, w;
    inline DVector4() = default;
    explicit inline DVector4(const dfloat &val) : x(val), y(val), z(val), w(val) {}
    explicit inline DVector4(const Vector4 &v) : x(v.x), y(v.y), z(v.z), w(v.w) {}
    inline DVector4(const dfloat &x, const dfloat &y, const dfloat &z, const dfloat &w) : x(x), y(y), z(z), w(w) {}
    inline DVector4(const DVector3 &v3, const dfloat &w) : x(v3.x), y(v3.y), z(v3.z), w(w) {}

    explicit operator Vector4() const {
        return Vector4(x.value, y.value, z.value, w.value);
    }

    explicit operator DVector3() const {
        return DVector3(x, y, z);
    }

    const dfloat &operator[](uint32_t dim) const {
        ASSERT(dim <= 3);
        return *(reinterpret_cast<const dfloat *>(this) + dim);
    }

    dfloat &operator[](uint32_t dim) {
        ASSERT(dim <= 3);
        return *(reinterpret_cast<dfloat *>(this) + dim);
    }

    void setValue(const Vector4 &val) {
        x.value = val.x;
        y.value = val.y;
        z.value = val.z;
        w.value = val.w;
    }

    Vector4 value() const {
        return Vector4(x.value, y.value, z.value, w.value);
    }

    DVector4 &operator+=(const dfloat &s) {
        x += s;
        y += s;
        z += s;
        w += s;
        return *this;
    }

    DVector4 &operator+=(const DVector4 &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }

    DVector4 &operator-=(const dfloat &s) {
        x -= s;
        y -= s;
        z -= s;
        w -= s;
        return *this;
    }

    DVector4 &operator-=(const DVector4 &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        w -= other.w;
        return *this;
    }

    DVector4 &operator*=(const dfloat &s) {
        x *= s;
        y *= s;
        z *= s;
        w *= s;
        return *this;
    }

    DVector4 &operator*=(const DVector4 &other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        w *= other.w;
        return *this;
    }

    DVector4 &operator/=(const dfloat &s) {
        x /= s;
        y /= s;
        z /= s;
        w /= s;
        return *this;
    }

    DVector4 &operator/=(const DVector4 &v) {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        w /= v.w;
        return *this;
    }
};

inline DVector4 operator+(DVector4 v, const dfloat &s) {
    v += s;
    return v;
}

inline DVector4 operator+(const dfloat &s, DVector4 v) {
    v += s;
    return v;
}

inline DVector4 operator+(DVector4 v0, const DVector4 &v1) {
    v0 += v1;
    return v0;
}

inline DVector4 operator-(DVector4 v, const dfloat &s) {
    v -= s;
    return v;
}

inline DVector4 operator-(DVector4 v0, const DVector4 &v1) {
    v0 -= v1;
    return v0;
}

inline DVector4 operator-(const dfloat &s, const DVector4 &v) {
    return DVector4(s) - v;
}

inline DVector4 operator-(const DVector4 &v) {
    DVector4 neg;
    neg.x = -v.x;
    neg.y = -v.y;
    neg.z = -v.z;
    neg.w = -v.w;
    return neg;
}

inline DVector4 operator*(DVector4 v, const dfloat &s) {
    v *= s;
    return v;
}

inline DVector4 operator*(dfloat s, const DVector4 &v) {
    return v * s;
}

inline DVector4 operator*(DVector4 v0, const DVector4 &v1) {
    v0 *= v1;
    return v0;
}

inline DVector4 operator*(const Matrix4x4 &m, const DVector4 &v) {
    DVector4 ret;
    for (uint32_t i = 0; i < 4; ++i)
        for (uint32_t j = 0; j < 4; ++j)
            ret[i] += m.arr[i][j] * v[j];
    return ret;
}

inline DVector4 operator/(DVector4 v, const dfloat &s) {
    v /= s;
    return v;
}

inline DVector4 operator/(DVector4 v0, const DVector4 &v1) {
    v0 /= v1;
    return v0;
}

inline DVector4 operator/(const dfloat &s, const DVector4 &v) {
    return DVector4(s) / v;
}

inline dfloat dot(const DVector4 &v0, const DVector4 &v1) {
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z + v0.w * v1.w;
}

struct VECTORIZER_API DMatrix3x3
{
    union
    {
        DVector3 rows[3];
        dfloat arr[3][3];
        struct {
            dfloat
                m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22;
        } m;
    };

    explicit DMatrix3x3(float diag = 1.0f) {
        for (uint32_t i = 0; i < 3; ++i)
            for (uint32_t j = 0; j < 3; ++j)
                arr[i][j] = i == j ? diag : 0.0f;
    }

    DMatrix3x3(const DMatrix3x3 &other) {
        memcpy(this, &other, sizeof(DMatrix3x3));
    }

    DMatrix3x3(const DVector3 &row0, const DVector3 &row1, const DVector3 &row2) {
        rows[0] = row0; rows[1] = row1; rows[2] = row2;
    }

    explicit DMatrix3x3(const DVector3 *rs) {
        memcpy(rows, rs, sizeof(DVector3) * 3);
    }

    explicit DMatrix3x3(const dfloat *a) {
        memcpy(arr, a, sizeof(dfloat) * 9);
    }

    DMatrix3x3(
        const dfloat &m00, const dfloat &m01, const dfloat &m02,
        const dfloat &m10, const dfloat &m11, const dfloat &m12,
        const dfloat &m20, const dfloat &m21, const dfloat &m22) {
        m.m00 = m00; m.m01 = m01; m.m02 = m02;
        m.m10 = m10; m.m11 = m11; m.m12 = m12;
        m.m20 = m20; m.m21 = m21; m.m22 = m22;
    }

    DMatrix3x3 &operator+=(const DMatrix3x3 &other) {
        for (uint32_t i = 0; i < 3; ++i)
            for (uint32_t j = 0; j < 3; ++j)
                arr[i][j] += other.arr[i][j];

        return *this;
    }
};

inline DVector3 operator*(const DMatrix3x3 &m, const DVector3 &v) {
    DVector3 ret;
    for (uint32_t i = 0; i < 3; ++i) {
        ret[i] = dot(m.rows[i], v);
    }
    return ret;
}

template <typename T2, typename = std::enable_if_t<
    std::is_same<T2, DMatrix3x3>::value ||
    std::is_same<T2, Matrix3x3>::value>>
inline DMatrix3x3 operator*(const DMatrix3x3 &m1, const T2 &m2) {
    DMatrix3x3 ret;
    for (uint32_t i = 0; i < 3; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            ret.arr[i][j] = 0;
            for (uint32_t k = 0; k < 3; ++k)
                ret.arr[i][j] += m1.arr[i][k] * m2.arr[k][j];
        }
    }
    return ret;
}

inline DMatrix3x3 operator*(const Matrix3x3 &m1, const DMatrix3x3 &m2) {
    DMatrix3x3 ret;
    for (uint32_t i = 0; i < 3; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            ret.arr[i][j] = 0;
            for (uint32_t k = 0; k < 3; ++k)
                ret.arr[i][j] += m1.arr[i][k] * m2.arr[k][j];
        }
    }
    return ret;
}

struct VECTORIZER_API DMatrix4x4
{
    union
    {
        DVector4 rows[4];
        dfloat arr[4][4];
        struct {
            dfloat
                m00, m01, m02, m03,
                m10, m11, m12, m13,
                m20, m21, m22, m23,
                m30, m31, m32, m33;
        } m;
    };

    explicit DMatrix4x4(float diag = 1.0f) {
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                arr[i][j] = i == j ? diag : 0.0f;
    }

    DMatrix4x4(const DMatrix4x4 &other) {
        memcpy(this, &other, sizeof(DMatrix4x4));
    }

    DMatrix4x4(const DVector4 &row0, const DVector4 &row1, const DVector4 &row2, const DVector4 &row3) {
        rows[0] = row0; rows[1] = row1; rows[2] = row2; rows[3] = row3;
    }

    explicit DMatrix4x4(const DVector4 *rs) {
        memcpy(rows, rs, sizeof(DVector4) * 4);
    }

    explicit DMatrix4x4(const dfloat *a) {
        memcpy(arr, a, sizeof(dfloat) * 16);
    }

    DMatrix4x4(
        const dfloat &m00, const dfloat &m01, const dfloat &m02, const dfloat &m03,
        const dfloat &m10, const dfloat &m11, const dfloat &m12, const dfloat &m13,
        const dfloat &m20, const dfloat &m21, const dfloat &m22, const dfloat &m23,
        const dfloat &m30, const dfloat &m31, const dfloat &m32, const dfloat &m33) {
        m.m00 = m00; m.m01 = m01; m.m02 = m02; m.m03 = m03;
        m.m10 = m10; m.m11 = m11; m.m12 = m12; m.m13 = m13;
        m.m20 = m20; m.m21 = m21; m.m22 = m22; m.m23 = m23;
        m.m30 = m30; m.m31 = m31; m.m32 = m32; m.m33 = m33;
    }

    explicit DMatrix4x4(const Matrix4x4 &matrix) {
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                arr[i][j] = matrix.arr[i][j];
    }

    DVector3 translation() const {
        return DVector3(m.m03, m.m13, m.m23);
    }

    DMatrix4x4 transpose() const {
        DMatrix4x4 t;
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                t.arr[i][j] = arr[j][i];
        return t;
    }

    DMatrix4x4 inverse() const {
        // Closed-form solution: https://github.com/willnode/N-Matrix-Programmer.git
        // Assume the matrix is invertible.

        dfloat A2323 = m.m22 * m.m33 - m.m23 * m.m32;
        dfloat A1323 = m.m21 * m.m33 - m.m23 * m.m31;
        dfloat A1223 = m.m21 * m.m32 - m.m22 * m.m31;
        dfloat A0323 = m.m20 * m.m33 - m.m23 * m.m30;
        dfloat A0223 = m.m20 * m.m32 - m.m22 * m.m30;
        dfloat A0123 = m.m20 * m.m31 - m.m21 * m.m30;
        dfloat A2313 = m.m12 * m.m33 - m.m13 * m.m32;
        dfloat A1313 = m.m11 * m.m33 - m.m13 * m.m31;
        dfloat A1213 = m.m11 * m.m32 - m.m12 * m.m31;
        dfloat A2312 = m.m12 * m.m23 - m.m13 * m.m22;
        dfloat A1312 = m.m11 * m.m23 - m.m13 * m.m21;
        dfloat A1212 = m.m11 * m.m22 - m.m12 * m.m21;
        dfloat A0313 = m.m10 * m.m33 - m.m13 * m.m30;
        dfloat A0213 = m.m10 * m.m32 - m.m12 * m.m30;
        dfloat A0312 = m.m10 * m.m23 - m.m13 * m.m20;
        dfloat A0212 = m.m10 * m.m22 - m.m12 * m.m20;
        dfloat A0113 = m.m10 * m.m31 - m.m11 * m.m30;
        dfloat A0112 = m.m10 * m.m21 - m.m11 * m.m20;

        dfloat invdet =
            m.m00 * (m.m11 * A2323 - m.m12 * A1323 + m.m13 * A1223)
            - m.m01 * (m.m10 * A2323 - m.m12 * A0323 + m.m13 * A0223)
            + m.m02 * (m.m10 * A1323 - m.m11 * A0323 + m.m13 * A0123)
            - m.m03 * (m.m10 * A1223 - m.m11 * A0223 + m.m12 * A0123);
        invdet = dfloat(1) / invdet;

        DMatrix4x4 inv;
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

    DMatrix4x4 &operator+=(const DMatrix4x4 &other) {
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                arr[i][j] += other.arr[i][j];

        return *this;
    }

    Matrix4x4 value() const {
        Matrix4x4 val;
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                val.arr[i][j] = arr[i][j].value;
        return val;
    }
};

inline DVector4 operator*(const DMatrix4x4 &m, const DVector4 &v) {
    DVector4 ret;
    for (uint32_t i = 0; i < 4; ++i) {
        ret[i] = dot(m.rows[i], v);
    }
    return ret;
}

template <typename T2, typename = std::enable_if_t<
    std::is_same<T2, DMatrix4x4>::value ||
    std::is_same<T2, Matrix4x4>::value>>
inline DMatrix4x4 operator*(const DMatrix4x4 &m1, const T2 &m2) {
    DMatrix4x4 ret;
    for (uint32_t i = 0; i < 4; ++i) {
        for (uint32_t j = 0; j < 4; ++j) {
            ret.arr[i][j] = 0;
            for (uint32_t k = 0; k < 4; ++k)
                ret.arr[i][j] += m1.arr[i][k] * m2.arr[k][j];
        }
    }
    return ret;
}

inline DMatrix4x4 operator*(const Matrix4x4 &m1, const DMatrix4x4 &m2) {
    DMatrix4x4 ret;
    for (uint32_t i = 0; i < 4; ++i) {
        for (uint32_t j = 0; j < 4; ++j) {
            ret.arr[i][j] = 0;
            for (uint32_t k = 0; k < 4; ++k)
                ret.arr[i][j] += m1.arr[i][k] * m2.arr[k][j];
        }
    }
    return ret;
}

// Only position carries gradient in this function.
inline DMatrix4x4 makeLookupView(const DVector3 &position, Vector3 view, Vector3 up) {
    view = normalize(view);
    Vector3 right = cross(up, -view);
    if (length2(right) < 1e-8f) {
        orthonormalBasis(-view, right, up);
    } else {
        right = normalize(right);
        up = cross(-view, right);
    }
    DVector3 t(dot(DVector3(right), position), dot(DVector3(up), position), dot(DVector3 (-view), position));
    return DMatrix4x4(
        right.x, right.y, right.z, -t.x,
        up.x, up.y, up.z, -t.y,
        -view.x, -view.y, -view.z, -t.z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

inline DMatrix4x4 makeTranslate(const dfloat &tx, const dfloat &ty, const dfloat &tz) {
    DMatrix4x4 translation(1.0f);
    translation.arr[0][3] = tx;
    translation.arr[1][3] = ty;
    translation.arr[2][3] = tz;
    return translation;
}

inline DMatrix4x4 makeRotate(const dfloat &angle, DVector3 axis) {
    DMatrix4x4 rotation(1.0f);
    axis = normalize(axis);
    dfloat sine = sin(angle);
    dfloat cosine = cos(angle);

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

inline DMatrix4x4 makeScale(const dfloat &sx, const dfloat &sy, const dfloat &sz) {
    DMatrix4x4 scale(1.0f);
    scale.arr[0][0] = sx;
    scale.arr[1][1] = sy;
    scale.arr[2][2] = sz;
    return scale;
}

struct VECTORIZER_API DVector2I64
{
    int64_t x, y;
    DVector2 d;

    DVector2I64() = default;

    Vector2I64 value() const {
        return Vector2I64(x, y);
    }

    DVector2I64 &operator+=(const DVector2I64 &other) {
        x += other.x;
        y += other.y;
        d += other.d;
        return *this;
    }

    DVector2I64 &operator-=(const DVector2I64 &other) {
        x -= other.x;
        y -= other.y;
        d -= other.d;
        return *this;
    }

    DVector2I64 &operator*=(const DVector2I64 &other) {
        x *= other.x;
        y *= other.y;
        d *= other.d;
        return *this;
    }
};

inline DVector2I64 operator+(DVector2I64 v0, const DVector2I64 &v1) {
    v0 += v1;
    return v0;
}

inline DVector2I64 operator-(DVector2I64 v0, const DVector2I64 &v1) {
    v0 -= v1;
    return v0;
}

inline DVector2I64 operator*(DVector2I64 v0, const DVector2I64 &v1) {
    v0 *= v1;
    return v0;
}

inline DVector2I64 lerpRound(const DVector2I64 &a, const DVector2I64 &b, const ddouble &t) {
    DVector2I64 ret;
    double x = a.x * (1.0 - t.value) + b.x * t.value;
    double y = a.y * (1.0 - t.value) + b.y * t.value;
    ret.x = (int64_t)x;
    ret.y = (int64_t)y;

    ret.d = lerp(a.d, b.d, (dfloat)t);
    return ret;
}

// TODO: move?
inline DVector2 cast(const DVector2I64 &v) {
    DVector2 ret;
    ret.x = (float)vtrz::cast(v.x);
    ret.y = (float)vtrz::cast(v.y);
    ret.x.grad = v.d.x.grad;
    ret.y.grad = v.d.y.grad;
    return ret;
}

inline DVector2I64 cast(const DVector2 &v) {
    DVector2I64 ret;
    ret.x = vtrz::cast(v.x.value);
    ret.y = vtrz::cast(v.y.value);
    ret.d = v;
    return ret;
}

inline float getVal(float x) { return x; }
inline float getVal(const dfloat &x) { return x.value; }
inline void setVal(float x, float v) { x = v; }
inline void setVal(dfloat &x, float v) { x.value = v; }

inline Vector2 getVal(const Vector2 &p) { return p; }
inline Vector2 getVal(const DVector2 &p) { return p.value(); }

inline Vector2I64 getVal(const Vector2I64 &p) { return p; }
inline Vector2I64 getVal(const DVector2I64 &p) { return p.value(); }

using Tri2Grad = std::array<DVector2, 3>;
using Tri3Grad = std::array<DVector3, 3>;

inline Tri2 getVal(const Tri2 &tri) { return tri; }
inline Tri2 getVal(const Tri2Grad &triGrad) { return { triGrad[0].value(), triGrad[1].value(), triGrad[2].value() }; }
inline Tri3 getVal(const Tri3 &tri) { return tri; }
inline Tri3 getVal(const Tri3Grad &triGrad) { return { triGrad[0].value(), triGrad[1].value(), triGrad[2].value() }; }

// This version accepts arbitrary input and can generate out-of-bound coord.
inline DVector3 barycentricCoordNoClamp(const DVector2 &point, const Tri2Grad &tri) {
    DVector3 coord;

    // {0, 1, 2} -> {a, b, c}
    DVector2 eAB = tri[1] - tri[0];
    DVector2 eAC = tri[2] - tri[0];
    dfloat areaABC = cross(eAB, eAC);

    DVector2 d = point - tri[0];
    dfloat areaABP = cross(eAB, d);
    coord.z = areaABP / areaABC;

    dfloat areaAPC = cross(d, eAC);
    coord.y = areaAPC / areaABC;

    coord.x = 1.0f - coord.y - coord.z;

    return coord;
}

inline DVector3 barycentricCoordSafe(const DVector2 &point, const Tri2Grad &tri) {
    DVector3 coord;

    // {0, 1, 2} -> {a, b, c}
    DVector2 eAB = tri[1] - tri[0];
    DVector2 eAC = tri[2] - tri[0];
    // float invAreaAbc = 1.0f / std::abs(cross(eAb, eAc));

    dfloat areaABC = cross(eAB, eAC);
    DVector2 d = point - tri[0];
    dfloat areaABP = cross(eAB, d);
    dfloat areaCAP = cross(d, eAC);
    dfloat areaBCP = areaABC - areaABP - areaCAP;
    if (areaABP.value < 0.0f && areaCAP.value < 0.0f) {
        areaABP.value = areaCAP.value = 0.0f;
        areaABC.value = areaBCP.value;
    } else if (areaABP.value < 0.0f && areaBCP.value < 0.0f) {
        areaABP.value = areaBCP.value = 0.0f;
        areaABC.value = areaCAP.value;
    } else if (areaBCP.value < 0.0f && areaCAP.value < 0.0f) {
        areaBCP.value = areaCAP.value = 0.0f;
        areaABC.value = areaABP.value;
    } else if (areaABP.value < 0.0f) {
        areaABP.value = 0.0f;
        areaABC.value = areaBCP.value + areaCAP.value;
    } else if (areaBCP.value < 0.0f) {
        areaBCP.value = 0.0f;
        areaABC.value = areaABP.value + areaCAP.value;
    } else if (areaCAP.value < 0.0f) {
        areaCAP.value = 0.0f;
        areaABC.value = areaABP.value + areaBCP.value;
    }

    coord.x = (dfloat)((ddouble)areaBCP / (ddouble)areaABC);
    coord.y = (dfloat)((ddouble)areaCAP / (ddouble)areaABC);
    coord.z = (dfloat)((ddouble)areaABP / (ddouble)areaABC);

    ASSERT(coord.x.value >= 0.0f && coord.x.value <= 1.0f);
    ASSERT(coord.y.value >= 0.0f && coord.y.value <= 1.0f);
    ASSERT(coord.z.value >= 0.0f && coord.z.value <= 1.0f);
    // ASSERT(coord.x + coord.y + coord.z >= 0.0f && coord.x + coord.y + coord.z <= 1.0f);

    return coord;
}

#pragma warning( pop )

}