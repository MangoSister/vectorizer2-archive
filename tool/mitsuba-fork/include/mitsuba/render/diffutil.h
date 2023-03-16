#pragma once
#if !defined(__MITSUBA_RENDER_DIFFUTIL_H_)
#define __MITSUBA_RENDER_DIFFUTIL_H_

#include <mitsuba/core/autodiff.h>
#include <mitsuba/core/spectrum.h>
#include <mitsuba/render/texture.h>
#include <type_traits>

MTS_NAMESPACE_BEGIN

// TODO: this file is basically a bunch of duplciate code...

template <typename Scalar, typename Gradient>
struct  StaticDFloat {
public:
    // ======================================================================
    /// @{ \name Constructors and accessors
    // ======================================================================

    /// Create a new constant automatic differentiation scalar
    inline StaticDFloat(Scalar value = (Scalar)0) : value(value), grad(0.0) { }

    /// Construct a new scalar with the specified value and one first derivative set to 1
    inline StaticDFloat(size_t index, const Scalar &value)
        : value(value), grad(0.0) {
        grad[index] = 1.0;
    }

    /// Construct a scalar associated with the given gradient
    inline StaticDFloat(Scalar value, const Gradient &grad)
        : value(value), grad(grad) { }

    /// Copy constructor
    inline StaticDFloat(const StaticDFloat &s)
        : value(s.value), grad(s.grad) { }

    // ======================================================================
    /// @{ \name Addition
    // ======================================================================
    friend StaticDFloat operator+(const StaticDFloat &lhs, const StaticDFloat &rhs) {
        return StaticDFloat(lhs.value + rhs.value, lhs.grad + rhs.grad);
    }

    friend StaticDFloat operator+(const StaticDFloat &lhs, const Scalar &rhs) {
        return StaticDFloat(lhs.value + rhs, lhs.grad);
    }

    friend StaticDFloat operator+(const Scalar &lhs, const StaticDFloat &rhs) {
        return StaticDFloat(rhs.value + lhs, rhs.grad);
    }

    inline StaticDFloat& operator+=(const StaticDFloat &s) {
        value += s.value;
        grad += s.grad;
        return *this;
    }

    inline StaticDFloat& operator+=(const Scalar &v) {
        value += v;
        return *this;
    }

    /// @}
    // ======================================================================

    // ======================================================================
    /// @{ \name Subtraction
    // ======================================================================

    friend StaticDFloat operator-(const StaticDFloat &lhs, const StaticDFloat &rhs) {
        return StaticDFloat(lhs.value - rhs.value, lhs.grad - rhs.grad);
    }

    friend StaticDFloat operator-(const StaticDFloat &lhs, const Scalar &rhs) {
        return StaticDFloat(lhs.value - rhs, lhs.grad);
    }

    friend StaticDFloat operator-(const Scalar &lhs, const StaticDFloat &rhs) {
        return StaticDFloat(lhs - rhs.value, -rhs.grad);
    }

    friend StaticDFloat operator-(const StaticDFloat &s) {
        return StaticDFloat(-s.value, -s.grad);
    }

    inline StaticDFloat& operator-=(const StaticDFloat &s) {
        value -= s.value;
        grad -= s.grad;
        return *this;
    }

    inline StaticDFloat& operator-=(const Scalar &v) {
        value -= v;
        return *this;
    }
    /// @}
    // ======================================================================

    // ======================================================================
    /// @{ \name Division
    // ======================================================================
    friend StaticDFloat operator/(const StaticDFloat &lhs, const Scalar &rhs) {
        if (rhs == 0)
            throw std::runtime_error("StaticDFloat: Division by zero!");
        Scalar inv = 1.0f / rhs;
        return StaticDFloat(lhs.value*inv, lhs.grad*inv);
    }

    friend StaticDFloat operator/(const Scalar &lhs, const StaticDFloat &rhs) {
        return lhs * inverse(rhs);
    }

    friend StaticDFloat operator/(const StaticDFloat &lhs, const StaticDFloat &rhs) {
        return lhs * inverse(rhs);
    }

    friend StaticDFloat inverse(const StaticDFloat &s) {
        Scalar valueSqr = s.value*s.value,
            invValueSqr = (Scalar)1 / valueSqr;

        // vn = 1/v, Dvn = -1/(v^2) Dv
        return StaticDFloat((Scalar)1 / s.value, s.grad * -invValueSqr);
    }

    inline StaticDFloat& operator/=(const Scalar &v) {
        value /= v;
        grad /= v;
        return *this;
    }

    /// @}
    // ======================================================================

    // ======================================================================
    /// @{ \name Multiplication
    // ======================================================================
    inline friend StaticDFloat operator*(const StaticDFloat &lhs, const Scalar &rhs) {
        return StaticDFloat(lhs.value*rhs, lhs.grad*rhs);
    }

    inline friend StaticDFloat operator*(const Scalar &lhs, const StaticDFloat &rhs) {
        return StaticDFloat(rhs.value*lhs, rhs.grad*lhs);
    }

    inline friend StaticDFloat operator*(const StaticDFloat &lhs, const StaticDFloat &rhs) {
        // Product rule
        return StaticDFloat(lhs.value*rhs.value,
            rhs.grad * lhs.value + lhs.grad * rhs.value);
    }

    inline StaticDFloat& operator*=(const Scalar &v) {
        value *= v;
        grad *= v;
        return *this;
    }

    inline StaticDFloat& operator*=(const StaticDFloat &v) {
        grad = v.grad * value + grad * v.value;
        value *= v.value;
        return *this;
    }

    /// @}
    // ======================================================================

    // ======================================================================
    /// @{ \name Comparison and assignment
    // ======================================================================

    inline void operator=(const StaticDFloat& s) { value = s.value; grad = s.grad; }
    inline void operator=(const Scalar &v) { value = v; grad = Gradient(0); }
    inline bool operator<(const StaticDFloat& s) const { return value < s.value; }
    inline bool operator<=(const StaticDFloat& s) const { return value <= s.value; }
    inline bool operator>(const StaticDFloat& s) const { return value > s.value; }
    inline bool operator>=(const StaticDFloat& s) const { return value >= s.value; }
    inline bool operator<(const Scalar& s) const { return value < s; }
    inline bool operator<=(const Scalar& s) const { return value <= s; }
    inline bool operator>(const Scalar& s) const { return value > s; }
    inline bool operator>=(const Scalar& s) const { return value >= s; }
    inline bool operator==(const Scalar& s) const { return value == s; }
    inline bool operator!=(const Scalar& s) const { return value != s; }

    /// @}
    // ======================================================================

    // ======================================================================
    /// @{ \name Comparison and assignment
    // ======================================================================
    Scalar value;
    Gradient grad;
};

template <typename Scalar, typename Gradient>
std::ostream &operator<<(std::ostream &out, const StaticDFloat<Scalar, Gradient> &s) {
    out << "[" << s.value
        << ", grad=" << s.grad
        << "]";
    return out;
}

namespace diffutil
{
/// @}
// ======================================================================

// ======================================================================
/// @{ \name Math functions
// ======================================================================

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> sqrt(const StaticDFloat<Scalar, Gradient> &s) {
    Scalar sqrtVal = std::sqrt(s.value),
        temp = (Scalar)1 / ((Scalar)2 * sqrtVal);

    // vn = sqrt(v)
    // Dvn = 1/(2 sqrt(v)) Dv
    return StaticDFloat<Scalar, Gradient>(sqrtVal, s.grad * temp);
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> pow(const StaticDFloat<Scalar, Gradient> &s, const Scalar &a) {
    Scalar powVal = std::pow(s.value, a),
        temp = a * std::pow(s.value, a - 1);
    // vn = v ^ a, Dvn = a*v^(a-1) * Dv
    return StaticDFloat<Scalar, Gradient>(powVal, s.grad * temp);
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> exp(const StaticDFloat<Scalar, Gradient> &s) {
    Scalar expVal = std::exp(s.value);

    // vn = exp(v), Dvn = exp(v) * Dv
    return StaticDFloat<Scalar, Gradient>(expVal, s.grad * expVal);
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> log(const StaticDFloat<Scalar, Gradient> &s) {
    Scalar logVal = std::log(s.value);

    // vn = log(v), Dvn = Dv / v
    return StaticDFloat<Scalar, Gradient>(logVal, s.grad / s.value);
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> sin(const StaticDFloat<Scalar, Gradient> &s) {
    // vn = sin(v), Dvn = cos(v) * Dv
    return StaticDFloat<Scalar, Gradient>(std::sin(s.value), s.grad * std::cos(s.value));
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> cos(const StaticDFloat<Scalar, Gradient> &s) {
    // vn = cos(v), Dvn = -sin(v) * Dv
    return StaticDFloat<Scalar, Gradient>(std::cos(s.value), s.grad * -std::sin(s.value));
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> acos(const StaticDFloat<Scalar, Gradient> &s) {
    if (std::abs(s.value) >= 1)
        throw std::runtime_error("acos: Expected a value in (-1, 1)");

    Scalar temp = -std::sqrt((Scalar)1 - s.value*s.value);

    // vn = acos(v), Dvn = -1/sqrt(1-v^2) * Dv
    return StaticDFloat<Scalar, Gradient>(std::acos(s.value),
        s.grad * ((Scalar)1 / temp));
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> asin(const StaticDFloat<Scalar, Gradient> &s) {
    if (std::abs(s.value) >= 1)
        throw std::runtime_error("asin: Expected a value in (-1, 1)");

    Scalar temp = std::sqrt((Scalar)1 - s.value*s.value);

    // vn = asin(v), Dvn = 1/sqrt(1-v^2) * Dv
    return StaticDFloat<Scalar, Gradient>(std::asin(s.value),
        s.grad * ((Scalar)1 / temp));
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> atan2(const StaticDFloat<Scalar, Gradient> &y, const StaticDFloat<Scalar, Gradient> &x) {
    Scalar denom = x.value*x.value + y.value*y.value;

    // vn = atan2(y, x), Dvn = (x*Dy - y*Dx) / (x^2 + y^2)
    return StaticDFloat<Scalar, Gradient>(std::atan2(y.value, x.value),
        y.grad * (x.value / denom) - x.grad * (y.value / denom));
}

//////////////////////////////////////////////////////////////////////////
// These functions are strictly not differentiable...
template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> abs(const StaticDFloat<Scalar, Gradient> &x) {
    if (x.value >= 0.0) {
        return x;
    } else {
        return -x;
    }
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> clamp(const StaticDFloat<Scalar, Gradient> &x, Float minValue, Float maxValue) {
    StaticDFloat<Scalar, Gradient> ret;
    ret.value = math::clamp(x.value, minValue, maxValue);
    ret.grad = x.grad;
    return ret;
}

template <typename Scalar, typename Gradient>
inline StaticDFloat<Scalar, Gradient> safe_acos(const StaticDFloat<Scalar, Gradient> &x) {
    return acos(clamp(x, Float(-0.999), Float(0.999)));
}

}


//////////////////////////////////////////////////////////////////////////

using DFloat = StaticDFloat<Float, Float>;
using DVector2 = TVector2<DFloat>;
using DVector = TVector3<DFloat>;
using DVector3 = TVector3<DFloat>;
using DVector4 = TVector4<DFloat>;
using DPoint = TPoint3<DFloat>;
using DSpectrum = TSpectrum<DFloat, SPECTRUM_SAMPLES>;

// The Matrix<M, N, T> base class has a bunch of irrelevant operations that cause compiling errors...
struct MTS_EXPORT_RENDER DMatrix3x3 {

    DFloat m[3][3];

    inline DMatrix3x3() = default;

    explicit inline DMatrix3x3(DFloat dvalue) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] = dvalue;
            }
        }
    }

    explicit inline DMatrix3x3(const Matrix3x3 &mtx) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] = DFloat(mtx.m[i][j]);
            }
        }
    }

    /// Matrix-vector multiplication
    template <typename VecType,
        typename = std::enable_if_t<
        std::is_same<VecType, Vector>::value ||
        std::is_same<VecType, DVector>::value>>
    inline DVector operator*(const VecType &v) const {
        return DVector(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
    }

    /// Scalar multiplication (creates a temporary)
    template <typename T,
        typename = std::enable_if_t<
        std::is_same<T, Float>::value ||
        std::is_same<T, DFloat>::value>>
    inline DMatrix3x3 operator*(T scalar) const {
        DMatrix3x3 result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.m[i][j] = m[i][j] * scalar;
        return result;
    }
};

template <typename MtxType,
    typename = std::enable_if_t<
    std::is_same<MtxType, DMatrix3x3>::value ||
    std::is_same<MtxType, Matrix3x3>::value>>
inline DMatrix3x3 operator*(const DMatrix3x3 &dmtx, const MtxType &mtx) {
    DMatrix3x3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            DFloat sum(0);
            for (int k = 0; k < 3; ++k)
                sum += dmtx.m[i][k] * mtx.m[k][j];
            result.m[i][j] = sum;
        }
    }
    return result;
}

// ...also need a DMatrix4x4
struct MTS_EXPORT_RENDER DMatrix4x4 {
    DFloat m[4][4];

    inline DMatrix4x4() = default;

    explicit inline DMatrix4x4(DFloat dvalue) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = dvalue;
            }
        }
    }

    explicit inline DMatrix4x4(const Matrix4x4 &mtx) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] = DFloat(mtx.m[i][j]);
            }
        }
    }

    void setIdentity() {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m[i][j] = (i == j) ? 1.0f : 0.0f;
    }

    /// Matrix-vector multiplication
    template <typename VecType,
        typename = std::enable_if_t<
        std::is_same<VecType, Vector4>::value ||
        std::is_same<VecType, DVector4>::value>>
        inline DVector4 operator*(const VecType &v) const {
        return DVector4(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
            m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w);
    }

    /// Scalar multiplication (creates a temporary)
    template <typename T,
        typename = std::enable_if_t<
        std::is_same<T, Float>::value ||
        std::is_same<T, DFloat>::value>>
        inline DMatrix4x4 operator*(T scalar) const {
        DMatrix4x4 result;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                result.m[i][j] = m[i][j] * scalar;
        return result;
    }

    bool invert(DMatrix4x4 &target) const {
        using diffutil::abs;

        int indxc[4], indxr[4];
        int ipiv[4];
        memset(ipiv, 0, sizeof(int) * 4);
        memcpy(target.m, m, 4 * 4 * sizeof(DFloat));

        for (int i = 0; i < 4; i++) {
            int irow = -1, icol = -1;
            DFloat big = 0;
            for (int j = 0; j < 4; j++) {
                if (ipiv[j] != 1) {
                    for (int k = 0; k < 4; k++) {
                        if (ipiv[k] == 0) {
                            if (abs(target.m[j][k]) >= big) {
                                big = abs(target.m[j][k]);
                                irow = j;
                                icol = k;
                            }
                        } else if (ipiv[k] > 1) {
                            return false;
                        }
                    }
                }
            }
            ++ipiv[icol];
            if (irow != icol) {
                for (int k = 0; k < 4; ++k)
                    std::swap(target.m[irow][k], target.m[icol][k]);
            }
            indxr[i] = irow;
            indxc[i] = icol;
            if (target.m[icol][icol] == 0)
                return false;
            DFloat pivinv = 1.f / target.m[icol][icol];
            target.m[icol][icol] = 1.f;
            for (int j = 0; j < 4; j++)
                target.m[icol][j] *= pivinv;
            for (int j = 0; j < 4; j++) {
                if (j != icol) {
                    DFloat save = target.m[j][icol];
                    target.m[j][icol] = 0;
                    for (int k = 0; k < 4; k++)
                        target.m[j][k] -= target.m[icol][k] * save;
                }
            }
        }
        for (int j = 4 - 1; j >= 0; j--) {
            if (indxr[j] != indxc[j]) {
                for (int k = 0; k < 4; k++)
                    std::swap(target.m[k][indxr[j]], target.m[k][indxc[j]]);
            }
        }
        return true;
    }
};

template <typename MtxType,
    typename = std::enable_if_t<
    std::is_same<MtxType, DMatrix4x4>::value ||
    std::is_same<MtxType, Matrix4x4>::value>>
    inline DMatrix4x4 operator*(const DMatrix4x4 &dmtx, const MtxType &mtx) {
    DMatrix4x4 result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            DFloat sum(0);
            for (int k = 0; k < 4; ++k)
                sum += dmtx.m[i][k] * mtx.m[k][j];
            result.m[i][j] = sum;
        }
    }
    return result;
}

// Yeah...also need a DTransform
struct MTS_EXPORT_RENDER DTransform {
public:
    DTransform() {
        m_transform.setIdentity();
        m_invTransform.setIdentity();
    }

    DTransform(const DMatrix4x4 &trafo)
        : m_transform(trafo) {
        bool success = m_transform.invert(m_invTransform);
        SAssert(success);
    }

    DTransform(const DMatrix4x4 &trafo, const DMatrix4x4 &invTrafo)
        : m_transform(trafo), m_invTransform(invTrafo) {
    }

    DTransform inverse() const {
        return DTransform(m_invTransform, m_transform);
    }

    template <typename TransformType,
        typename = std::enable_if_t<
        std::is_same<TransformType, Transform>::value ||
        std::is_same<TransformType, DTransform>::value>>
    DTransform operator*(const TransformType &t) const {
        return DTransform(m_transform * t.m_transform,
            t.m_invTransform * m_invTransform);
    }

    template <typename PointType,
        typename = std::enable_if_t<
        std::is_same<PointType, Point>::value ||
        std::is_same<PointType, DPoint>::value>>
    inline DPoint operator()(const PointType &p) const {
        DFloat x = m_transform.m[0][0] * p.x + m_transform.m[0][1] * p.y
            + m_transform.m[0][2] * p.z + m_transform.m[0][3];
        DFloat y = m_transform.m[1][0] * p.x + m_transform.m[1][1] * p.y
            + m_transform.m[1][2] * p.z + m_transform.m[1][3];
        DFloat z = m_transform.m[2][0] * p.x + m_transform.m[2][1] * p.y
            + m_transform.m[2][2] * p.z + m_transform.m[2][3];
        DFloat w = m_transform.m[3][0] * p.x + m_transform.m[3][1] * p.y
            + m_transform.m[3][2] * p.z + m_transform.m[3][3];

        SAssert(w != 0);
        if (w == 1.0f)
            return DFloat(x, y, z);
        else
            return DPoint(x, y, z) / w;
    }

    template <typename PointType,
        typename = std::enable_if_t<
        std::is_same<PointType, Point>::value ||
        std::is_same<PointType, DPoint>::value>>
    inline DPoint transformAffine(const PointType &p) const {
        DFloat x = m_transform.m[0][0] * p.x + m_transform.m[0][1] * p.y
            + m_transform.m[0][2] * p.z + m_transform.m[0][3];
        DFloat y = m_transform.m[1][0] * p.x + m_transform.m[1][1] * p.y
            + m_transform.m[1][2] * p.z + m_transform.m[1][3];
        DFloat z = m_transform.m[2][0] * p.x + m_transform.m[2][1] * p.y
            + m_transform.m[2][2] * p.z + m_transform.m[2][3];
        return DPoint(x, y, z);
    }

private:
    DMatrix4x4 m_transform;
    DMatrix4x4 m_invTransform;
};

MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_DIFFUTIL_H_ */
