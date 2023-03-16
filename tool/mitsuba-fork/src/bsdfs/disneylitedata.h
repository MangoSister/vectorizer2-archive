#if !defined(__DISNEY_LITE_DATA_H)
#define __DISNEY_LITE_DATA_H

#include <mitsuba/mitsuba.h>
#include <mitsuba/render/mipmap.h>

#include <vectorizer/vectorizer.h>

MTS_NAMESPACE_BEGIN

namespace disneylite
{

struct LookupEntry
{
    float m00, m02, m20, m22;
    float nd;
    float fd;
};

inline LookupEntry operator+(const LookupEntry &e1, const LookupEntry &e2)
{
    LookupEntry ret = e1;
    ret.m00 += e2.m00;
    ret.m02 += e2.m02;
    ret.m20 += e2.m20;
    ret.m22 += e2.m22;

    ret.nd += e2.nd;
    ret.fd += e2.fd;

    return ret;
}

inline LookupEntry operator-(const LookupEntry &e1, const LookupEntry &e2)
{
    LookupEntry ret = e1;
    ret.m00 -= e2.m00;
    ret.m02 -= e2.m02;
    ret.m20 -= e2.m20;
    ret.m22 -= e2.m22;

    ret.nd -= e2.nd;
    ret.fd -= e2.fd;

    return ret;
}

inline LookupEntry operator*(const LookupEntry &e, Float s)
{
    LookupEntry ret = e;
    ret.m00 *= s;
    ret.m02 *= s;
    ret.m20 *= s;
    ret.m22 *= s;

    ret.nd *= s;
    ret.fd *= s;

    return ret;
}

inline LookupEntry operator*(Float s, const LookupEntry &e)
{
    return e * s;
}

struct LookupEntryAD
{
    vtrz::dfloat m00, m02, m20, m22;
    vtrz::dfloat nd;
    vtrz::dfloat fd;

    LookupEntryAD() = default;

    inline LookupEntryAD(
        const vtrz::dfloat &m00, const vtrz::dfloat &m02,
        const vtrz::dfloat &m20, const vtrz::dfloat &m22,
        const vtrz::dfloat &nd, const vtrz::dfloat &fd) :
        m00(m00), m02(m02), m20(m20), m22(m22), nd(nd), fd(fd) {}
};

class LookupTable : public Object
{
public:
    LookupTable();
    LookupEntry eval(Float roughness, Float NdotV) const;
    LookupEntryAD evalAD(vtrz::dfloat roughness, vtrz::dfloat NdotV) const;

    Float evalSphereFormFactor(Float elevation, Float vecFormFactorLen) const;
    vtrz::dfloat evalSphereFormFactorAD(const vtrz::dfloat &elevation, const vtrz::dfloat &vecFormFactorLen) const;

    BlockedArray<LookupEntry> entries;
    BlockedArray<float> sphereFormFactors;

    MTS_DECLARE_CLASS()
protected:
    virtual ~LookupTable() = default;
};

}

MTS_NAMESPACE_END

#endif /* __DISNEY_LITE_DATA_H */
