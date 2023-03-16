#pragma once
#if !defined(__MITSUBA_RENDER_DERIVATIVE_H_)
#define __MITSUBA_RENDER_DERIVATIVE_H_

#include <mitsuba/core/cobject.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/render/common.h>

MTS_NAMESPACE_BEGIN

struct MTS_EXPORT_RENDER DerivativeTarget final : public ConfigurableObject
{
public:
    MTS_DECLARE_CLASS()

    DerivativeTarget(const Properties &props);

    DerivativeTarget(Stream *stream, InstanceManager *manager);

    void serialize(Stream *stream, InstanceManager *manager) const;

private:
    std::string m_targetObject;
    ref<ConfigurableObject> m_targetObjectRef;
    std::string m_targetParameter;
};

MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_DERIVATIVE_H_ */
