#include <mitsuba/render/scene.h>
#include <mitsuba/core/frame.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/render/derivative.h>

MTS_NAMESPACE_BEGIN

DerivativeTarget::DerivativeTarget(const Properties &props)
    : ConfigurableObject(props)
{
    m_targetObject = props.getString("object", std::string());
    m_targetParameter = props.getString("parameter", std::string());
}

DerivativeTarget::DerivativeTarget(Stream *stream, InstanceManager *manager)
    : ConfigurableObject(stream, manager)
{
    m_targetObject = stream->readString();
    m_targetParameter = stream->readString();
}

void DerivativeTarget::serialize(Stream *stream, InstanceManager *manager) const
{
    ConfigurableObject::serialize(stream, manager);
    stream->writeString(m_targetObject);
    stream->writeString(m_targetParameter);
}

MTS_IMPLEMENT_CLASS_S(DerivativeTarget, false, ConfigurableObject)
MTS_EXPORT_PLUGIN(DerivativeTarget, "Derivative target");
MTS_NAMESPACE_END
