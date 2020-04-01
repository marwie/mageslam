#pragma once

#include "Proxies\Proxy.h"
#include "Proxies\KeyframeFields.h"

namespace mage
{
    using MappingKeyframe = Proxy<Keyframe, proxy::Image, proxy::Pose, proxy::Intrinsics, proxy::UnAssociatedMask>;
}