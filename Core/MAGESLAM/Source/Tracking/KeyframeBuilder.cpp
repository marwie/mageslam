// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "KeyframeBuilder.h"
#include "Debugging/SkeletonLogger.h"

namespace mage
{
    IdGenerator<Keyframe> KeyframeBuilder::s_generator{};

    // The Keyframe proxy is initialized with a set of predicted intrinsics that stays with the image object.

    KeyframeBuilder::KeyframeBuilder(
        const std::shared_ptr<const AnalyzedImage>& image,
        const mage::Pose& pose,
        const std::vector<MapPointAssociations<MapPointTrackingProxy>::Association>& mapPoints)
        : KeyframeProxy{ s_generator.generate(), proxy::Image{image}, proxy::Pose{pose}, proxy::Intrinsics{{ ArrayFromMat<float, size_t(4), size_t(1)>(image->GetUndistortedCalibration().GetLinearIntrinsics()), image->GetWidth(), image->GetHeight()}}, proxy::Associations<MapPointTrackingProxy>{ image, mapPoints }, proxy::PoseConstraints{ false } }
    {
        SkeletonLogger::ImageLogging::LogImage(*this);
    }

    KeyframeBuilder::KeyframeBuilder(
        const std::shared_ptr<const AnalyzedImage>& image,
        const mage::Pose& pose) 
        : KeyframeProxy{ s_generator.generate(), proxy::Image{ image }, proxy::Pose{ pose }, proxy::Intrinsics{{ ArrayFromMat<float, size_t(4), size_t(1)>(image->GetUndistortedCalibration().GetLinearIntrinsics()), image->GetWidth(),image->GetHeight()}}, proxy::Associations<MapPointTrackingProxy>{ image }, proxy::PoseConstraints{false}}
    {
        SkeletonLogger::ImageLogging::LogImage(*this);
    }
}
