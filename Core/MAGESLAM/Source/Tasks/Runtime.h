// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include "BaseWorker.h"

#include "MageSlam.h"

#include <arcana/scheduling/state_machine.h>
#include <arcana/threading/dispatcher.h>
#include <gsl/gsl>
#include <future>
#include <memory>

namespace mage
{
    struct MageContext;
    struct MageSlamSettings;
    struct FrameData;

    class Fuser;

    class Runtime
    {
    public:
        struct ThreadingModel
        {
            // Max size 72 is required for x64 builds
            using DispatcherT = mira::dispatcher<72>;
            DispatcherT& RuntimeDispatcher;
            DispatcherT& TrackingDispatcher;
            DispatcherT& MappingDispatcher;
        };
        Runtime(const MageSlamSettings& settings, MageContext& context, Fuser& fuser, mira::state_machine_driver& driver, ThreadingModel& threadingModel);
        ~Runtime();

        void Run(gsl::span<const MAGESlam::CameraConfiguration> cameras);
        std::future<void> DisposeAsync();

        void TrackMono(std::shared_ptr<FrameData> frame);
        void TrackStereo(std::shared_ptr<FrameData> one, std::shared_ptr<FrameData> two);

        void AddSample(const mage::SensorSample& sample);
    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
