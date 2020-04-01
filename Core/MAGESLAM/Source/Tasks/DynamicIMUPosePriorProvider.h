#pragma once

#include "BaseWorker.h"

#include "Fuser/Interfaces.h"
#include "Tracking/IPosePriorProvider.h"

#include "Device/IMUCharacterization.h"

namespace mage
{
    class DynamicIMUPosePriorProvider : public BaseWorker, public IPosePriorProvider, public IIMUReceiver
    {
    public:
        DynamicIMUPosePriorProvider(const device::IMUCharacterization& characterization, bool assumeSamePosition);
        ~DynamicIMUPosePriorProvider();

        mira::task<Pose> GetPoseForTime(const TrackingFrameHistory& history, const std::chrono::system_clock::time_point& timestamp) override;
        void OnTrackingLost() override;

        void AddSample(const mage::SensorSample& sample) override;
    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
