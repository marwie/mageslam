#pragma once

#include "Interfaces.h"

#include "Device/IMUCharacterization.h"

namespace mage
{
    class AnalogVFTFilter;

    // This is a wrapper around the 6dof analog IMUFilterCAS which
    // tries to estimate scale as it's fusing the visual and inertial estimates.
    class AnalogCASIMU final :
        public IPredictor,
        public IIMUReceiver,
        public IVisualPoseReceiver,
        public IPoseEstimator
    {
    public:
        AnalogCASIMU(const device::IMUCharacterization& imuCharacterization, bool assumeSamePosition);
        AnalogCASIMU(const AnalogVFTFilter& filter, const mage::Pose& pose, mage::SensorSample::Timestamp time, bool assumeSamePosition);

        ~AnalogCASIMU();

        mage::Pose GetPose() const override;

        void AddSample(const mage::SensorSample& sample) override;

        void PredictUpTo(mage::SensorSample::Timestamp timestamp) override;

        void AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp) override;

        void OnTrackingLost() override;
    private:
        struct Impl;
        std::shared_ptr<Impl> m_impl;
    };
}
