#pragma once

#include "Interfaces.h"

#include "Device/IMUCharacterization.h"

namespace mage
{
    class AnalogCAIMU final :
        public IPredictor,
        public IIMUReceiver,
        public IVisualPoseReceiver,
        public IPoseEstimator
    {
    public:
        AnalogCAIMU(const device::IMUCharacterization& imuCharacterization);
        ~AnalogCAIMU();

        mage::Pose GetPose() const override;

        void AddSample(const mage::SensorSample& sample) override;

        void PredictUpTo(mage::SensorSample::Timestamp timestamp) override;

        void AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp) override;

        float GetLinearAccelerationNorm() const;
        float GetLinearVelocityNorm() const;
    private:
        struct Impl;
        std::shared_ptr<Impl> m_impl;
    };
}
