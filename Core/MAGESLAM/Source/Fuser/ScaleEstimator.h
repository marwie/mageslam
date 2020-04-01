#pragma once

#include "Interfaces.h"

#include "Device/IMUCharacterization.h"

namespace mage
{
    class ScaleEstimator final
    {
    public:
        ScaleEstimator(const device::IMUCharacterization& imuCharacterization);
        ~ScaleEstimator();

        float GetScale() const;

        void AddEntry(float metricLinearAcceleration, const mage::Pose& pose, const std::chrono::system_clock::time_point& timestamp);

    private:
        struct Impl;
        std::shared_ptr<Impl> m_impl;
    };
}
