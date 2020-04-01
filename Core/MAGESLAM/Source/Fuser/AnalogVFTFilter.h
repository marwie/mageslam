#pragma once

#include "Interfaces.h"

#include "Device/IMUCharacterization.h"

namespace ST
{
    class VFTFilter;
}

namespace mage
{
    // This is a wrapper around the 3dof analog visual free filter (VFTFilter).
    class AnalogVFTFilter final :
        public IPredictor,
        public IIMUReceiver
    {
    public:
        AnalogVFTFilter(const device::IMUCharacterization& imuCharacterization);
        ~AnalogVFTFilter();

        void AddSample(const mage::SensorSample& sample) override;

        void PredictUpTo(mage::SensorSample::Timestamp timestamp) override;

        bool IsConverged() const;

        const device::IMUCharacterization& GetCharacterization() const;

        gsl::span<const mage::SensorSample> GetSamples() const;
    private:
        friend class AnalogCASIMU;

        const ST::VFTFilter& GetInternalFilter() const;

        struct Impl;
        std::shared_ptr<Impl> m_impl;
    };
}
