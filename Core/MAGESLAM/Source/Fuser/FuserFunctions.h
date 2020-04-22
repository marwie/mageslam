#pragma once

#include "SensorSample.h"

#include <gsl/gsl>

#include <FuserLib/IMUConfig.h>
#include <Device/IMUCharacterization.h>

#include <BaseLib/SensorTime.h>

namespace mage
{
    const mage::SensorSample* FindCorrespondingAccelSample(const mage::SensorSample& gyro, gsl::span<const mage::SensorSample> samples);

    ST::IMUConfig BuildIMUConfigFromCharacterization(const device::IMUCharacterization& characterization);

    ST::SensorTime::time_t ToExternalSensorTime(const SensorSample::Timestamp& timestamp);
}
