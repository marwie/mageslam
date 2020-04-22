#include "FuserFunctions.h"

#include <BaseLib/SensorTime.h>

namespace mage
{
    const mage::SensorSample* FindCorrespondingAccelSample(const mage::SensorSample& gyro, gsl::span<const mage::SensorSample> samples)
    {
        std::ptrdiff_t gyroIndex = &gyro - samples.data();

        auto nextAccel = std::find_if(samples.begin() + gyroIndex, samples.end(), [](const mage::SensorSample& sample)
        {
            return sample.GetType() == mage::SensorSample::SampleType::Accelerometer;
        });

        std::ptrdiff_t gyroRIndex = samples.size() - gyroIndex - 1;

        auto prevAccel = std::find_if(samples.rbegin() + gyroRIndex, samples.rend(), [](const mage::SensorSample& sample)
        {
            return sample.GetType() == mage::SensorSample::SampleType::Accelerometer;
        });

        std::vector<const mage::SensorSample*> found;
        if (nextAccel != samples.end())
        {
            found.push_back(&*nextAccel);
        }
        if (prevAccel != samples.rend())
        {
            found.push_back(&*prevAccel);
        }

        auto closest = std::min_element(found.begin(), found.end(), [&](const mage::SensorSample* left, const mage::SensorSample* right)
        {
            return std::abs((gyro.GetTimestamp() - left->GetTimestamp()).count()) <
                std::abs((gyro.GetTimestamp() - right->GetTimestamp()).count());
        });

        if (closest == found.end())
            return nullptr;

        return *closest;
    }

    ST::IMUConfig BuildIMUConfigFromCharacterization(const device::IMUCharacterization& imuCharacterization)
    {
        ST::IMUConfig config{};
        config.IMUType = ST::InertialMeasurementUnitId::CALIBRATION_InertialSensorId_Undefined;
        config.AccelToGyro = ST::SO3_identity<float>();

        const float accelNoiseSigma[3] = { imuCharacterization.AccelNoiseSigma, imuCharacterization.AccelNoiseSigma, imuCharacterization.AccelNoiseSigma };
        const float gyroNoiseSigma[3] = { imuCharacterization.GyroNoiseSigma,  imuCharacterization.GyroNoiseSigma , imuCharacterization.GyroNoiseSigma };
        const float accelBiasVar[3] = { imuCharacterization.AccelBiasSigma * imuCharacterization.AccelBiasSigma, imuCharacterization.AccelBiasSigma * imuCharacterization.AccelBiasSigma, imuCharacterization.AccelBiasSigma * imuCharacterization.AccelBiasSigma };
        const float gyroBiasVar[3] = { imuCharacterization.GyroBiasSigma * imuCharacterization.GyroBiasSigma, imuCharacterization.GyroBiasSigma * imuCharacterization.GyroBiasSigma, imuCharacterization.GyroBiasSigma * imuCharacterization.GyroBiasSigma };

        config.SetNoise(accelNoiseSigma, gyroNoiseSigma, imuCharacterization.MagNoiseSigma);
        config.SetInitialBiasVariance(accelBiasVar, gyroBiasVar, imuCharacterization.MagBiasSigma *  imuCharacterization.MagBiasSigma);

        return config;
    }

    ST::SensorTime::time_t ToExternalSensorTime(const SensorSample::Timestamp& timestamp)
    {
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch());
        return ST::SensorTime::fromMicroseconds(microseconds.count());
    }
}
