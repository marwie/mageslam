//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#include "pch.h"
#include "SensorFilter.h"
#include "SimpleIMUFilter.h"
#include <BaseLib/SensorTime.h>
#include <analogbase.h>
#include <FuserLib/VFTFilter.h>
#include <FuserLib/IMUFilterEvent.h>
#include <FuserLib/IMUConfig.h>
#include <FuserLib/FuserUpdateStatus.h>
#include <FilterInitializationScaled.h>
#include <MathLib\MatrixOps.h>
#include <IMUFilterCAS.h>
#include "AnalogConversions.h"
#include <arcana/math.h>
#include <assert.h>
#include <set>

namespace
{
    template <typename T>
    static constexpr T SCALE_EPSILON = static_cast<T>(1e-5);

    ST::SensorTime::time_t ConvertToSensorTime(const mage::SensorSample::Timestamp& inTime)
    {
        std::chrono::microseconds timestampMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(inTime.time_since_epoch());
        return ST::SensorTime::fromMicroseconds(timestampMicroseconds.count());
    }

    //Negate the lower level filter's data to get world space vector
    std::array<float, 3> NegateVector(float data0, float data1, float data2)
    {
        return{ -data0, -data1, -data2 };
    }
}

namespace mage
{
    namespace
    {
        bool AllDifferentSampleTypes(const std::vector<SensorSample>& samples)
        {
            std::set<SensorSample::SampleType> types;

            for (const auto& el : samples)
            {
                if (!types.insert(el.GetType()).second)
                    return false;
            }

            return true;
        }

        void CopySampleData(float dest[3], const std::vector<float>& data)
        {
            dest[0] = data[0];
            dest[1] = data[1];
            dest[2] = data[2];
        }
    }

    template <typename F>
    SensorFilter<F>::SensorFilter(const mage::device::IMUCharacterization& imuCharacterization) :
        m_imuCharacterization(imuCharacterization)
    {
        m_config = std::make_unique<ST::IMUConfig>();
        m_config->IMUType = ST::InertialMeasurementUnitId::CALIBRATION_InertialSensorId_Undefined;

        const float accelNoiseSigma[3] = { imuCharacterization.AccelNoiseSigma, imuCharacterization.AccelNoiseSigma, imuCharacterization.AccelNoiseSigma };
        const float gyroNoiseSigma[3] = { imuCharacterization.GyroNoiseSigma,  imuCharacterization.GyroNoiseSigma , imuCharacterization.GyroNoiseSigma };
        const float accelBiasVar[3] = { imuCharacterization.AccelBiasSigma * imuCharacterization.AccelBiasSigma, imuCharacterization.AccelBiasSigma * imuCharacterization.AccelBiasSigma, imuCharacterization.AccelBiasSigma * imuCharacterization.AccelBiasSigma };
        const float gyroBiasVar[3] = { imuCharacterization.GyroBiasSigma * imuCharacterization.GyroBiasSigma, imuCharacterization.GyroBiasSigma * imuCharacterization.GyroBiasSigma, imuCharacterization.GyroBiasSigma * imuCharacterization.GyroBiasSigma };
        m_config->SetNoise(accelNoiseSigma, gyroNoiseSigma, imuCharacterization.MagNoiseSigma);
        m_config->SetInitialBiasVariance(accelBiasVar, gyroBiasVar, imuCharacterization.MagBiasSigma *  imuCharacterization.MagBiasSigma);

        m_filter = std::make_unique<F>();
    }

    template <typename F>
    SensorFilter<F>::~SensorFilter()
    { }

    template <typename F>
    bool SensorFilter<F>::GetWorldGravity(std::array<float, 3>& gravityWorld) const
    {
        gravityWorld = { 0,0,0 };

        if (m_initialized == false)
        {
            assert(false && "filter not initialized yet");
            return false;
        }

        {
            std::shared_lock<std::shared_mutex> guard(m_mutex);
            gravityWorld = NegateVector((float)m_filter->g.data()[0], (float)m_filter->g.data()[1], (float)m_filter->g.data()[2]);

            if (!m_filter->has_good_gravity())
            {
                return false;
            }
        }

        return true;
    }

    template <typename F>
    std::array<float, 3> SensorFilter<F>::GetBodyLinearVelocity(SensorSample::Timestamp& fromTime) const
    {
        if (m_initialized == false)
        {
            assert(false && "filter not initialized yet");
            fromTime = SensorSample::Timestamp{ SensorSample::Timestamp::duration(0) };
            return{ 0,0,0 };
        }

        std::shared_lock<std::shared_mutex> guard(m_mutex);
        fromTime = m_lastProcessedTimestamp;
        return NegateVector((float)m_filter->u.data()[0], (float)m_filter->u.data()[1], (float)m_filter->u.data()[2]);
    }

    template <typename F>
    std::array<float, 3> SensorFilter<F>::GetBodyAngularVelocity(SensorSample::Timestamp& fromTime) const
    {
        if (m_initialized == false)
        {
            assert(false && "filter not initialized yet");
            fromTime = SensorSample::Timestamp{ SensorSample::Timestamp::duration(0) };
            return{ 0,0,0 };
        }

        std::shared_lock<std::shared_mutex> guard(m_mutex);
        fromTime = m_lastProcessedTimestamp;
        return NegateVector((float)m_filter->w.data()[0], (float)m_filter->w.data()[1], (float)m_filter->w.data()[2]);
    }

    template <typename F>
    std::array<float, 3> SensorFilter<F>::GetBodyLinearAcceleration(SensorSample::Timestamp& fromTime) const
    {
        if (m_initialized == false)
        {
            assert(false && "filter not initialized yet");
            fromTime = SensorSample::Timestamp{ SensorSample::Timestamp::duration(0) };
            return{ 0,0,0 };
        }

        std::shared_lock<std::shared_mutex> guard(m_mutex);
        fromTime = m_lastProcessedTimestamp;
        return NegateVector((float)m_filter->a.data()[0], (float)m_filter->a.data()[1], (float)m_filter->a.data()[2]);
    }

    template <typename F>
    std::array<float, 3> SensorFilter<F>::GetBodyAngularAcceleration(SensorSample::Timestamp& fromTime) const
    {
        if (m_initialized == false)
        {
            assert(false && "filter not initialized yet");
            fromTime = SensorSample::Timestamp{ SensorSample::Timestamp::duration(0) };
            return{ 0,0,0 };
        }

        std::shared_lock<std::shared_mutex> guard(m_mutex);
        fromTime = m_lastProcessedTimestamp;
        return NegateVector((float)m_filter->r.data()[0], (float)m_filter->r.data()[1], (float)m_filter->r.data()[2]);
    }

    template<typename F>
    std::array<float, 4 * 4> SensorFilter<F>::GetDeltaPose(const SensorSample::Timestamp & /*fromTime*/) const
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        assert(false && "Not implemented");
        return{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    }

    template <typename F>
    void SensorFilter<F>::Reset(const SensorSample::Timestamp& resetTime)
    {
        std::unique_lock<std::shared_mutex> guard(m_mutex);
        m_filter->reset(ConvertToSensorTime(resetTime), *m_config.get());
        ResetInternal(resetTime);
    }

    template <typename F>
    void SensorFilter<F>::ResetInternal(const SensorSample::Timestamp& resetTime)
    {
        m_lastProcessedTimestamp = resetTime;
        m_resetTimestamp = resetTime;
        m_initialized = true;
        OnResetInternal(resetTime);
    }

    //
    // Process all samples at a given timestamp
    // requirement: all sensor samples must be for the same timestamp (within 100 us)
    //
    template <typename F>
    std::chrono::system_clock::time_point SensorFilter<F>::ProcessCorrelatedSamples(const std::vector<SensorSample>& sampleEvents)
    {
        if (sampleEvents.empty() || !IsValid())
            return m_lastProcessedTimestamp;

        auto sampleTimestamp = sampleEvents.front().GetTimestamp();
        if (sampleTimestamp < GetLastProcessedTimestamp())
        {
            return m_lastProcessedTimestamp;
        }

        auto timestamp = ConvertToSensorTime(sampleTimestamp);
        assert(AllDifferentSampleTypes(sampleEvents) && "Cannot have duplicate sensor sample types");

        ST::IMUEvent evt{};
        evt.timestamp = timestamp;

        // saturate/invalidate everything because each of our events can
        // only be one type of event so we'll enable them piecemeal
        evt.m_isAccelSaturated = true;
        evt.m_isGyroSaturated = true;
        evt.magnetometer_timestamp = ST::SensorTime::invalidTime();

        // fill in IMUFilterEvent class
        for (const SensorSample& sampleEvent : sampleEvents)
        {
            switch (sampleEvent.GetType())
            {
            case SensorSample::SampleType::Accelerometer:
                evt.m_isAccelSaturated = false;
                assert(sampleEvent.GetData().size() == 3 && "accelerometer data has 3 components");
                CopySampleData(evt.accel, sampleEvent.GetData());
                break;
            case SensorSample::SampleType::Gyrometer:
                evt.m_isGyroSaturated = false;
                assert(sampleEvent.GetData().size() == 3 && "gyro data has 3 components");
                CopySampleData(evt.rotvel, sampleEvent.GetData());
                break;
            case SensorSample::SampleType::Magnetometer:
                if (m_imuCharacterization.useMagnetometer)
                {
                    evt.magnetometer_timestamp = evt.timestamp;
                    assert(sampleEvent.GetData().size() == 4 && "magnetometer data has 4 components");
                    CopySampleData(evt.mag, sampleEvent.GetData());
                }
                break;
            default:
                assert(false && "Unexpected sample type.");
            }
        }

        {
            std::unique_lock<std::shared_mutex> guard(m_mutex);
            assert(IsValidInternal() && "Valid filter");

            ST::FuserUpdateStatusType result = m_filter->predict_and_update(evt, *m_config);
            (void)result;

            m_lastProcessedTimestamp = sampleTimestamp;
        }

        return m_lastProcessedTimestamp;
    }

    template <typename F>
    void SensorFilter<F>::PredictInternal(const SensorSample::Timestamp& timestamp)
    {
        if (m_lastProcessedTimestamp <= timestamp && IsValidInternal())
        {
            m_filter->predict(ConvertToSensorTime(timestamp));
            m_lastProcessedTimestamp = timestamp;
        }
    }

    template <typename F>
    bool SensorFilter<F>::IsValid() const
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        return IsValidInternal();
    }

    template <typename F>
    bool SensorFilter<F>::IsValidInternal() const
    {
        return m_initialized && m_filter != nullptr;
    }

    template<typename F>
    void SensorFilter<F>::PredictTo(const SensorSample::Timestamp& timestamp)
    {
        std::unique_lock<std::shared_mutex> guard(m_mutex);
        PredictInternal(timestamp);
    }

    template <typename F>
    SensorSample::Timestamp SensorFilter<F>::GetLastProcessedTimestamp() const
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        return GetLastProcessedTimestampInternal();
    }

    template <typename F>
    SensorSample::Timestamp SensorFilter<F>::GetLastProcessedTimestampInternal() const
    {
        assert((!IsValidInternal() || (ConvertToSensorTime(m_lastProcessedTimestamp) == m_filter->curr_t)) && "Filter time doesn't match bookkeeping");
        return m_lastProcessedTimestamp;
    }

    template <typename F>
    bool SensorFilter<F>::HasGoodGravity() const
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        return m_filter->has_good_gravity();
    }

    template <typename F>
    std::unique_ptr<F> SensorFilter<F>::ExtractFilter()
    {
        std::unique_lock<std::shared_mutex> guard(m_mutex);
        assert(IsValidInternal() && "requesting invalid filter");
        m_initialized = false;
        assert(ConvertToSensorTime(m_lastProcessedTimestamp) == m_filter->curr_t);
        return std::move(m_filter);
    }

    //// 3dof -----------------
    std::array<float, 4 * 4> SensorFilter3Dof::GetFilteredCameraToWorldIMU(std::array<double, 6 * 6>& cameraToWorldPoseCovariance)
    {
        if (m_initialized == false)
        {
            assert(false && "filter not initialized yet");
            return{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
        }

        ST::Matrix<6, 6, double> worldToCamCovariance;
        ST::SO3<double> worldToICameraSO3;
        {
            std::shared_lock<std::shared_mutex> guard(m_mutex);

            worldToICameraSO3 = ST::SO3_copy<double>(m_filter->x);

            ST::set_identity(worldToCamCovariance, m_filter->params.Px0);
            worldToCamCovariance.slice<3, 3, 3, 3>() = m_filter->Pxx;
        }
        ST::SO3<double> camToWorldSO3 = ST::SO3_inverse<double>(worldToICameraSO3);
        cameraToWorldPoseCovariance = GetCovarianceForInverse(AnalogSO3ToArray3x3(camToWorldSO3), AnalogMat6x6ToArray(worldToCamCovariance));

        return AnalogSO3ToArray4x4(camToWorldSO3);
    }

    void SensorFilter3Dof::AddVisualRotationUpdate(const std::array<float, 3 * 3>& matWorldIMUToCameraFromMage, const SensorSample::Timestamp& imageTime, const std::array<double, 3 * 3> poseCovariance)
    {
        float innov_sq{};
        ST::SO3<double>  visualX = Array3x3ToAnalogSO3(matWorldIMUToCameraFromMage);
        ST::Matrix<3, 3, double> covarianceX = Array3x3ToAnalogMat(poseCovariance);

        {
            std::unique_lock<std::shared_mutex> guard(m_mutex);
            assert(IsValidInternal() && "Valid filter");
            assert(GetLastProcessedTimestampInternal() == imageTime && "filter not run up to expected time");
            UNUSED(imageTime);
            m_filter->observe_state_x(visualX, covarianceX, innov_sq);
        }
    }

    void SensorFilter3Dof::AddVisualRotationUpdate(const std::array<float, 3 * 3>& matWorldIMUToCameraFromMage, const SensorSample::Timestamp& imageTime, double visualStdDev)
    {
        float innov_sq{};
        ST::SO3<double>  visualX = Array3x3ToAnalogSO3(matWorldIMUToCameraFromMage);
        {
            std::unique_lock<std::shared_mutex> guard(m_mutex);
            assert(IsValidInternal() && "Valid filter");
            assert(GetLastProcessedTimestampInternal() == imageTime && "filter not run up to expected time");
            UNUSED(imageTime);

            m_filter->observe_state_x(visualX, visualStdDev*visualStdDev, innov_sq);
        }
    }

    std::array<double, 6 * 6> SensorFilter3Dof::GetCovarianceForInverse(const std::array<float, 3 * 3>& invertedArray, const std::array<double, 6 * 6>& covarianceUninverted)
    {
        ST::Matrix <3, 3, double > invertedMat = Array3x3ToAnalogMat(invertedArray);
        ST::Matrix < 6, 6, double > fullCovarianceUninvertedMat = Array6x6ToAnalogMat(covarianceUninverted);

        ST::Matrix< 6, 6, double> covarianceInverted = fullCovarianceUninvertedMat;
        covarianceInverted.slice<3, 3, 3, 3>() = invertedMat *  fullCovarianceUninvertedMat.slice<3, 3, 3, 3>() * invertedMat.T();

        return AnalogMat6x6ToArray(covarianceInverted);
    }

    //// 6dof -----------------

    //transforms 6dof to 3dof. Does not return a valid ptr to the 3dof filter
    SensorFilter6Dof::SensorFilter6Dof(std::unique_ptr<SensorFilter3Dof> filter3Dof)
        : SensorFilter<ST::IMUFilterCAS>(filter3Dof->GetIMUSettings())
    {
        std::unique_ptr<ST::VFTFilter> internal3DofFilter = filter3Dof->ExtractFilter();

        ST::SE3<double> basePose;
        ST::SE3_identity(basePose);
        ST::InitializeVBTFromVFT(*m_filter, *internal3DofFilter, basePose, *m_config);

        ResetInternal(filter3Dof->GetLastProcessedTimestamp());
    }

    void SensorFilter6Dof::OnResetInternal(const SensorSample::Timestamp & resetTime)
    {
        m_deltaPoseResetTime = resetTime;
    }

    void SensorFilter6Dof::ResetDeltaPoseIntegration(const SensorSample::Timestamp& resetTime)
    {
        //base reset
        std::unique_lock<std::shared_mutex> guard(m_mutex);
        assert(IsValidInternal() && "Valid filter");
        m_filter->reset_delta_pose_integration();
        m_deltaPoseResetTime = resetTime;
    }

    void SensorFilter6Dof::ResetPoseTranslation()
    {
        std::unique_lock<std::shared_mutex> guard(m_mutex);
        assert(IsValidInternal() && "expecting valid filter for position reset");

        // save original pose and covariance
        ST::SE3< ST::fuser_scalar_t> worldToCamSE3;
        ST::SE3_copy< ST::fuser_scalar_t>(worldToCamSE3, m_filter->x);
        ST::Matrix<6, 6, ST::fuser_scalar_t> PxxCopy = m_filter->Pxx;

        //set the translation on pose to zero
        worldToCamSE3.t[0] = 0.0;   worldToCamSE3.t[1] = 0.0;   worldToCamSE3.t[2] = 0.0;

        //set the uncertainty on translation to something large
        const ST::fuser_scalar_t poseTranslationSigma = 1.0; // 1 meter uncertainty

        //TODO: this does not affect partials in pxx, may want to add in the future
        //TODO: this should maybe update all the other blocks that are px related eg: pxa, may want to add in the fuser
        ST::set_identity(PxxCopy.slice<0, 0, 3, 3>().ref(), static_cast<ST::fuser_scalar_t>(poseTranslationSigma * poseTranslationSigma));

        //reset the pose to this value
        m_filter->reset_pose_only(m_filter->curr_t, worldToCamSE3, PxxCopy);
    }

    bool SensorFilter6Dof::HasGoodScaleInternal() const
    {
        if (!IsValidInternal())
            return false;

        if (m_filter->k[0] < SCALE_EPSILON<std::remove_reference_t<decltype(m_filter->k[0])>>)
            return false;

        //recommended by analog for a 'good' scale value
        return (sqrt(m_filter->Pkk(0, 0)) / abs(m_filter->k[0])) < GoodScaleCovToScaleRatio;
    }

    bool SensorFilter6Dof::HasGoodScale() const
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        return HasGoodScaleInternal();
    }

    // returns METERS = MAGE * scale
    bool SensorFilter6Dof::GetEstimatedVisualToMetersScale(float& visualScaleToMeters) const
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        if (!IsValidInternal())
        {
            assert(false && "filter not initialized yet");
            visualScaleToMeters = 0.0f;
            return false;
        }

        const auto metersToVisualScale = static_cast<float>(m_filter->k[0]);
        if (fabs(metersToVisualScale) < SCALE_EPSILON<float>)
        {
            visualScaleToMeters = 0.0f;
            return false;
        }

        visualScaleToMeters = 1.0f / metersToVisualScale;
        return HasGoodScaleInternal();
    }

    std::array<float, 4 * 4>  SensorFilter6Dof::GetFilteredCameraToWorldIMU(std::array<double, 6 * 6>& cameraToWorldPoseCovariance)
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        if (!IsValidInternal())
        {
            assert(false && "filter not initialized yet");
            return{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
        }

        // world to camera
        ST::SE3<double> worldToCamSE3;
        ST::SE3_copy<double>(worldToCamSE3, m_filter->x);
        ST::Matrix<6, 6, ST::fuser_scalar_t> worldToCamCovariance = m_filter->Pxx;

        // camera to world
        ST::SE3<double> camToWorldSE3 = ST::SE3_inverse<double>(worldToCamSE3);
        cameraToWorldPoseCovariance = GetCovarianceForInverse(AnalogSE3ToMat4x4(camToWorldSE3), AnalogMat6x6ToArray(worldToCamCovariance));

        // filter->k holds the meters to mage scaling value
        camToWorldSE3.t[0] *= m_filter->k[0];
        camToWorldSE3.t[1] *= m_filter->k[0];
        camToWorldSE3.t[2] *= m_filter->k[0];

        //scale covariance
        // covariance of state x = x*transpose of x
        // believe this means the translational part (upper 3x3 should be multiplied by scale*scale
        double scaleSquared = m_filter->k[0] * m_filter->k[0];
        for (int idxRow = 0; idxRow < 3; idxRow++)
        {
            for (int idxCol = 0; idxCol < 3; idxCol++)
            {
                cameraToWorldPoseCovariance[idxRow * 6 + idxCol] *= scaleSquared;
            }
        }

        return AnalogSE3ToMat4x4(camToWorldSE3);
    }

    std::array<float, 4 * 4> SensorFilter6Dof::GetDeltaPose(const SensorSample::Timestamp& fromTime) const
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        if (!IsValidInternal())
        {
            assert(false && "filter not initialized yet");
            return{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
        }

        assert(GetLastProcessedTimestampInternal() == fromTime && "filter not run up to expected time");
        UNUSED(fromTime);

        ST::SE3<double> worldToCamSE3;
        ST::SE3_copy<double>(worldToCamSE3, m_filter->d);

        // filter->k holds the meters to mage scaling value
        worldToCamSE3.t[0] *= m_filter->k[0];
        worldToCamSE3.t[1] *= m_filter->k[0];
        worldToCamSE3.t[2] *= m_filter->k[0];

        return AnalogSE3ToMat4x4(worldToCamSE3);
    }

    bool SensorFilter6Dof::AddVisualPoseDeltaUpdate(const std::array<float, 4 * 4>& matIMU0ToIMU1FromMage, const SensorSample::Timestamp& image1Time, const std::array<double, 6 * 6>& deltaPoseCovariance)
    {
        ST::Matrix < 6, 6, double > covariance = Array6x6ToAnalogMat(deltaPoseCovariance);

        float innov_sq{};
        ST::SE3<double>  visuald = Array4x4ToAnalogSE3(matIMU0ToIMU1FromMage);
        {
            std::unique_lock<std::shared_mutex> guard(m_mutex);
            assert(IsValidInternal() && "Valid filter");

            //frame is the same as the reset time, skip update
            assert(m_deltaPoseResetTime < image1Time && "filter ran ahead");

            assert(GetLastProcessedTimestampInternal() == image1Time && "filter not run up to expected time");
            m_deltaPoseResetTime = image1Time;
            return m_filter->update_with_scaled_delta_pose(visuald, covariance, innov_sq, -1.0f);
        }
    }

    bool SensorFilter6Dof::AddVisualPoseDeltaUpdate(const std::array<float, 4 * 4>& matIMU0ToIMU1FromMage, const SensorSample::Timestamp& image1Time, double visualStdDev)
    {
        const double sig_sq = visualStdDev*visualStdDev;
        std::array<double, 6 * 6> deltaPoseCovariance{};
        deltaPoseCovariance[0 * 6 + 0] = sig_sq;
        deltaPoseCovariance[1 * 6 + 1] = sig_sq;
        deltaPoseCovariance[2 * 6 + 2] = sig_sq;
        deltaPoseCovariance[3 * 6 + 3] = sig_sq;
        deltaPoseCovariance[4 * 6 + 4] = sig_sq;
        deltaPoseCovariance[5 * 6 + 5] = sig_sq;

        return AddVisualPoseDeltaUpdate(matIMU0ToIMU1FromMage, image1Time, deltaPoseCovariance);
    }

    void SensorFilter6Dof::SwitchFilterOrigin(const std::array<float, 4 * 4>& matNewToOld, const std::array<double, 6 * 6>& poseCovariancInOldOrigin)
    {
        ST::SE3<double> newToOld = Array4x4ToAnalogSE3(matNewToOld);
        ST::Matrix<6, 6, double> poseCovNewInOldOrigin(poseCovariancInOldOrigin.data());
        {
            std::unique_lock<std::shared_mutex> guard(m_mutex);
            m_filter->transform_frame(newToOld, poseCovNewInOldOrigin);
        }
    }

    std::array<double, 6 * 6> SensorFilter6Dof::GetCovarianceForInverse(const std::array<float, 4 * 4>& invertedArray, const std::array<double, 6 * 6>& covarianceUninverted)
    {
        ST::SE3<double> invertedMat = Array4x4ToAnalogSE3(invertedArray);
        ST::Matrix < 6, 6, double > covarianceUninvertedMat = Array6x6ToAnalogMat(covarianceUninverted);

        //Cov(inv(A)) = Adj(inv(A)) * cov(A) * transpose(Adj(inv(A)))
        ST::Matrix<6, 6, double> adjointInvA;
        ST::SE3_adjoint(adjointInvA.data(), invertedMat);

        ST::Matrix< 6, 6, double> covarianceInverted = adjointInvA * covarianceUninvertedMat * adjointInvA.T();

        return AnalogMat6x6ToArray(covarianceInverted);
    }

    // AB = A * B
    // cov(A * B) = cov(A) + adj(A) * cov(B) * transpose(adj(A))
    std::array<double, 6 * 6> SensorFilter6Dof::GetCovarianceForMatMul(const std::array<float, 4 * 4>& matA, const std::array<double, 6 * 6>& covA, const std::array<double, 6 * 6>& covB)
    {
        ST::SE3<double> se3A = Array4x4ToAnalogSE3(matA);
        ST::Matrix < 6, 6, double > matCovA = Array6x6ToAnalogMat(covA);
        ST::Matrix < 6, 6, double > matCovB = Array6x6ToAnalogMat(covB);

        ST::Matrix<6, 6, double> adjointA;
        ST::SE3_adjoint(adjointA.data(), se3A);

        ST::Matrix < 6, 6, double > matCovAB = matCovA + adjointA * matCovB * adjointA.T();
        return AnalogMat6x6ToArray(matCovAB);
    }

    // expects metric world positions
    void SensorFilter6Dof::SetPoseWorldPosition(float xWorldPos, float yWorldPos, float zWorldPos, const std::array<double, 3 * 3>& covTranslation)
    {
        {
            //the x.t in the fuser is really a world to rig translation (what does origin look like from this position, so it is negated)
            std::unique_lock<std::shared_mutex> guard(m_mutex);

            ST::SE3<double> worldToCamSE3;
            ST::SE3_copy<double>(worldToCamSE3, m_filter->x);
            ST::SE3<double> camToWorldSE3 = ST::SE3_inverse<double>(worldToCamSE3);

            camToWorldSE3.t[0] = xWorldPos;
            camToWorldSE3.t[1] = yWorldPos;
            camToWorldSE3.t[2] = zWorldPos;

            worldToCamSE3 = ST::SE3_inverse<double>(camToWorldSE3);

            ST::SE3_copy<double>(m_filter->x, worldToCamSE3);

            // TODO: this does not affect partials in pxx, may want to add in the future
            //TODO: this should maybe update all the other blocks that are px related eg: pxa, may want to add in the fuser
            ST::Matrix<3, 3, double> matCov = Array3x3ToAnalogMat(covTranslation);
            m_filter->Pxx.set_slice<0, 0, 3, 3>(matCov);
        }
    }

    //Simple6dof
    std::array<float, 4 * 4>  SensorFilterSimple6Dof::GetFilteredCameraToWorldIMU(std::array<double, 6 * 6>&)
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        if (!IsValidInternal())
        {
            assert(false && "filter not initialized yet");
            return{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
        }

        // world to camera
        ST::SE3<double> worldToCamSE3;
        ST::SE3_copy<double>(worldToCamSE3, m_filter->x);

        /*double sq = sqrtf((float)m_filter->x.R[0] * (float)m_filter->x.R[0] + (float)m_filter->x.R[3] * (float)m_filter->x.R[3]);
        double xRad = atan2(m_filter->x.R[7], m_filter->x.R[8]);
        double yRad = atan2(-m_filter->x.R[6], sq);
        double zRad = atan2(m_filter->x.R[3], m_filter->x.R[0]);

        LogMessage<>(L"m_filter->x " + std::to_wstring(m_filter->x.t[0]) + L"  " + std::to_wstring(m_filter->x.t[1]) + L"  " + std::to_wstring(m_filter->x.t[2]));
        LogMessage<>(L"x angle " + std::to_wstring(xRad) + L"  " + std::to_wstring(yRad) + L"  " + std::to_wstring(zRad));*/

        // camera to world
        ST::SE3<double> camToWorldSE3 = ST::SE3_inverse<double>(worldToCamSE3);
        //mira::LogMessage<Tracing::TraceLevels::Information, 42>(L"Gravity " + std::to_wstring(m_filter->g[0]) + L"  " + std::to_wstring(m_filter->g[1]) + L"  " + std::to_wstring(m_filter->g[2]));
        //mira::LogMessage<Tracing::TraceLevels::Information, 42>(L"acc " + std::to_wstring(m_filter->a[0]) + L"  " + std::to_wstring(m_filter->a[1]) + L"  " + std::to_wstring(m_filter->a[2]));
        //mira::LogMessage<Tracing::TraceLevels::Information, 42>(L"gyro " + std::to_wstring(m_filter->w[0]) + L"  " + std::to_wstring(m_filter->w[1]) + L"  " + std::to_wstring(m_filter->w[2]));
        //mira::LogMessage<Tracing::TraceLevels::Information, 42>(L"translation " + std::to_wstring(camToWorldSE3.t[0]) + L"  " + std::to_wstring(camToWorldSE3.t[1]) + L"  " + std::to_wstring(camToWorldSE3.t[2]));
        return AnalogSE3ToMat4x4(camToWorldSE3);
    }

    std::array<float, 4 * 4> SensorFilterSimple6Dof::GetDeltaPose(const SensorSample::Timestamp& fromTime) const
    {
        std::shared_lock<std::shared_mutex> guard(m_mutex);
        if (!IsValidInternal())
        {
            assert(false && "filter not initialized yet");
            return{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
        }

        assert(GetLastProcessedTimestampInternal() == fromTime && "filter not run up to expected time");
        UNUSED(fromTime);

        ST::SE3<double> worldToCamSE3;
        ST::SE3_copy<double>(worldToCamSE3, m_filter->d);

        // filter->k holds the meters to mage scaling value
        worldToCamSE3.t[0] *= m_filter->k[0];
        worldToCamSE3.t[1] *= m_filter->k[0];
        worldToCamSE3.t[2] *= m_filter->k[0];

        return AnalogSE3ToMat4x4(worldToCamSE3);
    }

    void SensorFilterSimple6Dof::ResetDeltaPoseIntegration(const SensorSample::Timestamp& resetTime)
    {
        //base reset
        std::unique_lock<std::shared_mutex> guard(m_mutex);
        assert(IsValidInternal() && "Valid filter");
        m_filter->reset_delta_pose_integration();
        m_deltaPoseResetTime = resetTime;
    }

    void SensorFilterSimple6Dof::SimpleSwitchFilterOrigin(const std::array<float, 4 * 4>& curCameraToMageWorld, const SensorSample::Timestamp& resetTime)
    {
        ST::SE3<double> curToMageWorld = Array4x4ToAnalogSE3(curCameraToMageWorld);
        {
            std::unique_lock<std::shared_mutex> guard(m_mutex);
            m_filter->rotate_frame_reset(curToMageWorld, ConvertToSensorTime(resetTime));
        }

        ResetInternal(resetTime);
    }

    void SensorFilterSimple6Dof::OnResetInternal(const SensorSample::Timestamp& resetTime)
    {
        m_deltaPoseResetTime = resetTime;
    }

    template class SensorFilter<ST::VFTFilter>;
    template class SensorFilter<ST::IMUFilterCAS>;
    template class SensorFilter<ST::SimpleIMUFilter>;
}
