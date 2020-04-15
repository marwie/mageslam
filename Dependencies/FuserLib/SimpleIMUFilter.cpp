
#include "pch.h"
#include "SimpleIMUFilter.h"

#include <BaseLib/Debug.h>
#include "MathLib/LDLT.h"
#include "Utils/Constants.h"
#include "Plots.h"

namespace
{
    template <typename S>
    S exponential_damping(S damping_per_sec, S dt)
    {
        return damping_per_sec > 0 ? ST::exp(dt * ST::ln(damping_per_sec)) : (S)0;
    }
}

namespace ST
{
    bool SimpleIMUFilter::update_with_gyro_simple(_In_reads_(3) const float z[3])
    {
        w = -Vector<3, T>(asVectorRef<3>(z));

        return true;
    }

    bool SimpleIMUFilter::update_with_accel_simple(_In_reads_(3) const float z[3])
    {

        if (hasIMUGravity == false)
        {
            g = -asVectorRef<3>(z);
        }
        else
        {

            const Matrix<3, 3, T> R = Matrix<3, 3, T>(x.R);
            const Vector<3, T>Rg = R * g;

            a = -Vector<3, T>(asVectorRef<3>(z)) - (u ^ w) - Rg;
        }

        return true;
    }

    void SimpleIMUFilter::predict(SensorTime::time_t t)
    {
        if (t == curr_t)
        {
            return;
        }

        DEBUG_ONLY(ST_STATIC_CONST_CONFIG_PARAM(int, maxBackwardsMillisec, "SimpleIMUFilter.max_backwards_predict_milliseconds", 2););
        ASSERT(curr_t != SensorTime::invalidTime());
        ASSERT(t > curr_t || curr_t - t <= SensorTime::fromMicroseconds(maxBackwardsMillisec * 1000));

        const T dt = (T)SensorTime::toSeconds(t - curr_t);
        const T dt2 = dt * dt;
        const T half_dt2 = (T)0.5 * dt2;

        const T a_gain = exponential_damping(params.alpha_a, dt);

        Vector<6, T> avg_v = velocity() + (T) 0.5 * dt * acceleration();
        SE3<T> motion;
        SE3_exp(motion, (dt*avg_v).data());

        x = motion * x;
        SE3_coerce_rotation(x);
        d = motion * d;
        SE3_coerce_rotation(d);
        u += dt * a;
        w += dt * r;
        a *= a_gain;

        // Apply exponential decay for gyro sensitivity estimation
        ST_STATIC_CONST_CONFIG_PARAM(float, gyro_sf_upperlimit, "SimpleIMUFilter.gyro_sf_upperlimit", 0.02f);

        Vector<3, T> gyro_sf_gain;

        for (int i = 0; i < 3; i++)
        {
            gyro_sf_gain[i] = abs(gyro_sf[i]) < static_cast<T>(gyro_sf_upperlimit) ? exponential_damping(params.alpha1_gsf, dt) : exponential_damping(params.alpha2_gsf, dt);
            gyro_sf[i] *= gyro_sf_gain[i];
        }

        curr_t = t;
    }

    FuserUpdateStatusType SimpleIMUFilter::predict_and_update(const IMUFilterEvent& imuFilterEvent, const IMUConfig &imu_config)
    {
        FuserUpdateStatusType updateStatus = NO_UPDATE_ERROR;

        const char* const pPlotNames[4] =
        {
            "SimpleIMUFilter.Jerk_a",
            "SimpleIMUFilter.Jerk_r",
            "SimpleIMUFilter.SqrtQa",
            "SimpleIMUFilter.SqrtQr"
        };

        ST_STATIC_CONST_CONFIG_PARAM(int, plotProcNoise, "plot_proc_noise.SimpleIMUFilter", 0);
        m_procNoiseEstimator.Update(curr_t, a, r, (plotProcNoise != 0 ? pPlotNames : nullptr));

        predict(imuFilterEvent.m_timestamp);
        ST_STATIC_CONST_CONFIG_PARAM(int, disable_accelerometer, "disable_accelerometer", 0);
        ST_STATIC_CONST_CONFIG_PARAM(int, disable_gyro, "disable_gyro", 0);
        ST_STATIC_CONST_CONFIG_PARAM(int, use_mag, "VT.use_mag", 1);
        ST_STATIC_CONST_CONFIG_PARAM(int, plotInnov, "plot_imu_innov", 0);

        if (!disable_gyro && imuFilterEvent.m_pGyro != nullptr)
        {
            float innov_sq = 0.f;
            bool gyroUpdateSuccess = update_with_gyro_simple(imuFilterEvent.m_pGyro);

            if (!gyroUpdateSuccess)
            {
                updateStatus |= GYRO_UPDATE_ERROR;
            }

            if (plotInnov && gyroUpdateSuccess)
            {
                plots_add_sample("VT.GyroInnov", imuFilterEvent.m_timestamp, sqrtf(innov_sq));
            }
        }

        if (!disable_accelerometer && imuFilterEvent.m_pAccel != nullptr)
        {
            float innov_sq = 0.f;
            bool accelUpdateSuccess = update_with_accel_simple(imuFilterEvent.m_pAccel);

            if (!accelUpdateSuccess)
            {
                updateStatus |= ACCEL_UPDATE_ERROR;
            }

            if (plotInnov && accelUpdateSuccess)
            {
                plots_add_sample("VT.AccelInnov", imuFilterEvent.m_timestamp, sqrtf(innov_sq));
            }
        }

        if (use_mag && imuFilterEvent.m_pMag != nullptr)
        {
            float innov_sq = 0.f;
            bool magUpdateSuccess = update_with_mag(imuFilterEvent.m_pMag, imu_config.MagNoiseSigmaSq, innov_sq);

            if (!magUpdateSuccess)
            {
                updateStatus |= MAG_UPDATE_ERROR;
            }

            if (plotInnov && magUpdateSuccess)
            {
                plots_add_sample("VT.MagInnov", imuFilterEvent.m_timestamp, sqrtf(innov_sq));
            }
        }

        ST_STATIC_CONST_CONFIG_PARAM(int, zero_accel_hack, "zero_accel_hack", 0);
        if (zero_accel_hack)
        {
            ST_STATIC_CONST_CONFIG_PARAM(float, thresh_sigma, "zero_accel_hack.sigma_thresh", 0.f);
            if (trace(Paa.slice<0, 0, 3, 3>()) > thresh_sigma * thresh_sigma)
            {
                ST_STATIC_CONST_CONFIG_PARAM(float, zero_accel_sigma, "zero_accel_hack.sigma", 1.f);
                float innov_sq = 0.f;
                bool observeStateaSuccess = observe_state_a(Vector<3, T>(0.0), static_cast<T>(zero_accel_sigma * zero_accel_sigma), innov_sq);

                if (!observeStateaSuccess)
                {
                    // if update was not successful due to indefinite innovation covariance, reset the related state and covariances
                    StateMask stateMask;
                    stateMask.a = true;
                    reset_states(stateMask);
                    updateStatus |= OBSERVE_STATE_a_ERROR;
                }
            }
        }

        ST_STATIC_CONST_CONFIG_PARAM(int, constrain_gravity, "constrain_gravity_mag", 1);
        if (constrain_gravity)
        {
            ST_STATIC_CONST_CONFIG_PARAM(float, constrain_gravity_sigma, "constrain_gravity_mag.sigma", 1e0f);
            ST_STATIC_CONST_CONFIG_PARAM(float, gravity_mag, "gravity_mag", g_GravityMetersPerSecPerSec);

            const float sigma_sq = constrain_gravity_sigma * constrain_gravity_sigma;
            float innov_sq = 0.f;
            bool gravityMagUpdateSuccess = update_with_gravity_mag(gravity_mag, sigma_sq, innov_sq);

            if (!gravityMagUpdateSuccess)
            {
                updateStatus |= GRAVMAG_UPDATE_ERROR;
            }
        }

        return updateStatus;

    }

    void SimpleIMUFilter::rotate_frame_reset(_In_ const SE3<T>& rotateToMage, SensorTime::time_t t)
    {
        const Matrix<3, 3, T> R(rotateToMage.R);

        g = R * g;
        normalize(g);
        g = 9.81 * g;

        StateMask stateMask = true;
        stateMask.g = false;
        reset_states(stateMask);

        SE3_copy(x, rotateToMage);
        SE3_invert(x);

        hasIMUGravity = true;

        curr_t = t;
    }

    bool SimpleIMUFilter::has_good_gravity() const
    {
        return hasIMUGravity;
    }
}
