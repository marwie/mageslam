#include "AnalogCAIMU.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Utils/cv.h"
#include <boost/optional.hpp>

#include <arcana/containers/sorted_vector.h>
#include <arcana/analysis/object_trace.h>
#include <arcana/analysis/data_point.h>

#include <IMUFilterCA.h>

#include "Utils/Logging.h"

#include "FuserFunctions.h"
#include "ImuMath.h"

namespace E = Eigen;

namespace mage
{
    using seconds = std::chrono::duration<double>;

    struct alignas(16) AnalogCAIMU::Impl
    {
        ST::SE3<double> m_bodyIMUToBodyCamera = ST::SE3_identity<double>();
        ST::SE3<double> m_bodyCameraToBodyIMU = ST::SE3_identity<double>();

        mira::sorted_vector<mage::SensorSample, mage::SensorSample::Compare> m_samples;

        mage::SensorSample::Timestamp m_t0{};

        static constexpr double SIGMA_TRANSLATION = 3e-3;
        static constexpr double SIGMA_ROTATION = 3e-3;

        ST::Matrix<6, 6, double> m_sigmaPose{ 0.0 };

        std::array<float, 3> m_previousAccel{ 0, 0, 0 };

        ST::IMUFilterCA m_filter{};
        ST::IMUConfig m_config{};

        Impl(const device::IMUCharacterization& imuCharacterization)
            : m_config{ BuildIMUConfigFromCharacterization(imuCharacterization) }
        {
            for (int i = 0; i < 3; ++i)
            {
                m_sigmaPose(i, i) = SIGMA_TRANSLATION * SIGMA_TRANSLATION;
                m_sigmaPose(i + 3, i + 3) = SIGMA_ROTATION * SIGMA_ROTATION;
            }
        }

        mage::Pose GetPose() const
        {
            return imu::ViewSE3ToPose(m_bodyIMUToBodyCamera * m_filter.pose());
        }

        void PredictUpTo(mage::SensorSample::Timestamp timestamp)
        {
            if (m_t0 == mage::SensorSample::Timestamp{})
                return;

            auto from = m_samples.begin();
            auto to = std::find_if(from, m_samples.end(), [timestamp](const mage::SensorSample& sample)
            {
                return sample.GetTimestamp() > timestamp;
            });

            for (auto gyro = from; gyro != to; ++gyro)
            {
                if (gyro->GetTimestamp() < m_t0)
                    continue;

                if (gyro->GetType() == mage::SensorSample::SampleType::Gyrometer)
                {
                    E::Map<imu::Vector<float>> accelData{ m_previousAccel.data(), 3 };

                    const auto* accelSample = mage::FindCorrespondingAccelSample(*gyro, m_samples);
                    if (accelSample != nullptr)
                    {
                        accelData = E::Map<const imu::Vector<float>>{ accelSample->GetData().data(), 3 };
                    }

                    ST::IMUEvent ev{};
                    ev.timestamp = ToAnalogSensorTime(gyro->GetTimestamp());
                    ev.magnetometer_timestamp = ST::SensorTime::invalidTime();

                    E::Map<imu::Vector<float>>{ev.rotvel} = E::Map<const imu::Vector<float>>(gyro->GetData().data(), 3);
                    E::Map<imu::Vector<float>>{ev.accel} = accelData;

                    ST::IMUFilterEvent filterEv{ ev };

                    auto result = m_filter.predict_and_update(filterEv, m_config);
                    if (result != ST::FuserUpdateStatus::NO_UPDATE_ERROR)
                    {
                        std::wstringstream ss;
                        ss << L"Failed to predict and update based on the filter event: " << std::hex << result << "\n";
                        mage::LogMessage<>(ss.str());
                    }

                    FIRE_OBJECT_TRACE("IMU Linear Velocity.AnalogIMU", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        (float)E::Vector3d{ m_filter.u[0], m_filter.u[1], m_filter.u[2] }.norm()
                    )));

                    FIRE_OBJECT_TRACE("IMU Linear Accel.AnalogIMU", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        (float)E::Vector3d{ m_filter.a[0], m_filter.a[1], m_filter.a[2] }.norm()
                    )));

                    FIRE_OBJECT_TRACE("Gravity AnalogIMU", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.g[0], m_filter.g[1], m_filter.g[2] }.cast<float>()
                    )));

                    FIRE_OBJECT_TRACE("GyroBias AnalogIMU", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.gyro_bias[0], m_filter.gyro_bias[1], m_filter.gyro_bias[2] }.cast<float>()
                    )));

                    FIRE_OBJECT_TRACE("AccelBias AnalogIMU", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.accel_bias[0], m_filter.accel_bias[1], m_filter.accel_bias[2] }.cast<float>()
                    )));
                }
            }

            m_samples.erase(from, to);

            m_filter.predict(ToAnalogSensorTime(timestamp));
        }

        void ProcessPose(mage::SensorSample::Timestamp timestamp, const mage::Pose& pose)
        {
            ST::SE3<double> visualPoseInIMU = m_bodyCameraToBodyIMU * imu::PoseToViewSE3(pose);

            if (m_t0 == mage::SensorSample::Timestamp{})
            {
                // first pose
                m_t0 = timestamp;

                m_filter.reset(ToAnalogSensorTime(timestamp), m_config);

                ST::SE3_copy(m_filter.x, visualPoseInIMU);

                mage::LogMessage<>(L"Initialized fuser pose\n");
            }
            else
            {
                m_filter.predict(ToAnalogSensorTime(timestamp));

                //compute innov
                //ST::Vector<6, double> innov;
                //
                //ST::SE3<double> pred, imuExtrinsicEstimate;
                //ST::SE3_identity(imuExtrinsicEstimate);
                //ST::asMatrixRef<3, 3>(imuExtrinsicEstimate.R) = ST::asMatrixRef<3, 3>(m_filter.C.R);
                //ST::SE3_mult(pred, imuExtrinsicEstimate, m_filter.x);
                //ST::SE3_log(innov.data(), SE3_mult_a_binv(visualPoseInIMU, pred));

                ////skip or not
                //const ST::Vector<3, double> logT = ST::asVectorRef<3>(innov.data());
                //const ST::Vector<3, double> logR = ST::asVectorRef<3>(innov.data() + 3);
                //if (params.skipOutlierPoses && (ST::L2_norm(logT) > params.fuserOutlierPoseThresholdMeters ||
                //    ST::L2_norm(logR) > params.fuserOutlierPoseThresholdRadians))
                //{
                //    skipped_pose_timestamps_out << ST::SensorTime::toSeconds(ps.timestamp);
                //    skipPoseCount++;
                //    continue;
                //}

                //update
                float innov_sq = 0.f;
                if (!m_filter.update_with_pose(visualPoseInIMU, m_sigmaPose, innov_sq, -1.f))
                {
                    mage::LogMessage<>(L"Update with pose failed\n");
                }
            }
        }
    };

    AnalogCAIMU::AnalogCAIMU(const device::IMUCharacterization& imuCharacterization)
        : m_impl{ std::allocate_shared<Impl, Eigen::aligned_allocator<Impl>>({}, imuCharacterization) }
    {
        m_impl->m_bodyCameraToBodyIMU = imu::ToSE3(E::Map<const imu::Matrix<float>>(imuCharacterization.BodyCameraToBodyIMU.data(), 4, 4));
        m_impl->m_bodyIMUToBodyCamera = ST::SE3_inverse(m_impl->m_bodyCameraToBodyIMU);
    }

    AnalogCAIMU::~AnalogCAIMU() = default;

    void AnalogCAIMU::AddSample(const mage::SensorSample& sample)
    {
        if (sample.GetType() == mage::SensorSample::SampleType::Gyrometer)
        {
            m_impl->m_samples.insert(sample);
        }
        else if (sample.GetType() == mage::SensorSample::SampleType::Accelerometer)
        {
            // based on a conversation with Salim, the analog fuser has the accelerometer
            // values negated compared to the windows api.
            auto samples = sample.GetData();
            std::transform(samples.begin(), samples.end(), samples.begin(), std::negate<float>{});

            m_impl->m_samples.insert(mage::SensorSample{ sample.GetType(), sample.GetTimestamp(), std::move(samples) });
        }
    }

    mage::Pose AnalogCAIMU::GetPose() const
    {
        return m_impl->GetPose();
    }

    void AnalogCAIMU::PredictUpTo(mage::SensorSample::Timestamp timestamp)
    {
        m_impl->PredictUpTo(timestamp);
    }

    void AnalogCAIMU::AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp)
    {
        m_impl->ProcessPose(timestamp, pose);
    }

    float AnalogCAIMU::GetLinearAccelerationNorm() const
    {
        E::Vector3d accel{ m_impl->m_filter.a[0], m_impl->m_filter.a[1], m_impl->m_filter.a[2] };
        return (float)accel.norm();
    }

    float AnalogCAIMU::GetLinearVelocityNorm() const
    {
        E::Vector3d velo{ m_impl->m_filter.u[0], m_impl->m_filter.u[1], m_impl->m_filter.u[2] };
        return (float)velo.norm();
    }
}
