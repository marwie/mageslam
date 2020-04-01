#include "AnalogCASIMU.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Utils/cv.h"
#include <boost/optional.hpp>

#include <arcana/containers/sorted_vector.h>
#include <arcana/analysis/object_trace.h>
#include <arcana/analysis/data_point.h>

#include <IMUFilterCAS.h>
#include <VFTFilter.h>
#include <FilterInitializationScaled.h>

#include "AnalogVFTFilter.h"

#include "Utils/Logging.h"

#include "FuserFunctions.h"
#include "ImuMath.h"

namespace E = Eigen;

namespace mage
{
    using seconds = std::chrono::duration<double>;

    struct /*alignas(16)*/ AnalogCASIMU::Impl
    {
        // Due to precision issues (1 visual pose unit is about 4-5 cm)
        // we need to start by scaling the visual poses so the fuser doesn't
        // have to solve for a miniscule visual pose to meters scale.
        // This seems to help the fuser converge on a valid answer.
        static constexpr double VISUAL_BASE_SCALE = 1.0;

        ST::SE3<double> m_bodyIMUToBodyCamera = ST::SE3_identity<double>();
        ST::SE3<double> m_bodyCameraToBodyIMU = ST::SE3_identity<double>();

        mira::sorted_vector<mage::SensorSample, mage::SensorSample::Compare> m_samples;

        mage::SensorSample::Timestamp m_t0{};

        ST::SE3<double> m_previousVisualPose = ST::SE3_identity<double>();

        static constexpr double SIGMA_TRANSLATION = 3e-3;
        static constexpr double SIGMA_ROTATION = 3e-3;

        ST::Matrix<6, 6, double> m_sigmaPose{ 0.0 };

        std::array<float, 3> m_previousAccel{};

        ST::IMUFilterCAS m_filter{};
        ST::IMUConfig m_config{};

        bool m_trackingLost = false;

        Impl(const device::IMUCharacterization& imuCharacterization, bool assumeSamePosition)
            : m_config{ BuildIMUConfigFromCharacterization(imuCharacterization) }
        {
            for (int i = 0; i < 3; ++i)
            {
                m_sigmaPose(i, i) = SIGMA_TRANSLATION * SIGMA_TRANSLATION;
                m_sigmaPose(i + 3, i + 3) = SIGMA_ROTATION * SIGMA_ROTATION;
            }

            m_bodyCameraToBodyIMU = imu::ToSE3(E::Map<const imu::Matrix<float>>(imuCharacterization.BodyCameraToBodyIMU.data(), 4, 4));

            if (assumeSamePosition)
            {
                Eigen::Map<imu::Vector<double>>{m_bodyCameraToBodyIMU.t} *= 0;
            }

            if (m_filter.m_ApplyIMUCameraExtrinsics)
            {
                // Already has been rotated in the filter
                ST::SO3<double> imuRotation = ST::SO3_identity<double>();
                ST::asMatrixRef<3, 3>(m_bodyIMUToBodyCamera.R) = ST::asMatrixRef<3, 3>(imuRotation.R);
            }

            m_bodyIMUToBodyCamera = ST::SE3_inverse(m_bodyCameraToBodyIMU);
        }

        Impl(const AnalogVFTFilter& vftFilter, const mage::Pose& pose, mage::SensorSample::Timestamp time, bool assumeSamePosition)
            : Impl{ vftFilter.GetCharacterization(), assumeSamePosition }
        {
            if (vftFilter.GetInternalFilter().curr_t != ToAnalogSensorTime(time))
            {
                throw std::invalid_argument("vftFilter needs to be updated to the right time.");
            }

            m_t0 = time;

            m_filter.reset(ToAnalogSensorTime(time));

            const ST::VFTFilter& vftfilter = vftFilter.GetInternalFilter();

            // initialize our filter pose to identity
            // and transform the gravity vector from the 3dof
            // world frame to the body frame.
            m_filter.x = ST::SE3_identity<double>();

            imu::Vector<double> g;
            ST::SO3_transform_direction3(g.data(), vftfilter.x, vftfilter.g.data());
            Eigen::Map<imu::Vector<double>>{ m_filter.g.data() } = g;

            // Transfer over all the existing data that is in body frame
            // as it should be the same.
            //m_filter.u = vftfilter.u; //linear velocity        (tangential)
            m_filter.w = vftfilter.w; //angular velocity       (tangential)
            m_filter.a = vftfilter.a; //linear acceleration    (tangential)
            m_filter.r = vftfilter.r; //angular acceleration   (tangential)

            m_previousVisualPose = imu::PoseToViewSE3(pose);

            auto samples = vftFilter.GetSamples();
            m_samples.insert_presorted(samples.begin(), samples.end());
        }

        mage::Pose GetPose() const
        {
            ST::SE3<double> deltaPose;
            ST::SE3_copy<double>(deltaPose, m_filter.d);
            
            // filter->k holds the meters to mage scaling value
            deltaPose.t[0] *= m_filter.k[0];
            deltaPose.t[1] *= m_filter.k[0];
            deltaPose.t[2] *= m_filter.k[0];

            ST::SE3<double> deltaCameraPose = m_bodyIMUToBodyCamera * deltaPose * m_bodyCameraToBodyIMU;

            ST::SE3<double> pose = deltaCameraPose * m_previousVisualPose;
            Eigen::Map<Eigen::Vector3d>{ pose.t } *= VISUAL_BASE_SCALE;
            return imu::ViewSE3ToPose(pose);
        }

        void PredictUpTo(mage::SensorSample::Timestamp timestamp)
        {
            if (m_t0 == mage::SensorSample::Timestamp{} || m_trackingLost)
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

                    ST::SO3<ST::fuser_scalar_t> rotationSO3 = m_filter.C;
                    ST::Vector<3, ST::fuser_scalar_t> angle;
                    ST::SO3_log(angle.data(), rotationSO3);
                    FIRE_OBJECT_TRACE("IMU to Camera Rotation AnalogSIMU", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ angle[0], angle[1], angle[3] }.cast<float>()
                        )));

                    FIRE_OBJECT_TRACE("IMU Linear Velocity.AnalogSIMU", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        (float)E::Vector3d{ m_filter.u[0], m_filter.u[1], m_filter.u[2] }.norm()
                    )));

                    FIRE_OBJECT_TRACE("IMU Linear Accel.AnalogSIMU", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        (float)E::Vector3d{ m_filter.a[0], m_filter.a[1], m_filter.a[2] }.norm()
                    )));

                    FIRE_OBJECT_TRACE("Gravity AnalogSIMU", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.g[0], m_filter.g[1], m_filter.g[2] }.cast<float>()
                    )));

                    FIRE_OBJECT_TRACE("Gravity Converged.AnalogSIMU", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        m_filter.has_good_gravity() ? 1 : 0
                    )));

                    FIRE_OBJECT_TRACE("GyroBias AnalogSIMU", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.gyro_bias[0], m_filter.gyro_bias[1], m_filter.gyro_bias[2] }.cast<float>()
                    )));

                    FIRE_OBJECT_TRACE("AccelBias AnalogSIMU", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.accel_bias[0], m_filter.accel_bias[1], m_filter.accel_bias[2] }.cast<float>()
                    )));

                    FIRE_OBJECT_TRACE("ScaleEstimation.AnalogSIMU", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        (float)m_filter.k[0]
                    )));
                }
            }

            m_samples.erase(from, to);

            m_filter.predict(ToAnalogSensorTime(timestamp));
        }

        void ProcessPose(mage::SensorSample::Timestamp timestamp, const mage::Pose& pose)
        {
            ST::SE3<double> visualPose = imu::PoseToViewSE3(pose);
            Eigen::Map<Eigen::Vector3d>{ visualPose.t } *= (1 / VISUAL_BASE_SCALE);

            if (m_t0 == mage::SensorSample::Timestamp{} || m_trackingLost)
            {
                // first pose
                m_t0 = timestamp;

                if (m_trackingLost)
                {
                    m_trackingLost = false;

                    m_filter.reset_keep_gravity_and_calibration(ToAnalogSensorTime(timestamp));
                }
                else
                {
                    m_filter.reset(ToAnalogSensorTime(timestamp), m_config);
                }

                ST::SE3_copy(m_filter.x, ST::SE3_identity<double>());

                mage::LogMessage<>(L"Initialized fuser pose\n");
            }
            else
            {
                m_filter.predict(ToAnalogSensorTime(timestamp));

                // update with delta pose
                ST::SE3<double> deltaPose = ST::SE3_mult_a_binv(visualPose, m_previousVisualPose);

                float innov_sq = 0.f;
                if (!m_filter.update_with_scaled_delta_pose(m_bodyCameraToBodyIMU * deltaPose * m_bodyIMUToBodyCamera, m_sigmaPose, innov_sq, -1))
                {
                    std::wstringstream str;
                    str << L"Update with delta pose failed: " << innov_sq << L" \n";
                    mage::LogMessage<>(str.str().c_str());
                }

                FIRE_OBJECT_TRACE("Visual Innov.AnalogSIMU", this, (mira::make_data_point<float>(
                    timestamp,
                    innov_sq
                )));
            }

            m_previousVisualPose = visualPose;
        }

        void OnTrackingLost()
        {
            m_trackingLost = true;
        }
    };

    AnalogCASIMU::AnalogCASIMU(const device::IMUCharacterization& imuCharacterization, bool assumeSamePosition)
        : m_impl{ std::allocate_shared<Impl, Eigen::aligned_allocator<Impl>>({}, imuCharacterization, assumeSamePosition) }
    {}

    AnalogCASIMU::AnalogCASIMU(const AnalogVFTFilter& filter, const mage::Pose& pose, mage::SensorSample::Timestamp time, bool assumeSamePosition)
        : m_impl{ std::allocate_shared<Impl, Eigen::aligned_allocator<Impl>>({}, filter, pose, time, assumeSamePosition) }
    {}

    AnalogCASIMU::~AnalogCASIMU() = default;

    void AnalogCASIMU::AddSample(const mage::SensorSample& sample)
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

    mage::Pose AnalogCASIMU::GetPose() const
    {
        return m_impl->GetPose();
    }

    void AnalogCASIMU::PredictUpTo(mage::SensorSample::Timestamp timestamp)
    {
        m_impl->PredictUpTo(timestamp);
    }

    void AnalogCASIMU::AddPose(const mage::Pose& pose, mage::SensorSample::Timestamp timestamp)
    {
        m_impl->ProcessPose(timestamp, pose);
    }

    void AnalogCASIMU::OnTrackingLost()
    {
        m_impl->OnTrackingLost();
    }
}
