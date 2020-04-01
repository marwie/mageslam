#include "AnalogVFTFilter.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Utils/cv.h"
#include <boost/optional.hpp>

#include <arcana/containers/sorted_vector.h>
#include <arcana/analysis/object_trace.h>
#include <arcana/analysis/data_point.h>

#include <VFTFilter.h>

#include "Utils/Logging.h"

#include "FuserFunctions.h"
#include "ImuMath.h"

namespace E = Eigen;

namespace mage
{
    using seconds = std::chrono::duration<double>;

    struct /*alignas(16)*/ AnalogVFTFilter::Impl
    {
        mira::sorted_vector<mage::SensorSample, mage::SensorSample::Compare> m_samples;

        std::array<float, 3> m_previousAccel{};

        bool m_initialized = false;

        ST::VFTFilter m_filter{};
        ST::IMUConfig m_config{};

        device::IMUCharacterization m_characterization;

        Impl(const device::IMUCharacterization& imuCharacterization)
            : m_config{ BuildIMUConfigFromCharacterization(imuCharacterization) }
            , m_characterization{ imuCharacterization }
        {}

        void PredictUpTo(mage::SensorSample::Timestamp timestamp)
        {
            auto from = m_samples.begin();
            auto to = std::find_if(from, m_samples.end(), [timestamp](const mage::SensorSample& sample)
            {
                return sample.GetTimestamp() > timestamp;
            });

            for (auto gyro = from; gyro != to; ++gyro)
            {
                if (gyro->GetType() == mage::SensorSample::SampleType::Gyrometer)
                {
                    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(gyro->GetTimestamp().time_since_epoch());

                    if (!m_initialized)
                    {
                        m_filter.reset(ST::SensorTime::fromMicroseconds(microseconds.count()), m_config);

                        // set the filters SO3
                        m_filter.x = ST::SO3_identity<double>();

                        // zero out its integrated position (it has one but we shouldn't use it)
                        ST::zero(m_filter.pos);

                        m_initialized = true;
                    }

                    E::Map<imu::Vector<float>> accelData{ m_previousAccel.data(), 3 };

                    const auto* accelSample = mage::FindCorrespondingAccelSample(*gyro, m_samples);
                    if (accelSample != nullptr)
                    {
                        accelData = E::Map<const imu::Vector<float>>{ accelSample->GetData().data(), 3 };
                    }

                    ST::IMUEvent ev{};
                    ev.timestamp = ST::SensorTime::fromMicroseconds(microseconds.count());
                    ev.magnetometer_timestamp = ST::SensorTime::invalidTime();

                    E::Map<imu::Vector<float>>{ev.rotvel} = E::Map<const imu::Vector<float>>(gyro->GetData().data(), 3);
                    E::Map<imu::Vector<float>>{ev.accel} = accelData;

                    ST::IMUFilterEvent filterEv{ ev };

                    auto result = m_filter.predict_and_update(filterEv, m_config);
                    if (result != ST::FuserUpdateStatus::NO_UPDATE_ERROR)
                    {
                        std::wstringstream ss;
                        ss << L"VFT Failed to predict and update based on the filter event: " << std::hex << result << "\n";
                        mage::LogMessage<>(ss.str());
                    }

                    FIRE_OBJECT_TRACE("IMU Linear Velocity.AnalogVFT", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        (float)E::Vector3d{ m_filter.u[0], m_filter.u[1], m_filter.u[2] }.norm()
                    )));

                    FIRE_OBJECT_TRACE("IMU Linear Accel.AnalogVFT", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        (float)E::Vector3d{ m_filter.a[0], m_filter.a[1], m_filter.a[2] }.norm()
                    )));

                    FIRE_OBJECT_TRACE("Gravity AnalogVFT", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.g[0], m_filter.g[1], m_filter.g[2] }.cast<float>()
                    )));

                    FIRE_OBJECT_TRACE("Gravity Converged.AnalogVFT", this, (mira::make_data_point<float>(
                        gyro->GetTimestamp(),
                        m_filter.has_good_gravity() ? 1 : 0
                    )));

                    FIRE_OBJECT_TRACE("GyroBias AnalogVFT", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.gyro_bias[0], m_filter.gyro_bias[1], m_filter.gyro_bias[2] }.cast<float>()
                    )));

                    FIRE_OBJECT_TRACE("AccelBias AnalogVFT", this, (mira::make_data_point<E::Vector3f>(
                        gyro->GetTimestamp(),
                        E::Vector3d{ m_filter.accel_bias[0], m_filter.accel_bias[1], m_filter.accel_bias[2] }.cast<float>()
                    )));
                }
            }

            m_samples.erase(from, to);

            auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch());
            m_filter.predict(ST::SensorTime::fromMicroseconds(microseconds.count()));
        }
    };

    AnalogVFTFilter::AnalogVFTFilter(const device::IMUCharacterization& imuCharacterization)
        : m_impl{ std::allocate_shared<Impl, Eigen::aligned_allocator<Impl>>({}, imuCharacterization) }
    {}

    AnalogVFTFilter::~AnalogVFTFilter() = default;

    const ST::VFTFilter& AnalogVFTFilter::GetInternalFilter() const
    {
        return m_impl->m_filter;
    }

    const device::IMUCharacterization& AnalogVFTFilter::GetCharacterization() const
    {
        return m_impl->m_characterization;
    }

    void AnalogVFTFilter::AddSample(const mage::SensorSample& sample)
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

    void AnalogVFTFilter::PredictUpTo(mage::SensorSample::Timestamp timestamp)
    {
        m_impl->PredictUpTo(timestamp);
    }

    bool AnalogVFTFilter::IsConverged() const
    {
        return m_impl->m_filter.has_good_gravity();
    }

    gsl::span<const mage::SensorSample> AnalogVFTFilter::GetSamples() const
    {
        return m_impl->m_samples;
    }
}
