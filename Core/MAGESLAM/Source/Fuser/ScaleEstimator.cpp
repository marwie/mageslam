#include "ScaleEstimator.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Utils/cv.h"
#include <boost/optional.hpp>

#include <arcana/analysis/object_trace.h>
#include <arcana/analysis/data_point.h>

#include "ImuMath.h"

namespace E = Eigen;

namespace mage
{
    using seconds = std::chrono::duration<double>;

    struct Entry
    {
        std::chrono::system_clock::time_point TimeStamp;
        double LinearAccelerationNorm;
        E::Vector3d Position;
    };

    struct /*alignas(16)*/ ScaleEstimator::Impl
    {
        std::vector<Entry, E::aligned_allocator<Entry>> Entries;
        ST::SE3<double> BodyCameraToBodyIMU;
    };

    ScaleEstimator::ScaleEstimator(const device::IMUCharacterization& imuCharacterization)
        : m_impl{ std::allocate_shared<Impl, Eigen::aligned_allocator<Impl>>({}) }
    {
        m_impl->BodyCameraToBodyIMU = imu::ToSE3(E::Map<const imu::Matrix<float>>(imuCharacterization.BodyCameraToBodyIMU.data(), 4, 4));
    }

    ScaleEstimator::~ScaleEstimator() = default;

    float ScaleEstimator::GetScale() const
    {
        const int WINDOW_SIZE = 1;

        if (m_impl->Entries.size() < WINDOW_SIZE + 2)
            return 1.f;

        E::MatrixXd metricAccelerations(WINDOW_SIZE, 1);
        E::MatrixXd unitlessAccelerations(WINDOW_SIZE, 1);

        const int offset = gsl::narrow_cast<int>(m_impl->Entries.size()) - WINDOW_SIZE - 1;

        std::chrono::system_clock::time_point lastTs{};
        for (int i = 0; i < WINDOW_SIZE; ++i)
        {
            auto* entry = &m_impl->Entries[offset + i];
            metricAccelerations(i, 0) = entry->LinearAccelerationNorm;

            lastTs = entry->TimeStamp;

            auto dt = seconds{ (entry + 1)->TimeStamp - (entry - 1)->TimeStamp }.count() / 2;

            E::Vector3d accel = ((entry + 1)->Position - 2 * entry->Position + (entry - 1)->Position) / (dt * dt);
            unitlessAccelerations(i, 0) = accel.norm();
        }
        
        double scale = unitlessAccelerations.colPivHouseholderQr().solve(metricAccelerations)(0);
        //double scale = unitlessAccelerations.jacobiSvd(E::ComputeThinU | E::ComputeThinV).solve(metricAccelerations)(0);

        FIRE_OBJECT_TRACE("IMU Linear Accel.ScaleEstimator_U", this, (mira::make_data_point<float>(
            lastTs, unitlessAccelerations(WINDOW_SIZE - 1, 0)))
        );

        FIRE_OBJECT_TRACE("IMU Linear Accel.ScaleEstimator_M", this, (mira::make_data_point<float>(
            lastTs, metricAccelerations(WINDOW_SIZE - 1, 0)))
        );

        FIRE_OBJECT_TRACE("Scale Error.ScaleEstimator", this, (mira::make_data_point<float>(
            lastTs, (unitlessAccelerations * scale - metricAccelerations).norm()))
        ); // error in meters

        return scale;
    }

    void ScaleEstimator::AddEntry(float metricLinearAcceleration, const mage::Pose& pose, const std::chrono::system_clock::time_point& timestamp)
    {
        // Convert the pose to view SE3 then change the pose to be the imu device view matrix.
        // Take the invert of that to get the world position.
        ST::SE3<double> visualPoseInIMU = ST::SE3_inverse(m_impl->BodyCameraToBodyIMU * imu::PoseToViewSE3(pose));
        
        m_impl->Entries.push_back(Entry{
            timestamp,
            (double)metricLinearAcceleration,
            E::Map<const E::Vector3d>(visualPoseInIMU.t)
        });
    }
}
