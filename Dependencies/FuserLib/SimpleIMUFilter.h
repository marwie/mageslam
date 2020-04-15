#pragma once

#include <IMUFilterCAS.h>
#include <DiagLib/ConfigParam.h>

namespace ST
{
    class SimpleIMUFilter : public ST::IMUFilterCAS
    {
    public:
        bool update_with_gyro_simple(_In_reads_(3) const float z[3]);
        bool update_with_accel_simple(_In_reads_(3) const float z[3]);
        void predict(SensorTime::time_t t);
        void rotate_frame_reset(_In_ const SE3<T>& rotateToMage, SensorTime::time_t t);
        bool has_good_gravity() const;
        FuserUpdateStatusType predict_and_update(_In_ const IMUFilterEvent &imuFilterEvent, _In_ const IMUConfig &imu_config);
    private:
        bool hasIMUGravity = false;
    };

}
