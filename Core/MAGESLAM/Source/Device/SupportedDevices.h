#pragma once

#include "Device.h"

namespace mage
{
    namespace device
    {
        // supported Cameras
        CameraDevice GetCameraDeviceForA_EV2_RGB();
        CameraDevice GetCameraDeviceForA_EV2_WFOV();
        CameraDevice GetCameraDeviceForA_EV3_RGB();
        CameraDevice GetCameraDeviceForA_EV3_WFOV();
        CameraDevice GetCameraDeviceForLumia950();
        CameraDevice GetCameraDeviceForSurfacePro3();
        CameraDevice GetCameraDeviceForSurfaceBook();

        mage::Matrix GetExtrinsics( mira::CameraType cameraType);
        Matrix GetTransformFromWFOVToRGB_EV2();
        Matrix GetTransformFromWFOVToRGB_EV3();

        // supported IMUs
        IMUCharacterization GetIMUCharacterizationForEV3(const mira::CameraType& cameraType);
        IMUCharacterization GetIMUCharacterizationForEV2(const mira::CameraType& cameraType);
        IMUCharacterization GetIMUCharacterizationForLumia950();

        // typical mappings for each device
        std::map<mira::CameraType, const mage::CameraIdentity> GetDeviceCameraBindings(const mira::DeviceType& deviceType, const mira::RuntimeType& runtime, const StereoSettings& stereoSettings);
    }
}
