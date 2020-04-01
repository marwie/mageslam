//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#include "Device.h"
#include "Utils/cv.h"
#include "Utils/Constants.h"
#include "Utils/MageConversions.h"
#include "arcana/math.h"
#include <Plat/Device/device.h>
#include <math.h>

namespace mage
{
    namespace device
    {
        IMUCharacterization GetIMUCharacterizationForEV3(const mira::CameraType& cameraType)
        {
            IMUCharacterization imuCharacterization{};

            // TODO: Determine IMU noise and bias sigmas for EV3. The following values are copied from EV2 for now
            imuCharacterization.useMagnetometer = false;                    //results on 950 are worse with mag, than without
            imuCharacterization.ApplySensitivityEstimation = false;         //can turn on when we have visual update
            imuCharacterization.DefaultInitialBiasVarianceFactor = 1.0f;    //variances are fine
            imuCharacterization.AccelSampleRateMS = 4.0f;                   //this is the requested value, measured varies based on phone state
            imuCharacterization.GyroSampleRateMS = 4.0f;
            imuCharacterization.MagSampleRateMS = 16.0;
            imuCharacterization.AccelNoiseSigma = 0.01f;                  //This value was measured in m/s^2. Using datasheet it might be in micro g's/sqrtHz and needs to be converted.
            imuCharacterization.GyroNoiseSigma = 0.8 * 1e-3f;             //This value was measured in rad/s. Using datasheet it might be in millidegrees/sec/sqrtHz and needs to be converted.
            imuCharacterization.MagNoiseSigma = 0.7f;                     //uT
            imuCharacterization.AccelBiasSigma = g_GravityMetersPerSecPerSec * 1e-3f;           // 1 millig in m/s/s
            imuCharacterization.GyroBiasSigma = 1e-3f;                    // 1 mrad/s
            imuCharacterization.MagBiasSigma = 30.0f;                     //microtesla

            cv::Matx44f rotNeg90Z = mage::RotationZForCoordinateFrames(mira::deg2rad(-mira::QUARTER_CIRCLE_DEGREES<float>));
            cv::Matx44f rot180X = mage::RotationXForCoordinateFrames(mira::deg2rad(mira::HALF_CIRCLE_DEGREES<float>));

            // Extrinsics
            bool inverted = false;
            if (cameraType == mira::CameraType::A_EV3_RGB)
            {
                cv::Matx44f bodyIMUToBodyNeptuneCameraRotation = rot180X * rotNeg90Z;
                cv::Matx44f translationIMUToNeptuneCamera = cv::Matx44f::eye();
                translationIMUToNeptuneCamera(0, 3) = 0.028012f;
                translationIMUToNeptuneCamera(1, 3) = -0.000785f;
                translationIMUToNeptuneCamera(2, 3) = -0.003631f;
                cv::Matx44f bodyIMUToBodyNeptuneCamera = translationIMUToNeptuneCamera * bodyIMUToBodyNeptuneCameraRotation;
                cv::Matx44f bodyNeptuneCameraToBodyIMU = bodyIMUToBodyNeptuneCamera.inv(cv::DECOMP_LU, &inverted);
                assert(inverted && "Failed to invert matrix");

                imuCharacterization.BodyIMUToBodyCamera = ArrayFromMat(bodyIMUToBodyNeptuneCamera);
                imuCharacterization.BodyCameraToBodyIMU = ArrayFromMat(bodyNeptuneCameraToBodyIMU);
            }
            else if (cameraType == mira::CameraType::A_EV3_WFOV)
            {
                cv::Matx44f bodyIMUToBodyWakeCameraRotation = rot180X * rotNeg90Z;
                cv::Matx44f translationIMUToWakeCamera = cv::Matx44f::eye();
                translationIMUToWakeCamera(0, 3) = 0.028012f;
                translationIMUToWakeCamera(1, 3) = -0.060915f;
                translationIMUToWakeCamera(2, 3) = -0.00216f;
                cv::Matx44f bodyIMUToBodyWakeCamera = translationIMUToWakeCamera * bodyIMUToBodyWakeCameraRotation;
                cv::Matx44f bodyWakeCameraToBodyIMU = bodyIMUToBodyWakeCamera.inv(cv::DECOMP_LU, &inverted);
                assert(inverted && "Failed to invert matrix");

                imuCharacterization.BodyIMUToBodyCamera = ArrayFromMat(bodyIMUToBodyWakeCamera);
                imuCharacterization.BodyCameraToBodyIMU = ArrayFromMat(bodyWakeCameraToBodyIMU);
            }

            return imuCharacterization;
        }

        CameraDevice GetCameraDeviceForA_EV2_RGB()
        {
            //values from payal 3/19/2018
            float K1 = 0.09917295970610188f;
            float K2 = -0.2736467674661127f;
            float K3 = 0.2025526939937347f;
            float P1 = 0.f; //0.01010385837290303
            float P2 = 0.f; //-0.0009137655633108411

            CameraDevice cameraDevice{};

            cameraDevice.CameraType = mira::CameraType::A_EV2_RGB;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                { 0, 1183.68884969074f / 1920.0f },  //fx M, B  
                { 0, 1183.29391173091f / 1080.0f },  //fy M, B 
                0.5f,                           //cx 950.6390563138477
                0.5,                            //cy 533.480829247079
                { 200, 200 },                   //focal bounds
                { 1920, 1080 },                  //calibration size
                { K1, K2, K3, P1, P2 });        //poly3k

            cameraDevice.DefaultCameraFocus = 120;


            return cameraDevice;
        }

        CameraDevice GetCameraDeviceForA_EV2_WFOV()
        {
            //values from payal 3/19/2018
            float rational6k_K1 = 14.72925307312375f;
            float rational6k_K2 = 34.35379624373142f;
            float rational6k_K3 = -2.786258482485339f;
            float rational6k_K4 = 14.28660579311959f;
            float rational6k_K5 = 32.02213712135136f;
            float rational6k_K6 = 2.551175997195161f;
            float rational6k_P1 = 0.f; // -0.003164321523827901
            float rational6k_P2 = 0.f; // -0.002849099726536717

            CameraDevice cameraDevice{};

            cameraDevice.CameraType = mira::CameraType::A_EV2_WFOV;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                { 0, 356.0558197787609f / 640.0f },  //fx M, B  
                { 0, 356.1715500096677f / 480.0f },  //fy M, B 
                0.5f,  //cx 325.0602298516793
                0.5f,   //cy 240.1881182874836
                { 200, 200 },               //focal bounds
                { 640, 480 },               //calibration size
                {},                         //poly3k
                { rational6k_K1, rational6k_K2, rational6k_K3, rational6k_K4, rational6k_K5, rational6k_K6, rational6k_P1, rational6k_P2 }); //rational6k

            cameraDevice.DefaultCameraFocus = 200; //fake number

            return cameraDevice;
        }


        CameraDevice GetCameraDeviceForA_EV3_RGB()
        {
            //values from DeviceDb.cpp 5/1/2018
            float K1 = 0.0948010097142857f;
            float K2 = -0.222682427071429f;
            float K3 = 0.142323492928571f;
            float P1 = 0.f; //-0.0154263542857143
            float P2 = 0.f; //0.0137627781428571

            CameraDevice cameraDevice{};

            cameraDevice.CameraType = mira::CameraType::A_EV3_RGB;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                { 0, 1175.66533535714f / 1920.0f },  //fx M, B  
                { 0, 1177.33111671429f / 1080.0f },  //fy M, B 
                0.5f,                           //cx 964.2370172
                0.5,                            //cy 535.421278
                { 525, 525 },                   //focal bounds
                { 1920, 1080 },                  //calibration size
                { K1, K2, K3, P1, P2 });        //poly3k

            cameraDevice.DefaultCameraFocus = 525; //number from DeviceDb.cpp 5/1/2018


            return cameraDevice;
        }

        CameraDevice GetCameraDeviceForA_EV3_WFOV()
        {
            //values from payal 5/1/2018
            float rational6k_K1 = 4.352465191775507f;
            float rational6k_K2 = -0.4406155500260119f;
            float rational6k_K3 = -0.3964952078515903f;
            float rational6k_K4 = 3.964526507499666f;
            float rational6k_K5 = 0.2441246261188355f;
            float rational6k_K6 = -0.4676361436730525f;
            float rational6k_P1 = 0.f; // -0.0008876017797408126
            float rational6k_P2 = 0.f; // 0.0168160573509395

            CameraDevice cameraDevice{};

            cameraDevice.CameraType = mira::CameraType::A_EV3_WFOV;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                { 0, 354.6051975897762f / 640.0f },  //fx M, B  
                { 0, 355.1935470730183f / 480.0f },  //fy M, B 
                0.5f,  //cx 318.7977243002715
                0.5f,   //cy 245.6569515426437
                { 200, 200 },               //focal bounds
                { 640, 480 },               //calibration size
                {},                         //poly3k
                { rational6k_K1, rational6k_K2, rational6k_K3, rational6k_K4, rational6k_K5, rational6k_K6, rational6k_P1, rational6k_P2 }); //rational6k

            cameraDevice.DefaultCameraFocus = 200; //fake number

            return cameraDevice;
        }

        IMUCharacterization GetIMUCharacterizationForEV2(const mira::CameraType& cameraType)
        {
            IMUCharacterization imuCharacterization{};

            // TODO: Determine IMU settings for A:  https://microsoft.visualstudio.com/DefaultCollection/OS/_workitems?id=12403786&_a=edit
            // The following values are copied from 950 for now
            imuCharacterization.useMagnetometer = false;                    //results on 950 are worse with mag, than without
            imuCharacterization.ApplySensitivityEstimation = false;         //can turn on when we have visual update
            imuCharacterization.DefaultInitialBiasVarianceFactor = 1.0f;    //variances are fine
            imuCharacterization.AccelSampleRateMS = 4.0f;                   //this is the requested value, measured varies based on phone state
            imuCharacterization.GyroSampleRateMS = 4.0f;
            imuCharacterization.MagSampleRateMS = 16.0;
            imuCharacterization.AccelNoiseSigma = 0.01f;                  //This value was measured in m/s^2. Using datasheet it might be in micro g's/sqrtHz and needs to be converted.
            imuCharacterization.GyroNoiseSigma = 0.8 * 1e-3f;             //This value was measured in rad/s..Using datasheet it might be in millidegrees/sec/sqrtHz and needs to be converted.
            imuCharacterization.MagNoiseSigma = 0.7f;                     //uT
            imuCharacterization.AccelBiasSigma = g_GravityMetersPerSecPerSec * 1e-3f;           // 1 millig in m/s/s
            imuCharacterization.GyroBiasSigma = 1e-3f;                    // 1 mrad/s
            imuCharacterization.MagBiasSigma = 30.0f;                     //microtesla

            // Right now on EV2, we are using the wake camera and the imu both in the R2 panel (R2 is the right panel, C3 is the left panel)
            // The rotation between the camera and the imu is the same as 950
            cv::Matx44f rotNeg90Z = mage::RotationZForCoordinateFrames(mira::deg2rad(-mira::QUARTER_CIRCLE_DEGREES<float>));
            cv::Matx44f rot180X = mage::RotationXForCoordinateFrames(mira::deg2rad(mira::HALF_CIRCLE_DEGREES<float>));

            // Extrinsics
            bool inverted = false;
            if (cameraType == mira::CameraType::A_EV2_RGB)
            {
                cv::Matx44f bodyIMUToBodyNeptuneCameraRotation = rot180X * rotNeg90Z;
                cv::Matx44f translationIMUToNeptuneCamera = cv::Matx44f::eye();
                translationIMUToNeptuneCamera(0, 3) = 0.030862f;
                translationIMUToNeptuneCamera(1, 3) = -0.00001f;
                translationIMUToNeptuneCamera(2, 3) = -0.002731f;
                cv::Matx44f bodyIMUToBodyNeptuneCamera = translationIMUToNeptuneCamera * bodyIMUToBodyNeptuneCameraRotation;
                cv::Matx44f bodyNeptuneCameraToBodyIMU = bodyIMUToBodyNeptuneCamera.inv(cv::DECOMP_LU, &inverted);
                assert(inverted && "Failed to invert matrix");

                imuCharacterization.BodyIMUToBodyCamera = ArrayFromMat(bodyIMUToBodyNeptuneCamera);
                imuCharacterization.BodyCameraToBodyIMU = ArrayFromMat(bodyNeptuneCameraToBodyIMU);
            }
            else if (cameraType == mira::CameraType::A_EV2_WFOV)
            {
                cv::Matx44f bodyIMUToBodyWakeCameraRotation = rot180X * rotNeg90Z;
                cv::Matx44f translationIMUToWakeCamera = cv::Matx44f::eye();
                translationIMUToWakeCamera(0, 3) = 0.030862f;
                translationIMUToWakeCamera(1, 3) = -0.05817f;
                translationIMUToWakeCamera(2, 3) = -0.001461f;
                cv::Matx44f bodyIMUToBodyWakeCamera = translationIMUToWakeCamera * bodyIMUToBodyWakeCameraRotation;
                cv::Matx44f bodyWakeCameraToBodyIMU = bodyIMUToBodyWakeCamera.inv(cv::DECOMP_LU, &inverted);
                assert(inverted && "Failed to invert matrix");

                imuCharacterization.BodyIMUToBodyCamera = ArrayFromMat(bodyIMUToBodyWakeCamera);
                imuCharacterization.BodyCameraToBodyIMU = ArrayFromMat(bodyWakeCameraToBodyIMU);
            }

            return imuCharacterization;
        }

        CameraDevice GetCameraDeviceForSurfacePro3()
        {
            CameraDevice cameraDevice{};

            float K1 = 0.f;
            float K2 = 0.f;
            float K3 = 0.f;
            float P1 = 0.f;
            float P2 = 0.f;

            cameraDevice.CameraType = mira::CameraType::SurfacePro3;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                // Calibrated off of Brent's SP3
                {
                    {0, 1845.75f / 1920.0f},        // fx  M, B
                    {0, 1840.4f / 1080.0f},         // fy  M, B
                    979.76f / 1920.0f,              // cx
                    573.47f / 1080.0f,              // cy
                    {0,0},                          // focal bounds (N/A)
                    {1920, 1080},                   // calibration size
                    { K1, K2, K3, P1, P2 }          // poly3k
                });

            return cameraDevice;
        }

        CameraDevice GetCameraDeviceForSurfaceBook()
        {
            CameraDevice cameraDevice{};

            float K1 = 0.f;
            float K2 = 0.f;
            float K3 = 0.f;
            float P1 = 0.f;
            float P2 = 0.f;

            cameraDevice.CameraType = mira::CameraType::SurfaceBook;

            cameraDevice.Model = calibration::LinearFocalLengthModel(
                // Calibrated off of Nick's Surface Book
                // note that surface books have a decent autofocus, so this model could be refined
                // current values are focussed at around 3/4 of a meter
                {
                    { 0, 1587.29f / 1920.0f },          // fx  M, B
                    { 0, 1585.59f / 1080.0f },          // fy  M, B
                    963.24f / 1920.0f,                  // cx
                    560.54f / 1080.0f,                  // cy
                    { 0,0 },                            // focal bounds (N/A)
                    { 1920, 1080 },                     // calibration size
                    { K1, K2, K3, P1, P2 }              // poly3k
                });

            return cameraDevice;
        }

        CameraDevice GetCameraDeviceForLumia950()
        {
            float K0 = 0.094227405f;
            float K1 = -0.350755726f;
            float K2 = 0.416357188f;
            float P0 = 0.0f;
            float P1 = 0.0f;

            CameraDevice cameraDevice{};

            cameraDevice.CameraType = mira::CameraType::Lumia950;

            //these calibration numbers came from Analog, 09/27/16
            cameraDevice.Model = calibration::LinearFocalLengthModel(
                { -0.0001100515625f, 0.81877777291667f },   //fx M, B
                { -0.0001882685185f, 1.45169039537037f },   //fy M, B
                0.506385416667f,                            //cx
                0.51153703703704f,                          //cy
                { 550.0f, 700.0f },                         //focal bounds
                { 1920, 1080 },                              //calibration size
                { K0, K1, K2, P0, P1 });                    //poly3k

            cameraDevice.DefaultCameraFocus = 650;

            return cameraDevice;
        }

        IMUCharacterization GetIMUCharacterizationForLumia950()
        {
            // The image from the sensor on the 950 is always based on landscape, so holding the phone in portrait mode will pin the mage camera space convention(x right, y down, z forward) 
            // so that x points down and y points left, z is still forward.
            // The imu is installed on the 950 and exposed by the sensors api so that with the phone in portrait mode it is y up to the top of phone, x to right, and z back towards the user.
            // To rotate from body camera to body imu  rotate around camera z - 90. then rotate around that x axis 180 degrees.
            // column major, math flows right to left
            //cv::Matx44f rotNeg90Z = mage::RotationZForCoordinateFrames(mira::deg2rad(-mira::QUARTER_CIRCLE_DEGREES<float>));
            //cv::Matx44f rot180X = mage::RotationXForCoordinateFrames(mira::deg2rad(mira::HALF_CIRCLE_DEGREES<float>));
            //cv::Matx44f bodyIMUToBodyCameraRotation = rot180X * rotNeg90Z;

            // Ran the analog calibration tool with data using their 3d calibration marker board thanks to Eric Huang.
            // These values are taken straight from the calibration.json file "Rt" property.
            cv::Matx44f bodyCameraToBodyIMU{
                -0.0023918196093291044, -0.99980247020721436, 0.019730480387806892, 0.02890799380838871,
                -0.99998271465301514, 0.0024972527753561735, 0.0053207604214549065, 0.10563744604587555,
                -0.0053689810447394848, -0.019717413932085037, -0.99979120492935181, 0.0064810086041688919,
                0, 0, 0, 1
            };

            bool inverted = false;
            cv::Matx44f bodyIMUToBodyCamera = bodyCameraToBodyIMU.inv(cv::DECOMP_LU, &inverted);
            assert(inverted && "Failed to invert matrix");

            IMUCharacterization imuCharacterization{};
            imuCharacterization.useMagnetometer = false;                    //results on 950 are worse with mag, than without
            imuCharacterization.ApplySensitivityEstimation = false;         //can turn on when we have visual update
            imuCharacterization.DefaultInitialBiasVarianceFactor = 1.0f;    //variances are fine
            imuCharacterization.AccelSampleRateMS = 4.0f;                   //this is the requested value, measured varies based on phone state
            imuCharacterization.GyroSampleRateMS = 4.0f;
            imuCharacterization.MagSampleRateMS = 16.0;
            imuCharacterization.AccelNoiseSigma = 250.0f * 1e-6f * g_GravityMetersPerSecPerSec * sqrtf(0.5f * 1.0f / (1e-3f * imuCharacterization.AccelSampleRateMS));       // micro g's/sqrtHz to m/s^2
            imuCharacterization.GyroNoiseSigma = mira::deg2rad(20.0f * 1e-3f) * sqrtf(0.5f  * 1.0f / (1e-3f * imuCharacterization.GyroSampleRateMS));  // millidegrees/sec/sqrtHz to rad/s, (assume bandwidth is about half the sample rate). 
            imuCharacterization.MagNoiseSigma = 0.7f;                                                                                                  // uT
            imuCharacterization.AccelBiasSigma = 80.0f * g_GravityMetersPerSecPerSec * 1e-3f;            // 80 millig in m/s/s
            imuCharacterization.GyroBiasSigma = mira::deg2rad(20.0f) * 1e-3f;    // 20 degrees/sec = (20 * 3.14) / 180 mrad/s
            imuCharacterization.MagBiasSigma = 30.0f;                           //microtesla

            imuCharacterization.BodyIMUToBodyCamera = ArrayFromMat(bodyIMUToBodyCamera);
            imuCharacterization.BodyCameraToBodyIMU = ArrayFromMat(bodyCameraToBodyIMU);

            return imuCharacterization;
        }

        //returns a passive matrix that takes you from the device's origin (panel origin) in CAD space (device in portrait, looking at screen, x right, y up, z to user)
        // to the camera space center of the sensor in opencv convention(camera center, looking through camera x right, y down, z forward)
        mage::Matrix GetExtrinsics(mira::CameraType cameraType)
        {

            //TODO: we don't have rotational extrinsics calculated yet
            cv::Vec3f posRGBInPanel_A_EV2{ 0.033145f, 0.063887f, -0.004441f };
            cv::Vec3f posWFOVInPanel_A_EV2{ -0.025015f, 0.063887f,-0.003171f };

            cv::Vec3f posRGBInPanel_A_EV3{ 0.034075f, 0.060072f, -0.005019f };
            cv::Vec3f posWFOVInPanel_A_EV3{ -0.026055f, 0.060072f,-0.003604f };

            // sensors appear to be oriented with image upper left in the top right corner when device is held in portrait
            cv::Matx44f cadToSensorRotation{ 0, -1,  0, 0,
                                            -1,  0,  0, 0,
                                             0,  0, -1, 0,
                                             0,  0,  0, 1 };
            switch (cameraType)
            {
            case mira::CameraType::A_EV2_RGB:
            {
                //turns position into transform
                cv::Matx44f transPanelOriginToPanelRGB{ 1,0,0,-posRGBInPanel_A_EV2[0],
                    0,1,0,-posRGBInPanel_A_EV2[1],
                    0,0,1,-posRGBInPanel_A_EV2[2],
                    0,0,0,1 };
               
                cv::Matx44f panelOriginToCVRGB = cadToSensorRotation * transPanelOriginToPanelRGB;

                return ToMageMat(panelOriginToCVRGB);
            }
            case mira::CameraType::A_EV2_WFOV:
            {
                //turns position into transform
                cv::Matx44f transPanelOriginToPanelWFOV{ 1,0,0,-posWFOVInPanel_A_EV2[0],
                    0,1,0,-posWFOVInPanel_A_EV2[1],
                    0,0,1,-posWFOVInPanel_A_EV2[2],
                    0,0,0,1 };
                cv::Matx44f panelOriginToCVWFOV = cadToSensorRotation * transPanelOriginToPanelWFOV;

                assert((panelOriginToCVWFOV.inv() * cv::Vec4f(0, 0, 0, 1)) == cv::Vec4f(posWFOVInPanel_A_EV2[0], posWFOVInPanel_A_EV2[1], posWFOVInPanel_A_EV2[2], 1));
                return ToMageMat(panelOriginToCVWFOV);
            }
            case mira::CameraType::A_EV3_RGB:
            {
                //turns position into transform
                cv::Matx44f transPanelOriginToPanelRGB{ 1,0,0,-posRGBInPanel_A_EV3[0],
                    0,1,0,-posRGBInPanel_A_EV3[1],
                    0,0,1,-posRGBInPanel_A_EV3[2],
                    0,0,0,1 };
                
                cv::Matx44f panelOriginToCVRGB = cadToSensorRotation * transPanelOriginToPanelRGB;

                return ToMageMat(panelOriginToCVRGB);
            }
            case mira::CameraType::A_EV3_WFOV:
            {
                //turns position into transform
                cv::Matx44f transPanelOriginToPanelWFOV{ 1,0,0,-posWFOVInPanel_A_EV3[0],
                    0,1,0,-posWFOVInPanel_A_EV3[1],
                    0,0,1,-posWFOVInPanel_A_EV3[2],
                    0,0,0,1 };
                cv::Matx44f panelOriginToCVWFOV = cadToSensorRotation * transPanelOriginToPanelWFOV;

                assert((panelOriginToCVWFOV.inv() * cv::Vec4f(0, 0, 0, 1)) == cv::Vec4f(posWFOVInPanel_A_EV3[0], posWFOVInPanel_A_EV3[1], posWFOVInPanel_A_EV3[2], 1));
                return ToMageMat(panelOriginToCVWFOV);
            }
            case mira::CameraType::Lumia950:
            case mira::CameraType::SurfacePro3:
            case mira::CameraType::SurfaceBook:
            {
                return ToMageMat(cv::Matx44f::eye());
            }
            default:
                assert(false && "Intrinsics not provided for camera type");
                return ToMageMat(cv::Matx44f::zeros());
            }
        }

        Matrix GetTransformFromWFOVToRGB(mira::DeviceType deviceType)
        {
            mira::CameraType rgbCameraType;
            mira::CameraType wfovCameraType;

            switch (deviceType)
            {
            case mira::DeviceType::A_EV2:
                rgbCameraType = mira::CameraType::A_EV2_RGB;
                wfovCameraType = mira::CameraType::A_EV2_WFOV;
                break;
            case mira::DeviceType::A_EV3:
                rgbCameraType = mira::CameraType::A_EV3_RGB;
                wfovCameraType = mira::CameraType::A_EV3_WFOV;
                break;
            default:
                assert(false && "No known transformation for the given device type");
                return ToMageMat(cv::Matx44f::eye());
            }

            cv::Matx44f panelOriginToNarrow = ToCVMat4x4(device::GetExtrinsics(rgbCameraType));
            cv::Matx44f panelOriginToWide = ToCVMat4x4(device::GetExtrinsics(wfovCameraType));

            return ToMageMat(panelOriginToNarrow * panelOriginToWide.inv());
        }

        //translations in meters
        Matrix GetTransformFromWFOVToRGB_EV2()
        {
            cv::Matx44f panelOriginToNarrow = ToCVMat4x4(device::GetExtrinsics(mira::CameraType::A_EV2_RGB));
            cv::Matx44f panelOriginToWide = ToCVMat4x4(device::GetExtrinsics(mira::CameraType::A_EV2_WFOV));

            return ToMageMat(panelOriginToNarrow * panelOriginToWide.inv());
        }

        Matrix GetTransformFromWFOVToRGB_EV3()
        {
            cv::Matx44f panelOriginToNarrow = ToCVMat4x4(device::GetExtrinsics(mira::CameraType::A_EV3_RGB));
            cv::Matx44f panelOriginToWide = ToCVMat4x4(device::GetExtrinsics(mira::CameraType::A_EV3_WFOV));

            return ToMageMat(panelOriginToNarrow * panelOriginToWide.inv());
        }

        std::map<mira::CameraType, const mage::CameraIdentity> GetDeviceCameraBindings(const mira::DeviceType& deviceType, const mira::RuntimeType& runtime, const StereoSettings& stereoSettings)
        {
            switch (deviceType)
            {
            case mira::DeviceType::Lumia950:
                return { {mira::CameraType::Lumia950,  mage::CameraIdentity::MONO} };
            case mira::DeviceType::SurfacePro3:
                return { { mira::CameraType::SurfacePro3, mage::CameraIdentity::MONO } };
            case mira::DeviceType::SurfaceBook:
                return { { mira::CameraType::SurfaceBook, mage::CameraIdentity::MONO } };
            case mira::DeviceType::A_EV2:
                switch (runtime)
                {
                case mira::RuntimeType::Mono:
                    if (stereoSettings.PrimaryTrackingCamera == mage::CameraIdentity::STEREO_1)
                    {
                        return { { mira::CameraType::A_EV2_WFOV, mage::CameraIdentity::MONO } };
                    }
                    else
                    {
                        return { { mira::CameraType::A_EV2_RGB, mage::CameraIdentity::MONO } };
                    }
                case mira::RuntimeType::Stereo:
                    return { { mira::CameraType::A_EV2_WFOV, mage::CameraIdentity::STEREO_1 }, { mira::CameraType::A_EV2_RGB, mage::CameraIdentity::STEREO_2 } };
                default:
                    throw std::invalid_argument("Unknown runtime type");
                }
            case mira::DeviceType::A_EV3:
                switch (runtime)
                {
                case mira::RuntimeType::Mono:
                    if (stereoSettings.PrimaryTrackingCamera == mage::CameraIdentity::STEREO_1)
                    {
                        return { { mira::CameraType::A_EV3_WFOV, mage::CameraIdentity::MONO } };
                    }
                    else
                    {
                        return { { mira::CameraType::A_EV3_RGB, mage::CameraIdentity::MONO } };
                    }
                case mira::RuntimeType::Stereo:
                    return { { mira::CameraType::A_EV3_WFOV, mage::CameraIdentity::STEREO_1 },{ mira::CameraType::A_EV3_RGB, mage::CameraIdentity::STEREO_2 } };
                default:
                    throw std::invalid_argument("Unknown runtime type");
                }
            default:
                throw std::invalid_argument("Unknown runtime type");
            }
        }
    }
}
