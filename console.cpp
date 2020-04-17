#include "Core/MAGESLAM/Source/MageSlam.h"

#include <iostream>

void main()
{
    std::cout << "Hello, world!" << std::endl;

    mage::MageSlamSettings settings{};

    mage::Size imageSize{};
    mage::Matrix extrinsics{};
    mage::MAGESlam::CameraConfiguration configuration{ mage::CameraIdentity::MONO, imageSize, mage::PixelFormat::GRAYSCALE8, extrinsics };
    auto configurations = gsl::make_span<mage::MAGESlam::CameraConfiguration>(&configuration, 1);

    mage::device::IMUCharacterization imuCharacterization{};

    auto slam = std::make_unique<mage::MAGESlam>(settings, configurations, imuCharacterization);

    mage::Intrinsics intrinsics{};
    auto cameraModel = std::make_shared<mage::calibration::PinholeCameraModel>(intrinsics);

    mage::FrameId frameId{};
    std::chrono::system_clock::time_point timePoint{};
    mira::CameraSettings cameraSettings{};
    mage::MAGESlam::FrameFormat format{ frameId, cameraModel, timePoint, cameraSettings };
    
    auto pixels = gsl::make_span<uint8_t>(nullptr, 0);
    mage::MAGESlam::Frame frame{ format, pixels };
    slam->ProcessFrame(frame);
    
    mage::MAGESlam::Fossilize(std::move(slam));
}