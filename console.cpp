#include "Core/MAGESLAM/Source/MageSlam.h"

#include <iostream>

void main()
{
    std::cout << "Hello, world!" << std::endl;

    mage::MAGESlam slam{ {}, gsl::make_span<mage::MAGESlam::CameraConfiguration>(nullptr, 0), {} };
    mage::MAGESlam::FrameFormat format{ mage::FrameId{}, nullptr, std::chrono::system_clock::time_point{}, mira::CameraSettings() };
    slam.ProcessFrame(mage::MAGESlam::Frame(format, gsl::make_span<uint8_t>(nullptr, 0)));
    slam.Fossilize(nullptr);
}