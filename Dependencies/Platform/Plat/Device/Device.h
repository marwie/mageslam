// Copyright (C) Microsoft Corporation. All rights reserved.

#pragma once

namespace mira
{
    enum class CameraType
    {
        Unknown,
        Lumia950,
        A_EV2_RGB,
        A_EV2_WFOV,
        SurfacePro3,
        SurfaceBook,
        A_EV3_RGB,
        A_EV3_WFOV,
    };

    inline constexpr char const* ToString(CameraType type)
    {        
        switch (type)
        {
        case CameraType::Unknown:
            return "Unknown";
        case CameraType::Lumia950:
            return "Lumia950";
        case CameraType::A_EV2_RGB:
            return "A_EV2_RGB";
        case CameraType::A_EV2_WFOV:
            return "A_EV2_WFOV";
        case CameraType::A_EV3_RGB:
            return "A_EV3_RGB";
        case CameraType::A_EV3_WFOV:
            return "A_EV3_WFOV";
        case CameraType::SurfaceBook:
            return "SurfaceBook";
        case CameraType::SurfacePro3:
            return "SurfacePro3";
        default:
            return "";
        }
    }

    enum class DeviceType
    {
        Unknown,
        Lumia950,
        A_EV2,
        SurfacePro3,
        SurfaceBook,
        A_EV3,
        AnalogSynth,
        Middlebury,
    };

    enum class RuntimeType
    {
        Mono,
        Stereo
    };
}
