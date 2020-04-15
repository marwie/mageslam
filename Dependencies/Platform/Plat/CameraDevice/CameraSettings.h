//
// Copyright (C) Microsoft Corporation. All rights reserved.
//
#pragma once

#include <Plat/TimeDefinitions.h>
#include <boost/optional.hpp>
#include <chrono>

namespace mira
{
    struct CameraSettings
    {
        // Used for serialization.
        struct Fields
        {
            union
            {
                struct
                {
                    bool ExposureTimeValid : 1;
                    bool WhiteBalanceValid : 1;
                    bool LensPositionValid : 1;
                    bool IsoSpeedValid : 1;
                    bool IsoAnalogGainValid : 1;
                    bool IsoDigitalGainValid : 1;
                } bits;
                uint32_t data;
            };
        };

        CameraSettings() = default;

        CameraSettings(
            boost::optional<uint32_t> whiteBalance,
            boost::optional<HundredsNanoseconds> exposureTime,
            boost::optional<uint32_t> lensPosition,
            boost::optional<uint32_t> isoSpeed,
            boost::optional<float> isoAnalogGain,
            boost::optional<float> isoDigitalGain)
            : WhiteBalance{ whiteBalance },
            ExposureTime{ exposureTime },
            LensPosition{ lensPosition },
            IsoSpeed{ isoSpeed },
            IsoAnalogGain{ isoAnalogGain },
            IsoDigitalGain{ isoDigitalGain }
        {}

        void SetLensPosition(const boost::optional<uint32_t> lensPosition)
        {
            LensPosition = lensPosition;
        }

        void SetWhiteBalance(const boost::optional<uint32_t> whiteBalance)
        {
            WhiteBalance = whiteBalance;
        }

        void SetExposureTime(const boost::optional<HundredsNanoseconds> exposureTime)
        {
            ExposureTime = exposureTime;
        }

        void SetIsoSpeed(const boost::optional<uint32_t> isoSpeed)
        {
            IsoSpeed = isoSpeed;
        }

        void SetIsoAnalogGain(const boost::optional<float> isoAnalogGain)
        {
            IsoAnalogGain = isoAnalogGain;
        }

        void SetIsoDigitalGain(const boost::optional<float> isoDigitalGain)
        {
            IsoDigitalGain = isoDigitalGain;
        }

        boost::optional<uint32_t> GetLensPosition() const
        {
            return LensPosition;
        }

        boost::optional<uint32_t> GetWhiteBalance() const
        {
            return WhiteBalance;
        }

        boost::optional<HundredsNanoseconds> GetExposureTime() const
        {
            return ExposureTime;
        }

        boost::optional<uint32_t> GetIsoSpeed() const
        {
            return IsoSpeed;
        }

        boost::optional<float> GetIsoAnalogGain() const
        {
            return IsoAnalogGain;
        }

        boost::optional<float> GetIsoDigitalGain() const
        {
            return IsoDigitalGain;
        }

        Fields GetPopulatedFields() const
        {
            return
            {
                !!ExposureTime,
                !!WhiteBalance,
                !!LensPosition,
                !!IsoSpeed,
                !!IsoAnalogGain,
                !!IsoDigitalGain
            };
        }

        friend inline bool operator==(const CameraSettings& lhs, const CameraSettings& rhs);

    private:
        boost::optional<uint32_t>               WhiteBalance{};
        boost::optional<HundredsNanoseconds>    ExposureTime{};
        boost::optional<uint32_t>               LensPosition{};
        boost::optional<uint32_t>               IsoSpeed{};
        boost::optional<float>                  IsoAnalogGain{};
        boost::optional<float>                  IsoDigitalGain{};
    };

    inline bool operator==(const CameraSettings& lhs, const CameraSettings& rhs)
    {
        return
            lhs.ExposureTime == rhs.ExposureTime &&
            lhs.WhiteBalance == rhs.WhiteBalance &&
            lhs.LensPosition == rhs.LensPosition &&
            lhs.IsoSpeed == rhs.IsoSpeed &&
            lhs.IsoAnalogGain == rhs.IsoAnalogGain &&
            lhs.IsoDigitalGain == rhs.IsoDigitalGain;
    }

    inline bool operator!=(const CameraSettings& lhs, const CameraSettings& rhs)
    {
        return !(lhs == rhs);
    }
}
