// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <Plat/TimeDefinitions.h>
#include <optional>
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
                    bool IsoExternalGainValid : 1;
                    bool IsoDigitalGainValid : 1;
                } bits;
                uint32_t data;
            };
        };

        CameraSettings() = default;

        CameraSettings(
            std::optional<uint32_t> whiteBalance,
            std::optional<HundredsNanoseconds> exposureTime,
            std::optional<uint32_t> lensPosition,
            std::optional<uint32_t> isoSpeed,
            std::optional<float> isoExternalGain,
            std::optional<float> isoDigitalGain)
            : WhiteBalance{ whiteBalance },
            ExposureTime{ exposureTime },
            LensPosition{ lensPosition },
            IsoSpeed{ isoSpeed },
            IsoExternalGain{ isoExternalGain },
            IsoDigitalGain{ isoDigitalGain }
        {}

        void SetLensPosition(const std::optional<uint32_t> lensPosition)
        {
            LensPosition = lensPosition;
        }

        void SetWhiteBalance(const std::optional<uint32_t> whiteBalance)
        {
            WhiteBalance = whiteBalance;
        }

        void SetExposureTime(const std::optional<HundredsNanoseconds> exposureTime)
        {
            ExposureTime = exposureTime;
        }

        void SetIsoSpeed(const std::optional<uint32_t> isoSpeed)
        {
            IsoSpeed = isoSpeed;
        }

        void SetIsoExternalGain(const std::optional<float> isoExternalGain)
        {
            IsoExternalGain = isoExternalGain;
        }

        void SetIsoDigitalGain(const std::optional<float> isoDigitalGain)
        {
            IsoDigitalGain = isoDigitalGain;
        }

        std::optional<uint32_t> GetLensPosition() const
        {
            return LensPosition;
        }

        std::optional<uint32_t> GetWhiteBalance() const
        {
            return WhiteBalance;
        }

        std::optional<HundredsNanoseconds> GetExposureTime() const
        {
            return ExposureTime;
        }

        std::optional<uint32_t> GetIsoSpeed() const
        {
            return IsoSpeed;
        }

        std::optional<float> GetIsoExternalGain() const
        {
            return IsoExternalGain;
        }

        std::optional<float> GetIsoDigitalGain() const
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
                !!IsoExternalGain,
                !!IsoDigitalGain
            };
        }

        friend inline bool operator==(const CameraSettings& lhs, const CameraSettings& rhs);

    private:
        std::optional<uint32_t>               WhiteBalance{};
        std::optional<HundredsNanoseconds>    ExposureTime{};
        std::optional<uint32_t>               LensPosition{};
        std::optional<uint32_t>               IsoSpeed{};
        std::optional<float>                  IsoExternalGain{};
        std::optional<float>                  IsoDigitalGain{};
    };

    inline bool operator==(const CameraSettings& lhs, const CameraSettings& rhs)
    {
        return
            lhs.ExposureTime == rhs.ExposureTime &&
            lhs.WhiteBalance == rhs.WhiteBalance &&
            lhs.LensPosition == rhs.LensPosition &&
            lhs.IsoSpeed == rhs.IsoSpeed &&
            lhs.IsoExternalGain == rhs.IsoExternalGain &&
            lhs.IsoDigitalGain == rhs.IsoDigitalGain;
    }

    inline bool operator!=(const CameraSettings& lhs, const CameraSettings& rhs)
    {
        return !(lhs == rhs);
    }
}
