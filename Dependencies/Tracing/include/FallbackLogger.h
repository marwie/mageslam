//
//  Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include "TraceConstants.h"

namespace Mira3DScan
{
    class FallbackLogger
    {
        friend FallbackLogger& GetTraceLogger();

    private:
        FallbackLogger() = default;
        virtual ~FallbackLogger() = default;
        FallbackLogger(const FallbackLogger&) = delete;
        FallbackLogger& operator=(const FallbackLogger&) & = delete;

    public:
        template<uint8_t level, uint64_t keyword>
        bool IsEnabled()
        {}

        template<uint8_t level, uint64_t keyword>
        inline void LogMessage(const wchar_t* message)
        {}

        template<uint8_t level, uint64_t keyword, typename T>
        inline void LogMessage(const T& message)
        {}

        template<uint8_t level, uint64_t keyword, uint8_t opcode, typename N, typename C>
        inline void LogTimedEventStart(const N& event_name, const C& event_count)
        {}

        template<uint8_t level, uint64_t keyword, uint8_t opcode, typename N, typename C, typename D>
        inline void LogTimedEventStop(const N& event_name, const C& event_count, const D& duration)
        {}
    };

    inline FallbackLogger& GetTraceLogger()
    {
        static FallbackLogger logger;
        return logger;
    }
}
