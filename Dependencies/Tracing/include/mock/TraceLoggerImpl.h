//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include <TraceConstants.h>
#include <arcana/macros.h>

namespace Tracing
{
#pragma warning(push)
#pragma warning(disable : 4100)

    template<class TProvider>
    class TraceLoggerImpl
    {
    protected:
        TraceLoggerImpl()
        {}

        virtual ~TraceLoggerImpl()
        {}

        TraceLoggerImpl(const TraceLoggerImpl&) = delete;
        TraceLoggerImpl& operator=(const TraceLoggerImpl&) & = delete;

        mutable int m_logMessageCount = 0;

    public:
        int GetLogMessageCount()
        {
            return m_logMessageCount;
        }

        template<uint8_t level, uint64_t keyword>
        bool IsEnabled() const
        {}

        template<uint8_t level, uint64_t keyword, typename T>
        void LogMessage(const T& message) const
        {
            m_logMessageCount++;
        }

        template<uint8_t level, uint64_t keyword, typename Str1, typename Str2>
        void LogStatistic(const Str1& statistic,
            const Str2& category,
            const int64_t& value,
            const uint64_t& context) const
        {}

        template<uint8_t level, uint64_t keyword, uint8_t opcode, typename N>
        void LogEvent(const N& event_name) const
        {}

        template<uint8_t level, uint64_t keyword, uint8_t opcode, typename N>
        void LogTimedEventStart(const N& event_name, const uint64_t& event_count) const
        {}

        template<uint8_t level, uint64_t keyword, uint8_t opcode, typename N>
        void LogTimedEventStop(const N& event_name, const uint64_t& event_count, const uint64_t& duration) const
        {}

        // Macro for logging an event with variable arguments.
        // traceLoggerImpl is the platform-specific trace logger implementation.
#define LOG_EVENT(traceLoggerImpl, eventName, ...) __VA_ARGS__

        // Macro for logging values without using a type-specific TraceLogging function
#define LOG_VALUE(value, ...) UNUSED(value)
    };

    template<class TProvider>
    TProvider& GetTraceLogger()
    {
        static TProvider logger;
        return logger;
    }

#pragma warning(pop)
}
