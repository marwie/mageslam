//
//  Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

// clang format off
#include <Windows.h>
// clang format on

#include <TraceConstants.h>
#include <TraceLoggingActivity.h>
#include <TraceLoggingProvider.h>
#include <evntrace.h>
#include <string>

#include "..\RuntimeTracking.h"

namespace Tracing
{
    template<class TProvider>
    class TraceLoggerImpl
    {
    protected:
        TraceLoggerImpl(const TraceLoggingHProvider& handle)
            : m_etwProviderHandle(handle)
        {}

        virtual ~TraceLoggerImpl()
        {}

        TraceLoggerImpl(const TraceLoggerImpl&) = delete;
        TraceLoggerImpl& operator=(const TraceLoggerImpl&) & = delete;

        constexpr const bool convert_boolean_to_bool(BOOLEAN val) const
        {
            // false == 0, true != 0
            return val != 0;
        }

        constexpr const BOOLEAN convert_bool_to_boolean(bool val) const
        {
            // if true, return 1 (false == 0)
            return val ? 1 : 0;
        }

    public:
        const TraceLoggingHProvider& GetProviderHandle() const
        {
            return m_etwProviderHandle;
        }

        template<uint8_t level, uint64_t keyword>
        bool IsEnabled() const
        {
            return convert_boolean_to_bool(TraceLoggingProviderEnabled(m_etwProviderHandle, level, keyword));
        }

        template<uint8_t level, uint64_t keyword, typename T>
        void LogMessage(const T& message) const
        {
            OutputToConsole<level>(message, std::bool_constant<level <= TraceLevels::Information>{});

            if (IsEnabled<level, keyword>())
            {
                TraceLoggingWrite(m_etwProviderHandle,
                                  "BeihaiMessage",
                                  TraceLoggingKeyword(keyword),
                                  TraceLoggingLevel(level),
                                  TraceLoggingWideString(c_str(message), "Message"));
            }
        }

        template<uint8_t level, uint64_t keyword, typename Str1, typename Str2>
        void LogStatistic(const Str1& statistic,
                          const Str2& category,
                          const int64_t& value,
                          const uint64_t& context) const
        {
            if (IsEnabled<level, keyword>())
            {
                TraceLoggingWrite(m_etwProviderHandle,
                                  "Statistic",
                                  TraceLoggingKeyword(keyword),
                                  TraceLoggingLevel(level),
                                  TraceLoggingWideString(c_str(statistic), "Statistic", "The type of statistic"),
                                  TraceLoggingWideString(c_str(category), "Category", "category of the statistic"),
                                  TraceLoggingWideString(c_str(RuntimeTracking::CurrentScope()),
                                                         "Scope",
                                                         "The scope in which the measure was taken"),
                                  TraceLoggingInt64(value, "Value", "The value of the statistic"),
                                  TraceLoggingHexUInt64(context, "Context", "The context of the statistic"));
            }
        }

        template<uint8_t level, uint64_t keyword, uint8_t opcode, typename N>
        void LogEvent(const N& event_name) const
        {
            if (IsEnabled<level, keyword>())
            {
                TraceLoggingWrite(m_etwProviderHandle,
                                  "Perf",
                                  TraceLoggingKeyword(keyword),
                                  TraceLoggingLevel(level),
                                  TraceLoggingOpcode(opcode),
                                  TraceLoggingWideString(c_str(event_name), "EventName", "Name of the event", 0x1));
            }
        }

        template<uint8_t level, uint64_t keyword, uint8_t opcode, typename N>
        void LogTimedEventStart(const N& event_name, const uint64_t& event_count) const
        {
            if (IsEnabled<level, keyword>())
            {
                TraceLoggingWrite(m_etwProviderHandle,
                                  "TimedEvent",
                                  TraceLoggingKeyword(keyword),
                                  TraceLoggingLevel(level),
                                  TraceLoggingOpcode(opcode),
                                  TraceLoggingWideString(c_str(event_name), "EventName", "Name of the event", 0x1),
                                  TraceLoggingUInt64(event_count, "N", "Count of events", 0x2));
            }
        }

        template<uint8_t level, uint64_t keyword, uint8_t opcode, typename N>
        void LogTimedEventStop(const N& event_name, const uint64_t& event_count, const uint64_t& duration) const
        {
            if (IsEnabled<level, keyword>())
            {
                TraceLoggingWrite(
                    m_etwProviderHandle,
                    "TimedEvent",
                    TraceLoggingKeyword(keyword),
                    TraceLoggingLevel(level),
                    TraceLoggingOpcode(opcode),
                    TraceLoggingWideString(c_str(event_name), "EventName", "Name of the event", 0x1),
                    TraceLoggingUInt64(event_count, "N", "Count of events", 0x2),
                    TraceLoggingUInt64(duration, "Duration(ns)", "Time for this event in nanoseconds", 0x3));
            }
        }

            // Macro for logging an event with variable arguments.
            // traceLoggerImpl is the platform-specific trace logger implementation.
#define LOG_EVENT(traceLoggerImpl, eventName, ...)                                                                     \
    TraceLoggingWrite(::Tracing::GetTraceLogger<traceLoggerImpl>().GetProviderHandle(), eventName, __VA_ARGS__)

            // Macro for logging values without using a type-specific TraceLogging function
#define LOG_VALUE(value, ...) TraceLoggingValue(value, __VA_ARGS__)

    protected:
        const TraceLoggingHProvider& m_etwProviderHandle;

    private:
        template<typename T, typename = decltype(static_cast<const T*>(nullptr)->data())>
        static const wchar_t* c_str(const T& str)
        {
            return str.data();
        }

        static const wchar_t* c_str(const wchar_t* str)
        {
            return str;
        }

        template<uint8_t level, typename T>
        static void OutputToConsole(const T& message, std::true_type)
        {
            if (IsDebuggerPresent())
            {
                OutputDebugStringW(GetTraceLevelString(level));
                OutputDebugStringW(L": ");
                OutputDebugStringW(c_str(message));
                OutputDebugStringW(L"\n");
            }
        }

        template<uint8_t level, typename T>
        static void OutputToConsole(const T&, std::false_type)
        {}
    };

    template<class TProvider>
    TProvider& GetTraceLogger()
    {
        static TProvider logger;
        return logger;
    }
}
