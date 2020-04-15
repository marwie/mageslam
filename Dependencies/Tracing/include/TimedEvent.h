//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include <atomic>
#include <chrono>
#include <string>

#include "RuntimeTracking.h"
#include "TraceLogger.h"

#include "Plat/Telemetry/TimingTelemetryService.h"
#include "Plat/TimeDefinitions.h"

namespace Tracing
{
    template<class TProvider,
             uint8_t level = TraceLevels::Information,
             uint64_t keyword = TraceKeywords::Perf,
             typename T,
             typename N>
    inline void LogStartEvent(const T& event_name, const N& event_count)
    {
        GetTraceLogger<TProvider>().LogTimedEventStart<level, keyword, Opcodes::Start>(event_name, event_count);
    }

    template<class TProvider,
             uint8_t level = TraceLevels::Information,
             uint64_t keyword = TraceKeywords::Perf,
             typename T,
             typename N,
             typename D>
    inline void LogStopEvent(const T& event_name, const N& event_count, const D& duration)
    {
        GetTraceLogger<TProvider>().LogTimedEventStop<level, keyword, Opcodes::End>(event_name, event_count, duration);
    }

    template<class TProvider>
    class TimedEvent
    {
        using time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;
        using precision_clock = std::chrono::high_resolution_clock;

    public:
        explicit TimedEvent(const wchar_t* eventName)
            : m_eventName{ eventName }
        {
            assert(eventName && "You shouldn't be creating an anonymous TimedEvent");
        };

        // Add a telemetry event name to send aggregated telemetry for this event.
        TimedEvent(const wchar_t* eventName, std::string telemetryName, gsl::span<const mira::TimeUnitMillisecondsF> telemetryHistogramBuckets)
            : m_eventName{ eventName }
            , m_telemetryName{ std::move(telemetryName) }
            , m_telemetryHistogramBuckets{ telemetryHistogramBuckets }
        {
        }

        TimedEvent(const TimedEvent&) = delete;
        TimedEvent& operator=(const TimedEvent&) = delete;
        TimedEvent& operator=(TimedEvent&&) & = default;
        TimedEvent(TimedEvent&&) = default;

        struct Data
        {
            uint64_t Id;
            time_point Start;
        };

        inline Data Start()
        {
            Data data;

            data.Id = m_generator.fetch_add(1);
            data.Start = precision_clock::now();

            Tracing::LogStartEvent<TProvider, TraceLevels::Verbose>(m_eventName, data.Id);

            return data;
        };

        inline void Stop(const Data& data)
        {
            std::chrono::nanoseconds duration{ precision_clock::now() - data.Start };

            // submit duration for aggregation
            Tracing::LogStopEvent<TProvider, TraceLevels::Verbose>(m_eventName, data.Id, duration.count());

            if (!m_telemetryName.empty())
            {
                if (auto timingAggregator = mira::TimingTelemetryService::Instance().lock())
                {
                    timingAggregator->AddValue(m_telemetryName, duration, m_telemetryHistogramBuckets);
                }
            }
        }

        const wchar_t* GetName() const
        {
            return m_eventName;
        }

    private:
        const wchar_t* m_eventName;
        std::atomic<uint64_t> m_generator{ 0 };
        const std::string m_telemetryName{};
        const gsl::span<const mira::TimeUnitMillisecondsF> m_telemetryHistogramBuckets{};
    };

    template<class TProvider>
    class TimedEventRunner final
    {
    public:
        explicit TimedEventRunner(TimedEvent<TProvider>& timedEvent)
            : m_timedEvent{ timedEvent }
            , m_data{ timedEvent.Start() }
            , m_oldScope{ RuntimeTracking::CurrentScope() }
        {
            RuntimeTracking::SetScope(timedEvent.GetName());
        }

        ~TimedEventRunner()
        {
            RuntimeTracking::SetScope(m_oldScope);

            m_timedEvent.Stop(m_data);
        }

        TimedEventRunner(const TimedEventRunner&) = delete;
        const TimedEventRunner& operator=(const TimedEventRunner&) = delete;

    private:
        TimedEvent<TProvider>& m_timedEvent;
        typename TimedEvent<TProvider>::Data m_data;
        const wchar_t* m_oldScope;
    };
}
