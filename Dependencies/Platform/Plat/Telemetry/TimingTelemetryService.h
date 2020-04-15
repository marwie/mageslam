//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include "Plat/TimeDefinitions.h"
#include <arcana/experimental/array.h>
#include <arcana/functional/inplace_function.h>
#include <gsl/gsl>
#include <vector>

namespace mira
{
    namespace HistogramBuckets
    {
        using namespace std::chrono_literals;
        static constexpr auto None = gsl::span<TimeUnitMillisecondsF>{};
        static constexpr auto PerFrameWork = mira::make_array<TimeUnitMillisecondsF>(16.7ms, 20ms, 33.3ms, 40ms, 100ms, 1000ms, 1h);
        static constexpr auto SubFrameDuration = mira::make_array<TimeUnitMillisecondsF>(0.2ms, 1ms, 5ms, 10ms, 20ms, 30ms, 40ms, 1h);
        static constexpr auto RegularTickRate = mira::make_array<TimeUnitMillisecondsF>(16.2ms, 17.2ms, 32.8ms, 33.8ms, 1h);
    };

    // An object that can be called from anywhere in the code to create timing stats and
    // provide them in a format ready to send to telemetry.
    //
    // AddValue is thread-safe and can be called from any thread.
    // The typical way to use this API is by creating a TimedEvent with the special
    // instrumentation constructor where a telemetry tag is provided.
    class TimingTelemetryPhase;

    class TimingTelemetryService
    {
    public:
        struct AggregatedResult
        {
            std::string Name{};
            std::string PhaseName{};
            TimeUnitMillisecondsF SamplePeriod{};
            int NumSamples{};
            TimeUnitMillisecondsF Min{};
            TimeUnitMillisecondsF Max{};
            TimeUnitMillisecondsF Mean{};
            TimeUnitMillisecondsF Median{};
            TimeUnitMillisecondsF StandardDeviation{};
            std::vector<std::pair<TimeUnitMillisecondsF, int>> OccurrenceHistogram{};

            // Move-only.
            AggregatedResult() = default;
            AggregatedResult(const AggregatedResult&) = delete;
            AggregatedResult& operator=(const AggregatedResult&) = delete;
            AggregatedResult(AggregatedResult&&) = default;
            AggregatedResult& operator=(AggregatedResult&&) = default;
        };

        static void BeginPhase(std::weak_ptr<TimingTelemetryPhase> phaseInstance);
        static std::weak_ptr<TimingTelemetryPhase> Instance();

    private:
        static std::weak_ptr<TimingTelemetryPhase> s_phaseInstance;
    };

    class TimingTelemetryPhase
    {
    public:
        using ResultReadyFunction = std::function<void(const TimingTelemetryService::AggregatedResult& results)>;

        TimingTelemetryPhase(std::string phaseName, ResultReadyFunction resultReadyFunction, TimeUnitMillisecondsF timeBetweenAggregations);
        ~TimingTelemetryPhase();

        void AddValue(const std::string& measureName, TimeUnitMillisecondsF duration, gsl::span<const TimeUnitMillisecondsF> histogramBuckets = HistogramBuckets::None);

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
