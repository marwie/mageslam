//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#include "TimingTelemetryService.h"

// TODO: Tracing

#include "Plat/MiraAssert.h"
#include <arcana/math.h>
#include <arcana/utils/algorithm.h>
#include <mutex>
#include <unordered_map>

using namespace std::chrono_literals;

namespace mira
{
    namespace
    {
        constexpr auto MaxMeasures = size_t{ 10 };

        TimingTelemetryService::AggregatedResult aggregate(gsl::span<TimeUnitMillisecondsF> samples, gsl::span<const TimeUnitMillisecondsF> histogramBuckets)
        {
            MIRA_EXPECTS(!samples.empty());

            TimingTelemetryService::AggregatedResult result;
            result.NumSamples = gsl::narrow_cast<int>(samples.size());
            result.Min = *std::min_element(begin(samples), end(samples));
            result.Max = *std::max_element(begin(samples), end(samples));
            result.Mean = mira::mean<TimeUnitMillisecondsF>(begin(samples), end(samples));
            result.Median = mira::median<TimeUnitMillisecondsF>(begin(samples), end(samples)); // Note: Re-orders collection.

            if (samples.size() > 1)
            {
                result.StandardDeviation = { mira::standard_deviation<float>(begin(samples), end(samples), [](auto sample) { return sample.count(); }) };
            }

            if (!histogramBuckets.empty())
            {
                // Write histogram labels.
                std::transform(begin(histogramBuckets), end(histogramBuckets), back_inserter(result.OccurrenceHistogram), [](auto label) { return std::make_pair(label, 0); });

                // Write histogram values.
                for (auto sample : samples)
                {
                    auto bucket = std::distance(begin(histogramBuckets), std::lower_bound(begin(histogramBuckets), end(histogramBuckets), sample));
                    result.OccurrenceHistogram[bucket].second++;
                }
            }

            return result;
        }
    }

    std::weak_ptr<TimingTelemetryPhase> TimingTelemetryService::s_phaseInstance;

    class TimingTelemetryPhase::Impl
    {
    public:
        Impl(std::string phaseName, ResultReadyFunction resultReadyFunction, const TimeUnitMillisecondsF timeBetweenAggregations)
            : m_phaseName{ std::move(phaseName) }
            , m_resultReadyFunction{ std::move(resultReadyFunction) }
            , m_timeBetweenAggregations{ timeBetweenAggregations }
        {
        }

        ~Impl()
        {
            // Send all in-flight measures.
            for (auto& measureKeyValue : m_measures)
            {
                auto name = measureKeyValue.first;
                auto& measure = measureKeyValue.second;

                std::unique_lock<std::mutex> samplesLock{ measure.SamplesMutex };
                if (!measure.Samples.empty())
                {
                    const auto samplePeriod = std::chrono::steady_clock::now() - measure.SamplePeriodBegin;
                    std::vector<TimeUnitMillisecondsF> samples;
                    std::swap(samples, measure.Samples);
                    samplesLock.unlock();

                    AggregateAndSend(name, samples, samplePeriod, measure.HistogramBuckets);
                }
            }
        }

        void AddValue(const std::string& name, const TimeUnitMillisecondsF duration, gsl::span<const TimeUnitMillisecondsF> histogramBuckets)
        {
            auto& measure = GetOrInsertMeasure(name);
            MIRA_ASSERT(m_measures.bucket_count() <= MaxMeasures, "A lot of different timing measures are being sent to telemetry. Reduce measures, or incease max number of measures.");

            std::unique_lock<std::mutex> samplesLock{ measure.SamplesMutex };
            if (measure.Samples.empty())
            {
                measure.SamplePeriodBegin = std::chrono::steady_clock::now();
                measure.HistogramBuckets = histogramBuckets;
            }

            measure.Samples.push_back(duration);

            // Aggregate and send if necessary.
            const auto samplePeriod = std::chrono::steady_clock::now() - measure.SamplePeriodBegin;
            if (samplePeriod > m_timeBetweenAggregations)
            {
                std::vector<TimeUnitMillisecondsF> samples;
                std::swap(samples, measure.Samples);
                samplesLock.unlock();

                AggregateAndSend(name, samples, samplePeriod, histogramBuckets);
            }
        }

    private:
        struct Measure
        {
            std::chrono::steady_clock::time_point SamplePeriodBegin;
            std::vector<TimeUnitMillisecondsF> Samples;
            std::mutex SamplesMutex;
            gsl::span<const TimeUnitMillisecondsF> HistogramBuckets;
        };

        Measure& GetOrInsertMeasure(const std::string& name)
        {
            std::lock_guard<std::mutex> lock{ m_measuresInsertMutex };
            return m_measures[name];
        }

        void AggregateAndSend(
            const std::string& name,
            gsl::span<TimeUnitMillisecondsF> samples,
            TimeUnitMillisecondsF samplePeriod,
            gsl::span<const TimeUnitMillisecondsF> histogramBuckets
        )
        {
            auto result = aggregate(samples, histogramBuckets);
            result.Name = name;
            result.PhaseName = m_phaseName;
            result.SamplePeriod = samplePeriod;
            m_resultReadyFunction(result);
        }

        const std::string m_phaseName;
        const TimeUnitMillisecondsF m_timeBetweenAggregations;
        const ResultReadyFunction m_resultReadyFunction;
        std::unordered_map<std::string, Measure> m_measures{};
        std::mutex m_measuresInsertMutex{};
    };

    void TimingTelemetryService::BeginPhase(std::weak_ptr<TimingTelemetryPhase> instance)
    {
        s_phaseInstance = std::move(instance);
    }

    std::weak_ptr<TimingTelemetryPhase> TimingTelemetryService::Instance()
    {
        return s_phaseInstance;
    }

    TimingTelemetryPhase::TimingTelemetryPhase(std::string phaseName, ResultReadyFunction resultReadyFunction, TimeUnitMillisecondsF timeBetweenAggregations)
        : m_impl{ std::make_unique<Impl>(std::move(phaseName), std::move(resultReadyFunction), timeBetweenAggregations) }
    {
    }

    TimingTelemetryPhase::~TimingTelemetryPhase() = default;

    void TimingTelemetryPhase::AddValue(const std::string& measureName, const TimeUnitMillisecondsF duration, const gsl::span<const TimeUnitMillisecondsF> histogramBuckets)
    {
        m_impl->AddValue(measureName, duration, histogramBuckets);
    }
}
