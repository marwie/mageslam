#pragma once

#include "MAGESlamTraceLoggerImpl.h"
#include "ScopeTimer.h"

namespace mage
{
    template<uint8_t level = Tracing::TraceLevels::Information, uint64_t keyword = Tracing::TraceKeywords::None>
    inline void LogMessage(const wchar_t* message)
    {
        Tracing::LogMessage<MAGESlamTraceLoggerImpl, level, keyword>(message);
    }

    template<uint8_t level = Tracing::TraceLevels::Information, uint64_t keyword = Tracing::TraceKeywords::None>
    inline void LogMessage(const std::wstring& message)
    {
        Tracing::LogMessage<MAGESlamTraceLoggerImpl, level, keyword>(message);
    }

    template <uint8_t level = Tracing::TraceLevels::Information, uint64_t keyword = Tracing::TraceKeywords::None>
    inline void LogStatistic(const wchar_t* statistic, const wchar_t* category, const int64_t& value, const uint64_t& context)
    {
        Tracing::LogStatistic<MAGESlamTraceLoggerImpl, level, keyword>(statistic, category, value, context);
    }
}