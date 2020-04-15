//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include <TraceLoggerImpl.h>

#include <assert.h>
#include <memory>
#include <string>
#include <utility>

namespace Tracing
{
    // allows for creating potentially expensive debug code without paying for it if when trace consumers are not listening.
    template <class TProvider, uint8_t level, uint64_t keyword>
    inline bool IsEnabled()
    {
        return GetTraceLogger<TProvider>().IsEnabled<level, keyword>();
    }

    // Logs a message to the current tracing implementation
    template<class TProvider, uint8_t level = TraceLevels::Information, uint64_t keyword = TraceKeywords::None>
    inline void LogMessage(const wchar_t* message)
    {
        GetTraceLogger<TProvider>().LogMessage<level, keyword>(message);
    };

    template<class TProvider,
        uint8_t level = TraceLevels::Information,
        uint64_t keyword = TraceKeywords::None,
        typename TString>
        inline void LogMessage(const TString& message)
    {
        GetTraceLogger<TProvider>().LogMessage<level, keyword>(message);
    };

    template<class TProvider,
        uint8_t level = TraceLevels::Information,
        uint64_t keyword = TraceKeywords::None,
        typename Str1,
        typename Str2>
        inline void LogStatistic(const Str1& statistic, const Str2& category, const int64_t& value, const uint64_t& context)
    {
        GetTraceLogger<TProvider>().LogStatistic<level, keyword>(statistic, category, value, context);
    }
}
