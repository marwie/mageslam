// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#define SCOPE_TIMER(...)

#include <string>

namespace Tracing
{
    namespace TraceLevels
    {
        constexpr uint8_t Information = 0;
        constexpr uint8_t Warning = 1;
        constexpr uint8_t Verbose = 2;
    }

    namespace TraceKeywords
    {
        constexpr uint64_t None = 0;
    }
}

namespace mage
{
    template<uint8_t level = Tracing::TraceLevels::Information, uint64_t keyword = Tracing::TraceKeywords::None>
    inline void LogMessage(const char*) {}

    template<uint8_t level = Tracing::TraceLevels::Information, uint64_t keyword = Tracing::TraceKeywords::None>
    inline void LogMessage(const std::string&) {}

    template <uint8_t level = Tracing::TraceLevels::Information, uint64_t keyword = Tracing::TraceKeywords::None>
    inline void LogStatistic(const char*, const char*, const int64_t&, const uint64_t&) {}
}