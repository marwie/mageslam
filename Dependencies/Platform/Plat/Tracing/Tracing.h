//
// Copyright (C) Microsoft Corporation. All rights reserved.
//
#pragma once

// TODO: Tracing

/*#include <Plat/Tracing/MiraTraceLoggerImpl.h>
#include <Plat/Tracing/ScopeTimer.h>
#include <TraceLogger.h>*/

namespace mira
{
    template<uint8_t level, uint64_t keyword>
    inline bool IsLoggerEnabled()
    {
        return false; //Tracing::IsEnabled<MiraTraceLoggerImpl, level, keyword>();
    }

    // Logs a message to the current tracing implementation
    // template<uint8_t level = Tracing::TraceLevels::Information, uint64_t keyword = Tracing::TraceKeywords::Debug>
    template<typename T, typename...>
    inline void LogMessage(T/*const wchar_t* message*/)
    {
        // Tracing::LogMessage<MiraTraceLoggerImpl, level, keyword>(message);
    }

    /*template<uint8_t level = Tracing::TraceLevels::Information, uint64_t keyword = Tracing::TraceKeywords::Debug, typename TString>
    inline void LogMessage(const TString& message)
    {
        LogMessage<level, keyword>(message.data());
    }

    template<typename TString>
    inline void LogWarning(const TString& message)
    {
        LogMessage<Tracing::TraceLevels::Warning>(message);
    }

    template<typename TString>
    inline void LogError(const TString& message)
    {
        LogMessage<Tracing::TraceLevels::Error>(message);
    }*/
}
