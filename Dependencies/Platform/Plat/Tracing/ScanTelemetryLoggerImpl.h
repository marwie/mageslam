//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include <TraceLoggerImpl.h>
#include "TelemetryKeywords.h"

namespace mira
{
    TRACELOGGING_DECLARE_PROVIDER(scanTelemetryProviderHandle);

    class ScanTelemetryLoggerImpl : public Tracing::TraceLoggerImpl<ScanTelemetryLoggerImpl>
    {
        friend ScanTelemetryLoggerImpl& Tracing::GetTraceLogger();

    private:
        ScanTelemetryLoggerImpl();
        virtual ~ScanTelemetryLoggerImpl();
    };

    // Macro to log telemetry with the correct UTC parameters
#define LOG_SCAN_TELEMETRY_EVENT(eventName, ...) \
    LOG_EVENT(mira::ScanTelemetryLoggerImpl, eventName, TraceLoggingLevel(TRACE_LEVEL_INFORMATION), TraceLoggingKeyword(MICROSOFT_KEYWORD_MEASURES), TraceLoggingBool(true, "UTCReplace_AppSessionGuid"), __VA_ARGS__)
}
