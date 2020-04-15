//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include <TraceLoggerImpl.h>

namespace mira
{
    TRACELOGGING_DECLARE_PROVIDER(m_etwScanProviderHandle);

    class ScanTraceLoggerImpl : public Tracing::TraceLoggerImpl<ScanTraceLoggerImpl>
    {
        friend ScanTraceLoggerImpl& Tracing::GetTraceLogger();

    private:
        ScanTraceLoggerImpl();
        virtual ~ScanTraceLoggerImpl();
    };
}
