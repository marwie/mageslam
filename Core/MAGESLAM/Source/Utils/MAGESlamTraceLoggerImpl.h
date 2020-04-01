//
// Copyright (C) Microsoft Corporation. All rights reserved.
//
#pragma once

#include <TraceLogger.h>

namespace mage
{
    TRACELOGGING_DECLARE_PROVIDER(m_etwScanProviderHandle);

    class MAGESlamTraceLoggerImpl : public Tracing::TraceLoggerImpl<MAGESlamTraceLoggerImpl>
    {
        friend MAGESlamTraceLoggerImpl& Tracing::GetTraceLogger();

    private:
        MAGESlamTraceLoggerImpl();
        virtual ~MAGESlamTraceLoggerImpl();
    };
}