//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include <Windows.h>
#include <TraceLogger.h>
#include <TraceLoggingProvider.h>

namespace mira
{
    TRACELOGGING_DECLARE_PROVIDER(m_etwMiraProviderHandle);

    class MiraTraceLoggerImpl : public Tracing::TraceLoggerImpl<MiraTraceLoggerImpl>
    {
        friend MiraTraceLoggerImpl& Tracing::GetTraceLogger();

    private:
        MiraTraceLoggerImpl();
        virtual ~MiraTraceLoggerImpl();
    };
}
