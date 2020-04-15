//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#include "MiraTraceLoggerImpl.h"

TRACELOGGING_DEFINE_RUNTIME_TRACKING_STORAGE;

namespace mira
{
    // {922B2ED6-922E-46C1-B206-223600D2CAD5}
    TRACELOGGING_DEFINE_PROVIDER(m_etwMiraProviderHandle, "mira.tracelogger", (0x922b2ed6, 0x922e, 0x46c1, 0xb2, 0x6, 0x22, 0x36, 0x0, 0xd2, 0xca, 0xd5));

    MiraTraceLoggerImpl::MiraTraceLoggerImpl()
        : TraceLoggerImpl(m_etwMiraProviderHandle)
    {
        TraceLoggingRegister(m_etwMiraProviderHandle);
    }

    MiraTraceLoggerImpl::~MiraTraceLoggerImpl()
    {
        TraceLoggingUnregister(m_etwMiraProviderHandle);
    }
}
