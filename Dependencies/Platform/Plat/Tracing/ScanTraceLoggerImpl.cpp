//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#include "ScanTraceLoggerImpl.h"

namespace mira
{
    //{7C1F984F-D3F4-4FCE-A407-BD77E88A3B9E}
    TRACELOGGING_DEFINE_PROVIDER(m_etwScanProviderHandle, "Mira.Scan.Engine", (0x7c1f984f, 0xd3f4, 0x4fce, 0xa4, 0x7, 0xbd, 0x77, 0xe8, 0x8a, 0x3b, 0x9e));

    ScanTraceLoggerImpl::ScanTraceLoggerImpl()
        : TraceLoggerImpl(m_etwScanProviderHandle)
    {
        TraceLoggingRegister(m_etwScanProviderHandle);
    }

    ScanTraceLoggerImpl::~ScanTraceLoggerImpl()
    {
        TraceLoggingUnregister(m_etwScanProviderHandle);
    }
}
