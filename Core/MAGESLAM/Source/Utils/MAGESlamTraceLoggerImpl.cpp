//
// Copyright (C) Microsoft Corporation. All rights reserved.
//
#include "MAGESlamTraceLoggerImpl.h"

namespace mage
{
    // {dbccb482-0db9-4ae8-8cc5-6ae76c448309}
    TRACELOGGING_DEFINE_PROVIDER(m_etwScanProviderHandle, "mage.MAGESLaM", (0xdbccb482, 0x0db9, 0x4ae8, 0x8c, 0xc5, 0x6a, 0xe7, 0x6c, 0x44, 0x83, 0x09));

    MAGESlamTraceLoggerImpl::MAGESlamTraceLoggerImpl()
        : TraceLoggerImpl(m_etwScanProviderHandle)
    {
        TraceLoggingRegister(m_etwScanProviderHandle);
    }

    MAGESlamTraceLoggerImpl::~MAGESlamTraceLoggerImpl()
    {
        TraceLoggingUnregister(m_etwScanProviderHandle);
    }
}