//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#include "ScanTelemetryLoggerImpl.h"

namespace mira
{
    // Define the telemetry event etw provider with the UTC telemetry OptionGroup
    // Proider Guid: {676d034e-963c-51f0-8f32-cafcc8b2f23f}
    TRACELOGGING_DEFINE_PROVIDER(scanTelemetryProviderHandle, "Microsoft.Beihai.Core.ScanWindowsApp", (0x676d034e, 0x963c, 0x51f0, 0x8f, 0x32, 0xca, 0xfc, 0xc8, 0xb2, 0xf2, 0x3f), TraceLoggingOptionMicrosoftTelemetry());

    ScanTelemetryLoggerImpl::ScanTelemetryLoggerImpl()
        : TraceLoggerImpl(scanTelemetryProviderHandle)
    {
        TraceLoggingRegister(scanTelemetryProviderHandle);
    }

    ScanTelemetryLoggerImpl::~ScanTelemetryLoggerImpl()
    {
        TraceLoggingUnregister(scanTelemetryProviderHandle);
    }
}
