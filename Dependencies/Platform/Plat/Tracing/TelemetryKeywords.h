//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include <TraceLoggerImpl.h>

namespace mira
{
    // Defining the provider with the following OptionGroup enables its events to fire as UTC telemetry
#define TraceLoggingOptionMicrosoftTelemetry() \
            TraceLoggingOptionGroup(0x4f50731a, 0x89cf, 0x4782, 0xb3, 0xe0, 0xdc, 0xe8, 0xc9, 0x4, 0x76, 0xba)

                // UTC Telemetry Keywords. MICROSOFT_KEYWORD_MEASURES should be used in most cases.
#define MICROSOFT_KEYWORD_CRITICAL_DATA 0x0000800000000000 // Bit 47
#define MICROSOFT_KEYWORD_MEASURES      0x0000400000000000 // Bit 46
#define MICROSOFT_KEYWORD_TELEMETRY     0x0000200000000000 // Bit 45
}
