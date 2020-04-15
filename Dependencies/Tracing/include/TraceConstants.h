//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include <cstdint>

namespace Tracing
{
    enum TraceKeywords : uint64_t
    {
        None                        = 0,
        Debug                       = 0x1,
        Perf                        = 0x2,
        TrackingPipeline            = 0x4,
        ReconstructionPipeline      = 0x8,
        CameraDevice                = 0x10,
    };

    namespace TraceLevel
    {
        const uint8_t error         = 1;
        const uint8_t warning       = 2;
        const uint8_t info          = 3;
        const uint8_t verbose       = 4;
    }

    enum TraceLevels : uint8_t
    {
        MinTraceLevel = TraceLevel::error,
        Error = TraceLevel::error,
        Warning = TraceLevel::warning,
        Information = TraceLevel::info,
        Verbose = TraceLevel::verbose,
        MaxTraceLevel = TraceLevel::verbose
    };

    namespace Opcode
    {
        const uint8_t info = 0;
        const uint8_t start = 1;
        const uint8_t stop = 2;
    }

    enum Opcodes : uint8_t
    {
        MinOpcode = Opcode::info,
        Info = Opcode::info,
        Start = Opcode::start,
        End = Opcode::stop,
        MaxOpcode = Opcode::stop
    };

    inline const wchar_t* GetTraceLevelString(uint8_t traceLevel)
    {
        switch (traceLevel)
        {
        case TraceLevels::Error:
            return L"Error";
        case TraceLevels::Warning:
            return L"Warning";
        case TraceLevels::Information:
            return L"Information";
        case TraceLevels::Verbose:
            return L"Verbose";
        default:
            return L"Unknown";
        }
    }
}
