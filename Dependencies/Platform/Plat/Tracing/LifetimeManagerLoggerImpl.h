//
// Copyright (C) Microsoft Corporation. All rights reserved.
//
#pragma once

#include <TraceLoggerImpl.h>
#include <string>

namespace mira
{

    class LifetimeManagerLoggerImpl final : public Tracing::TraceLoggerImpl<LifetimeManagerLoggerImpl>
    {
        friend LifetimeManagerLoggerImpl& Tracing::GetTraceLogger();

    public:
        static LifetimeManagerLoggerImpl& GetTelemetryLogger() { return Tracing::GetTraceLogger<mira::LifetimeManagerLoggerImpl>(); };
        void LogUIResponsive(const wchar_t* entryPoint);
        void LogVisibleComplete(const wchar_t* entryPoint);
        std::wstring const GetAumId() const { return m_AppLifecycleInstrumentationAumId; }
        std::wstring const GetPackageFullName() const { return m_AppLifecycleInstrumentationPackageFullName; }
        std::wstring const GetPsmKey() const { return m_AppLifecycleInstrumentationPsmKey; }
        ~LifetimeManagerLoggerImpl();

    private:
        LifetimeManagerLoggerImpl();
        const std::wstring m_AppLifecycleInstrumentationAumId;
        const std::wstring m_AppLifecycleInstrumentationPackageFullName;
        const std::wstring m_AppLifecycleInstrumentationPsmKey;
    };

}
