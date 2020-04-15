//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#include "LifetimeManagerLoggerImpl.h"
#include <windows.h>
#include <winmeta.h>
#include <string>
#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.ApplicationModel.Core.h>

namespace
{

#if BUILDFLAVOR > 1
    const wchar_t* buildType = L"PROD";
#else
    const wchar_t* buildType = L"DEV/TEST";
#endif

    /*
    Code from: https://osgwiki.com/wiki/App_Launch_Measurement_Implementation_Guide
    */
#define LIFETIME_MANAGER_EVENT_WRITE(strEvent, wstrMetadata1, wstrMetadata2, dwordMetadata3, dwordMetadata4) \
    TraceLoggingWrite(mira::LifetimeManagerLoggerImpl::GetTelemetryLogger().GetProviderHandle(),\
        strEvent,\
        TraceLoggingWideString(mira::LifetimeManagerLoggerImpl::GetTelemetryLogger().GetAumId().c_str(), "AumId"),\
        TraceLoggingWideString(mira::LifetimeManagerLoggerImpl::GetTelemetryLogger().GetPackageFullName().c_str(), "PackageFullName"),\
        TraceLoggingWideString(mira::LifetimeManagerLoggerImpl::GetTelemetryLogger().GetPsmKey().c_str(), "PsmKey"),\
        TraceLoggingWideString(wstrMetadata1, "wstrMetadata1"),\
        TraceLoggingWideString(wstrMetadata2, "wstrMetadata2"),\
        TraceLoggingUInt32(dwordMetadata3, "dwordMetadata3"),\
        TraceLoggingUInt32(dwordMetadata4, "dwordMetadata4"),\
        TraceLoggingLevel(WINEVENT_LEVEL_INFO),\
        TraceLoggingOpcode(WINEVENT_OPCODE_STOP),\
        TraceLoggingKeyword(0x2000000000000),\
        TraceLoggingKeyword(WINEVENT_KEYWORD_RESPONSE_TIME))
}

namespace mira
{
    // Guid = "EF00584A-2655-462C-BC24-E7DE630E7FBF"
    TRACELOGGING_DEFINE_PROVIDER(lifetimeManagerProvider, "Microsoft.Windows.AppLifeCycle", (0xef00584a, 0x2655, 0x462c, 0xbc, 0x24, 0xe7, 0xde, 0x63, 0xe, 0x7f, 0xbf));
    LifetimeManagerLoggerImpl::LifetimeManagerLoggerImpl()
        : TraceLoggerImpl(lifetimeManagerProvider)
        , m_AppLifecycleInstrumentationAumId{ std::wstring(winrt::Windows::ApplicationModel::Package::Current().Id().FamilyName()) + std::wstring(L"!") + std::wstring(winrt::Windows::ApplicationModel::Core::CoreApplication::Id()) }
        , m_AppLifecycleInstrumentationPackageFullName{ winrt::Windows::ApplicationModel::Package::Current().Id().FullName() }
        , m_AppLifecycleInstrumentationPsmKey{ std::wstring(winrt::Windows::ApplicationModel::Package::Current().Id().FullName()) + std::wstring(L"+") + std::wstring(winrt::Windows::ApplicationModel::Core::CoreApplication::Id()) }
    {
        TraceLoggingRegister(lifetimeManagerProvider);
    }

    void LifetimeManagerLoggerImpl::LogUIResponsive(const wchar_t* entryPoint)
    {
        LIFETIME_MANAGER_EVENT_WRITE("ModernAppLaunch_UIResponsive", entryPoint, buildType, BUILDFLAVOR, 0);
    }
    void LifetimeManagerLoggerImpl::LogVisibleComplete(const wchar_t* entryPoint)
    {
        LIFETIME_MANAGER_EVENT_WRITE("ModernAppLaunch_VisibleComplete", entryPoint, buildType, BUILDFLAVOR, 0);
    }

    LifetimeManagerLoggerImpl::~LifetimeManagerLoggerImpl()
    {
        TraceLoggingUnregister(lifetimeManagerProvider);
    }
}
