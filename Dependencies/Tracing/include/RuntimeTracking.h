//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

namespace Tracing
{
    template<typename>
    class TimedEventRunner;

    class RuntimeTracking
    {
    public:
        RuntimeTracking() = delete;

        static const wchar_t* CurrentScope()
        {
            return m_scope;
        }

    private:
        template<typename>
        friend class ::Tracing::TimedEventRunner;

        static void SetScope(const wchar_t* scope)
        {
            m_scope = scope;
        }

        static thread_local const wchar_t* m_scope;
    };
}

#define TRACELOGGING_DEFINE_RUNTIME_TRACKING_STORAGE                                                                   \
    thread_local const wchar_t* ::Tracing::RuntimeTracking::m_scope = nullptr