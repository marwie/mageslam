//
//  Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include "TimedEvent.h"
#include "arcana\macros.h"

#ifndef TRACE_LOGGER_IMPLEMENTATION
#error "In order to use scope timers you need to define which implementation to use"
#endif

/*
Helper define to time a scope
*/

#ifdef REMOVE_TIMERS
#define SCOPE_TIMER(name) (void)0
#define FUNCTION_TIMER() (void)0
#else
#define SCOPE_TIMER(name) INTERNAL_SCOPE_TIMER(name, __COUNTER__)
#define FUNCTION_TIMER() INTERNAL_FUNCTION_TIMER(__FUNCSIG__, __COUNTER__)
#endif


/*
Internal helpers to expand the counter
*/

#define EXPAND_FUNCTION_TIMER(name) L##name
#define INTERNAL_SCOPE_TIMER(name, instanceId) INTERNAL_EXPANDED_SCOPE_TIMER(WSTRINGIFY_MACRO(name), instanceId)
#define INTERNAL_FUNCTION_TIMER(name, instanceId) INTERNAL_EXPANDED_SCOPE_TIMER(EXPAND_FUNCTION_TIMER(name), instanceId)

#define INTERNAL_EXPANDED_SCOPE_TIMER(name, instanceId) \
    static ::Tracing::TimedEvent<TRACE_LOGGER_IMPLEMENTATION> staticScopeTimer ## instanceId (name); \
    ::Tracing::TimedEventRunner<TRACE_LOGGER_IMPLEMENTATION> scopeTimerRunner ## instanceId (staticScopeTimer ## instanceId)
