//-----------------------------------------------------------------------------
//
//  File: CvMarkersObj.h
//
//  Copyright (C) 2011 Microsoft Corporation
//  All rights reserved.
//
//  ConcurrencyVisualizer markers API for C++ language.
//
//-----------------------------------------------------------------------------
#pragma once
#include <tchar.h>
#include <stdexcept>
#include <utils\cvmarkers.h>

/// <summary>
///     The <c>Concurrency</c> namespace provides classes and functions that give you access to the Concurrency Runtime,
///     a concurrent programming framework for C++. For more information, see <see cref="Concurrency Runtime"/>.
/// </summary>
namespace Concurrency
{
/// <summary>
///     The <c>diagnostics</c> namespace provides functionality for emitting PPA markers
/// </summary>
namespace diagnostic
{
    enum marker_importance {
        critical_importance = CvImportanceCritical,
        high_importance = CvImportanceHigh,
        normal_importance = CvImportanceNormal,
        low_importance = CvImportanceLow
    };

    /// <summary>
    /// marker_series represents PPA marker series and exposes functionality for emitting spans, flags and messages
    /// </summary>
    class marker_series
    {
        friend class span;
        PCV_MARKERSERIES _Series;
        PCV_PROVIDER _Provider;

        PCV_MARKERSERIES _GetSeries() const
        {
            return _Series;
        }

        void _Init_marker_series(_In_ LPCTSTR _SeriesName)
        {
            HRESULT error = CvInitProvider(&CvDefaultProviderGuid, &_Provider);
            if (FAILED(error))
            {
                throw std::runtime_error("provider initialization error");
            }

            error = CvCreateMarkerSeries(_Provider, _SeriesName, &_Series);
            if (FAILED(error))
            {
                throw std::runtime_error("series initialization error");
            }
        }

    public:
        /// <summary>
        /// Create a default instance of marker_series with default provider and series objects
        /// </summary>
        marker_series()
        {
            _Init_marker_series(_T(""));
        }

        /// <summary>
        /// Create an instance of marker_series with default provider and series objects
        /// </summary>
        marker_series(_In_ LPCTSTR _SeriesName)
        {
            _Init_marker_series(_SeriesName);
        }

        /// <summary>
        /// Determines if any session has enabled the provider.
        /// </summary>
        bool is_enabled()
        {
            return CvIsEnabled(_Provider) == S_OK;
        }

        /// <summary>
        /// Determines if any session has enabled the provider with specified level and category.
        /// </summary>
        bool is_enabled(marker_importance _Importance, int _Category)
        {
            return CvIsEnabledEx(_Provider, (CV_IMPORTANCE)_Importance, _Category) == S_OK;
        }

        /// <summary>
        /// Emit a flag marker using default importance and default category
        /// </summary>
        void write_flag(_In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteFlagExV(_Series, CvImportanceNormal, CvDefaultFlagCategory, _Format, _Args);
            va_end(_Args);
        }

        /// <summary>
        /// Emit a flag marker using given importance and default category
        /// </summary>
        void write_flag(marker_importance _Importance, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteFlagExV(_Series, (CV_IMPORTANCE)_Importance, CvDefaultFlagCategory, _Format, _Args);
            va_end(_Args);
        }

        /// <summary>
        /// Emit a flag marker using default importance and given category
        /// </summary>
        void write_flag(int _Category, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteFlagExV(_Series, CvImportanceNormal, _Category, _Format, _Args);
            va_end(_Args);
        }

        /// <summary>
        /// Emit a flag marker using given importance and category
        /// </summary>
        void write_flag(marker_importance _Importance, int _Category, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteFlagExV(_Series, (CV_IMPORTANCE)_Importance, _Category, _Format, _Args);
            va_end(_Args);
        }

        /// <summary>
        /// Emit a alert marker which is a flag with critical importance and alert category.
        /// </summary>
        void write_alert(_In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteAlertV(_Series, _Format, _Args);
            va_end(_Args);
        }

        /// <summary>
        /// Emit a message marker using default importance and default category
        /// </summary>
        void write_message(_In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteMessageExV(_Series, CvImportanceNormal, CvDefaultFlagCategory, _Format, _Args);
            va_end(_Args);
        }

        /// <summary>
        /// Emit a message marker using give importance and default category
        /// </summary>
        void write_message(marker_importance _Importance, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteMessageExV(_Series, (CV_IMPORTANCE)_Importance, CvDefaultFlagCategory, _Format, _Args);
            va_end(_Args);
        }

        /// <summary>
        /// Emit a message marker using default importance and given category
        /// </summary>
        void write_message(int _Category, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteMessageExV(_Series, CvImportanceNormal, _Category, _Format, _Args);
            va_end(_Args);
        }

        /// <summary>
        /// Emit a message marker using given importance and given category
        /// </summary>
        void write_message(marker_importance _Importance, int _Category, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvWriteMessageExV(_Series, (CV_IMPORTANCE)_Importance, _Category, _Format, _Args);
            va_end(_Args);
        }
    };

    /// <summary>
    /// span is a RAII class that starts the span object in the constructor and leaves it in the destructor
    /// </summary>
    class span
    {
        PCV_SPAN _Span;
    public:
        span(marker_series _Series, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvEnterSpanExV(_Series._GetSeries(), CvImportanceNormal, CvDefaultFlagCategory, &_Span, _Format, _Args);
            va_end(_Args);
        }

        span(marker_series _Series, marker_importance _Importance, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvEnterSpanExV(_Series._GetSeries(), (CV_IMPORTANCE)_Importance, CvDefaultFlagCategory, &_Span, _Format, _Args);
            va_end(_Args);
        }

        span(marker_series _Series, int _Category, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvEnterSpanExV(_Series._GetSeries(), CvImportanceNormal, _Category, &_Span, _Format, _Args);
            va_end(_Args);
        }

        span(marker_series _Series, marker_importance _Importance, int _Category, _In_ LPCTSTR _Format, ...)
        {
            va_list _Args;
            va_start(_Args, _Format);
            CvEnterSpanExV(_Series._GetSeries(), (CV_IMPORTANCE)_Importance, _Category, &_Span, _Format, _Args);
            va_end(_Args);
        }

        ~span()
        {
            CvLeaveSpan(_Span);
        }
    };
}
}
