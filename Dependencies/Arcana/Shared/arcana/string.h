// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <codecvt>
#include <string>
#include <locale>

#include <gsl/gsl>

namespace mira
{
    struct string_compare
    {
        using is_transparent = std::true_type;

        bool operator()(gsl::cstring_span<> a, gsl::cstring_span<> b) const
        {
            return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end());
        }

        bool operator()(gsl::czstring_span<> a, gsl::czstring_span<> b) const
        {
            return (*this)(a.as_string_span(), b.as_string_span());
        }

        bool operator()(const char* a, const char* b) const
        {
            return strcmp(a, b) < 0;
        }
    };
}
