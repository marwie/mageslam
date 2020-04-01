//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

namespace mira
{
    template<typename T>
    std::weak_ptr<T> make_weak(const std::shared_ptr<T>& ptr)
    {
        return ptr;
    }
}
