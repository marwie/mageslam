//
// Copyright (C) Microsoft Corporation. All rights reserved.
//

#pragma once

#include "SensorSample.h"

namespace mage
{
    bool LegacyDebugRead(std::istream& inputFileStream, const uint64_t& binVersion, SensorSample& outSample);
}