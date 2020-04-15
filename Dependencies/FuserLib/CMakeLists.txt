set(SOURCES
    "AnalogConversions.h"
    "FuserLib.h"
    "LegacySerialization.cpp"
    "LegacySerialization.h"
    "pch.cpp"
    "pch.h"
    "SensorFilter.cpp"
    "SensorFilter.h"
    "SensorSample.cpp"
    "SensorSample.h"
    "SensorSampleQueue.cpp"
    "SensorSampleQueue.h"
    "SensorSampleSerializable.h"
    "SimpleIMUFilter.cpp"
    "SimpleIMUFilter.h"
    "TimeUnits.h")


add_library(FuserLib ${SOURCES})

target_include_directories(FuserLib
    PRIVATE "${CMAKE_SOURCE_DIR}/Core/MAGESLAM/Source" # This is...horrific.
    PRIVATE ".." # ... Ugh.
    INTERFACE ".")

target_link_libraries(FuserLib
    PUBLIC cereal
    PUBLIC Arcana)
