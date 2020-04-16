set(SOURCES 
    "Plat/Telemetry/TimingTelemetryService.cpp")

add_library(Platform ${SOURCES})
target_link_libraries(Platform PUBLIC Arcana)
target_include_directories(Platform PUBLIC ".")
