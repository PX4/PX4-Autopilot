px4_add_library(FlightTaskAltitudeVoyager
        FlightTaskAltitudeVoyager.cpp
)

target_link_libraries(FlightTaskAltitudeVoyager PUBLIC FlightTaskManualAltitude)
target_include_directories(FlightTaskAltitudeVoyager PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
