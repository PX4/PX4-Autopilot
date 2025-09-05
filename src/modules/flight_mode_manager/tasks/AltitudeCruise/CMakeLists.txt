px4_add_library(FlightTaskAltitudeCruise
        FlightTaskAltitudeCruise.cpp
)

target_link_libraries(FlightTaskAltitudeCruise PUBLIC FlightTaskManualAltitude)
target_include_directories(FlightTaskAltitudeCruise PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
