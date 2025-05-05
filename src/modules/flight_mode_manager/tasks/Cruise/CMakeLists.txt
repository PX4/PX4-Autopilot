px4_add_library(FlightTaskCruise
    FlightTaskCruise.cpp
)

target_link_libraries(FlightTaskCruise PUBLIC 
    FlightTask  # Base flight task dependency
    # Add other dependencies if needed
)

target_include_directories(FlightTaskCruise PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
