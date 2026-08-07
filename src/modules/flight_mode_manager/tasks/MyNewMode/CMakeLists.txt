px4_add_library(FlightTaskMyNewMode
        FlightTaskMyNewMode.cpp
)
target_link_libraries(FlightTaskMyNewMode PUBLIC FlightTaskManualPosition)
target_include_directories(FlightTaskMyNewMode PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
