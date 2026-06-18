
if(NOT WIN32)
add_executable(
        rl_environments_pendulum_sac_arm_test
        test.cpp
)
target_link_libraries(
        rl_environments_pendulum_sac_arm_test
        PRIVATE
        RLtools::Core
)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug" AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_options(rl_environments_pendulum_sac_arm_test PRIVATE -Ofast -march=native)
endif()

add_executable(
        rl_environments_pendulum_sac_arm_test_benchmark
        test.cpp
)
target_link_libraries(
        rl_environments_pendulum_sac_arm_test_benchmark
        PRIVATE
        RLtools::Core
)
target_compile_definitions(rl_environments_pendulum_sac_arm_test_benchmark PRIVATE BENCHMARK)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug" AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_options(rl_environments_pendulum_sac_arm_test PRIVATE -Ofast -march=native)
endif()
endif()
