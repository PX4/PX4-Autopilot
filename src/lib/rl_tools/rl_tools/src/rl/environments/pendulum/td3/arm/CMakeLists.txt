if(NOT MSVC)
add_executable(
        rl_environments_pendulum_td3_arm_test
        test.cpp
)
target_link_libraries(
        rl_environments_pendulum_td3_arm_test
        PRIVATE
        RLtools::Core
)

endif()
