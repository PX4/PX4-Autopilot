add_executable(
        test_rl_environment_wrappers_scale_observations
        scale_observations.cpp
)
target_link_libraries(
        test_rl_environment_wrappers_scale_observations
        PRIVATE
        RLtools::RLtools
        GTest::gtest_main
)
gtest_discover_tests(test_rl_environment_wrappers_scale_observations)

