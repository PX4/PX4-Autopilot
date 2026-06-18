if(RL_TOOLS_ENABLE_HDF5 AND RL_TOOLS_TEST_DATA_PATH)
    # RL Environment Pendulum test
    add_executable(test_rl_environments_pendulum pendulum.cpp)
    target_link_libraries(test_rl_environments_pendulum RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_environments_pendulum)
endif()
