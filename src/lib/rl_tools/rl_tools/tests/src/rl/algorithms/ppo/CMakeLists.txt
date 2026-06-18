# PPO tests
if(RL_TOOLS_ENABLE_TENSORBOARD)
    add_executable(test_rl_algorithms_ppo ppo.cpp)
    target_link_libraries(test_rl_algorithms_ppo PRIVATE RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_algorithms_ppo)
endif()
