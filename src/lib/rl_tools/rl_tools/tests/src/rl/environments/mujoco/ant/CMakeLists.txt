if(RL_TOOLS_ENABLE_HDF5)
    add_executable(test_rl_environments_mujoco_ant ant.cpp)
    target_link_libraries(test_rl_environments_mujoco_ant PRIVATE RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_environments_mujoco_ant)
endif()

add_executable(test_rl_environments_mujoco_ant_td3_test training_td3_test.cpp)
target_link_libraries(test_rl_environments_mujoco_ant_td3_test PRIVATE RLtools::RLtools GTest::gtest_main)
gtest_discover_tests(test_rl_environments_mujoco_ant_td3_test)
target_compile_definitions(test_rl_environments_mujoco_ant_td3_test PRIVATE -DRL_TOOLS_TEST_RL_ENVIRONMENTS_MUJOCO_ANT_TRAINING_TD3_TEST)


if(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ENABLE_UI)
    add_executable(test_rl_environments_mujoco_ant_ui ui.cpp)
    target_link_libraries(test_rl_environments_mujoco_ant_ui PRIVATE RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_environments_mujoco_ant_ui)
endif()


if(RL_TOOLS_ENABLE_HDF5 AND RL_TOOLS_ENABLE_TENSORBOARD)
    add_executable(test_rl_environments_mujoco_ant_ppo_test training_ppo_test.cpp)
    target_link_libraries(test_rl_environments_mujoco_ant_ppo_test PRIVATE RLtools::RLtools GTest::gtest_main HighFive tensorboard_logger mujoco)
    gtest_discover_tests(test_rl_environments_mujoco_ant_ppo_test)
endif()


