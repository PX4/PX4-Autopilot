if(RL_TOOLS_ENABLE_HDF5 AND RL_TOOLS_TEST_DATA_PATH)
    # RL Algorithm td3 test first stage mlp
    add_executable(test_rl_algorithms_td3_first_stage_mlp first_stage_mlp.cpp)
    target_link_libraries(test_rl_algorithms_td3_first_stage_mlp RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_algorithms_td3_first_stage_mlp)

    # RL Algorithm td3 test second stage MLP
    add_executable(test_rl_algorithms_td3_second_stage_mlp second_stage_mlp.cpp)
    target_link_libraries(test_rl_algorithms_td3_second_stage_mlp RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_algorithms_td3_second_stage_mlp)
endif()

# RL Algorithm td3 test full training
add_executable(test_rl_algorithms_td3_full_training_debug full_training_test.cpp)
target_link_libraries(test_rl_algorithms_td3_full_training_debug PRIVATE RLtools::Core GTest::gtest_main)
target_compile_definitions(test_rl_algorithms_td3_full_training_debug PRIVATE RL_TOOLS_TEST_RL_ALGORITHMS_TD3_FULL_TRAINING_DEBUG)
gtest_discover_tests(test_rl_algorithms_td3_full_training_debug)


# RL Algorithm td3 test full training
add_executable(test_rl_algorithms_td3_full_training full_training_test.cpp)
target_link_libraries(test_rl_algorithms_td3_full_training PRIVATE RLtools::Core GTest::gtest_main)
target_compile_options(test_rl_algorithms_td3_full_training PRIVATE -Ofast)
gtest_discover_tests(test_rl_algorithms_td3_full_training)

# RL Algorithm td3 test full training dummy dependencies compilation without any deps
add_executable(test_rl_algorithms_td3_full_training_dummy_dep_compilation full_training_dummy_dep.cpp)
target_link_libraries(test_rl_algorithms_td3_full_training_dummy_dep_compilation RLtools::Core GTest::gtest_main)
add_test(
        NAME test_rl_algorithms_td3_full_training_dummy_dep_compilation
        COMMAND test_rl_algorithms_td3_full_training_dummy_dep_compilation
)

# RL Algorithm td3 test full training dummy dependencies training by just replacing the context
add_executable(test_rl_algorithms_td3_full_training_dummy_dep_training full_training_dummy_dep.cpp)
target_compile_definitions(test_rl_algorithms_td3_full_training_dummy_dep_training PRIVATE RL_TOOLS_OPERATIONS_CPU)
target_link_libraries(test_rl_algorithms_td3_full_training_dummy_dep_training RLtools::Core GTest::gtest_main)
add_test(
        NAME test_rl_algorithms_td3_full_training_dummy_dep_training
        COMMAND test_rl_algorithms_td3_full_training_dummy_dep_training
)
