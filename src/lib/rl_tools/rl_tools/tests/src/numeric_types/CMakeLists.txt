if(RL_TOOLS_NUMERIC_TYPES_ENABLE_BF16)
    add_executable(test_numeric_types_type_policy type_policy.cpp)
    target_link_libraries(test_numeric_types_type_policy PRIVATE RLtools::Core GTest::gtest_main)
    gtest_discover_tests(test_numeric_types_type_policy)

    add_executable(test_numeric_types_mixed_precision_supervised_training supervised_training.cpp)
    target_link_libraries(test_numeric_types_mixed_precision_supervised_training PRIVATE RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_numeric_types_mixed_precision_supervised_training)
endif()
