if(RL_TOOLS_ENABLE_JSON)
    if(RL_TOOLS_TEST_DATA_PATH)
        add_executable(test_rl_environments_l2f_validation validation.cpp)
        target_link_libraries(test_rl_environments_l2f_validation RLtools::RLtools GTest::gtest_main)
        gtest_discover_tests(test_rl_environments_l2f_validation)
    endif()

    add_executable(test_rl_environments_l2f_import_export import_export.cpp)
    target_link_libraries(test_rl_environments_l2f_import_export RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_environments_l2f_import_export)
endif()

