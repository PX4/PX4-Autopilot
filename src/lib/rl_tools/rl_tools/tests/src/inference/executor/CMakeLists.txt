if(RL_TOOLS_TEST_DATA_PATH)
    add_executable(test_inference_executor test_inference_executor.cpp)
    target_link_libraries(test_inference_executor PRIVATE RLtools::Core GTest::gtest_main)
    gtest_discover_tests(test_inference_executor)

    add_executable(test_inference_l2f_c_interface test_inference_l2f_c_interface.cpp l2f_c_interface.cpp)
    target_link_libraries(test_inference_l2f_c_interface PRIVATE RLtools::Core GTest::gtest_main)
    gtest_discover_tests(test_inference_l2f_c_interface)

    add_executable(test_inference_l2f_c_interface_write_policy write_example_policy.cpp)
    target_link_libraries(test_inference_l2f_c_interface_write_policy PRIVATE RLtools::Core GTest::gtest_main)
    gtest_discover_tests(test_inference_l2f_c_interface_write_policy)
endif()
