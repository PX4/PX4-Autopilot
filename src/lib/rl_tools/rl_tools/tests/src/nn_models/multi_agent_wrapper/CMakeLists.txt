add_executable(test_nn_models_multi_agent_wrapper_persist_code multi_agent_wrapper_persist_code.cpp)
target_link_libraries(test_nn_models_multi_agent_wrapper_persist_code
        PRIVATE
        RLtools::Core
        GTest::gtest_main
)
gtest_discover_tests(test_nn_models_multi_agent_wrapper_persist_code)

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_multi_agent_wrapper_persist_code.h)
    add_executable(test_nn_models_multi_agent_wrapper_persist_code_compile multi_agent_wrapper_persist_code_compile.cpp)
    target_link_libraries(test_nn_models_multi_agent_wrapper_persist_code_compile
            PRIVATE
            RLtools::Core
            GTest::gtest_main
    )
    gtest_discover_tests(test_nn_models_multi_agent_wrapper_persist_code_compile)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_multi_agent_wrapper_persist_code_backward.h)
    add_executable(test_nn_models_multi_agent_wrapper_persist_code_compile_backward multi_agent_wrapper_persist_code_compile_backward.cpp)
    target_link_libraries(test_nn_models_multi_agent_wrapper_persist_code_compile_backward
            PRIVATE
            RLtools::Core
            GTest::gtest_main
    )
    gtest_discover_tests(test_nn_models_multi_agent_wrapper_persist_code_compile_backward)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_multi_agent_wrapper_persist_code_forward.h)
    add_executable(test_nn_models_multi_agent_wrapper_persist_code_compile_forward multi_agent_wrapper_persist_code_compile_forward.cpp)
    target_link_libraries(test_nn_models_multi_agent_wrapper_persist_code_compile_forward
            PRIVATE
            RLtools::Core
            GTest::gtest_main
    )
    gtest_discover_tests(test_nn_models_multi_agent_wrapper_persist_code_compile_forward)
endif()
