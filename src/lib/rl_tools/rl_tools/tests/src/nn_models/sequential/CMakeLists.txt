
if(RL_TOOLS_ENABLE_HDF5)
        add_executable(test_nn_models_sequential_persist persist.cpp)
        target_link_libraries(test_nn_models_sequential_persist PRIVATE RLtools::RLtools GTest::gtest_main)
        gtest_discover_tests(test_nn_models_sequential_persist)

endif()

add_executable(test_nn_models_sequential_persist_code persist_code.cpp)
target_link_libraries(test_nn_models_sequential_persist_code PRIVATE RLtools::RLtools GTest::gtest_main)
gtest_discover_tests(test_nn_models_sequential_persist_code)

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code.h)
        add_executable(test_nn_models_sequential_persist_code_compile persist_code_compile.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile PRIVATE RLtools::Core GTest::gtest_main)
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code_model_2.h)
        add_executable(test_nn_models_sequential_persist_code_compile_model_2 persist_code_compile_model_2.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile_model_2
                PRIVATE
                RLtools::Core
                GTest::gtest_main
        )
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile_model_2)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code_model_2_forward.h)
        add_executable(test_nn_models_sequential_persist_code_compile_model_2_forward persist_code_compile_model_2_forward.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile_model_2_forward
                PRIVATE
                RLtools::Core
                GTest::gtest_main
        )
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile_model_2_forward)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code_model_2_backward.h)
        add_executable(test_nn_models_sequential_persist_code_compile_model_2_backward persist_code_compile_model_2_backward.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile_model_2_backward
                PRIVATE
                RLtools::Core
                GTest::gtest_main
        )
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile_model_2_backward)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code_model_mlp.h)
        add_executable(test_nn_models_sequential_persist_code_compile_model_mlp persist_code_compile_model_mlp.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile_model_mlp
                PRIVATE
                RLtools::Core
                GTest::gtest_main
        )
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile_model_mlp)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code_model_mlp_forward.h)
        add_executable(test_nn_models_sequential_persist_code_compile_model_mlp_forward persist_code_compile_model_mlp_forward.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile_model_mlp_forward
                PRIVATE
                RLtools::Core
                GTest::gtest_main
        )
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile_model_mlp_forward)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code_model_sample_and_squash_forward.h)
        add_executable(test_nn_models_sequential_persist_code_compile_model_sas_forward persist_code_compile_model_sample_and_squash_forward.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile_model_sas_forward
                PRIVATE
                RLtools::Core
                GTest::gtest_main
        )
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile_model_sas_forward)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code_model_sample_and_squash_backward.h)
        add_executable(test_nn_models_sequential_persist_code_compile_model_sas_backward persist_code_compile_model_sample_and_squash_backward.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile_model_sas_backward
                PRIVATE
                RLtools::Core
                GTest::gtest_main
        )
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile_model_sas_backward)
endif()

if(EXISTS ${RL_TOOLS_TEST_DATA_PATH}/nn_models_sequential_persist_code_model_sample_and_squash_gradient.h)
        add_executable(test_nn_models_sequential_persist_code_compile_model_sas_gradient persist_code_compile_model_sample_and_squash_gradient.cpp)
        target_link_libraries(test_nn_models_sequential_persist_code_compile_model_sas_gradient
                PRIVATE
                RLtools::Core
                GTest::gtest_main
        )
        gtest_discover_tests(test_nn_models_sequential_persist_code_compile_model_sas_gradient)
endif()

add_executable(test_nn_models_sequential_d_input_benchmark d_input_benchmark.cpp)
target_link_libraries(test_nn_models_sequential_d_input_benchmark PRIVATE RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_nn_models_sequential_d_input_benchmark)

add_executable(test_nn_models_sequential_compose compose.cpp)
target_link_libraries(test_nn_models_sequential_compose PRIVATE RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_nn_models_sequential_compose)



if(RL_TOOLS_BACKEND_ENABLE_CUDA)
        add_subdirectory(cuda)
endif()