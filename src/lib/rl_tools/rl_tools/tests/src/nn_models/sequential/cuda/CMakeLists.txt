project(test_rl_tools_nn_models_sequential_cuda CUDA)


if(RL_TOOLS_BACKEND_ENABLE_CUDA AND RL_TOOLS_BACKEND_ENABLE_MKL)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CXX_STANDARD  17)

    add_executable(test_rl_tools_nn_models_sequential_cuda sequential_cuda.cu)
    target_link_libraries(test_rl_tools_nn_models_sequential_cuda PRIVATE rl_tools_full GTest::gtest_main)
    gtest_discover_tests(test_rl_tools_nn_models_sequential_cuda)
endif()
