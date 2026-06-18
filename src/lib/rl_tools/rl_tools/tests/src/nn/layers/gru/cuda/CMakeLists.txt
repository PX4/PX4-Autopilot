cmake_minimum_required(VERSION 3.16)

project(test_rl_tools_nn_layers_gru_cuda CUDA)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD  17)

add_executable(test_nn_layers_gru_cuda gru_cuda.cu)
target_link_libraries(test_nn_layers_gru_cuda PRIVATE rl_tools_full GTest::gtest_main)
gtest_discover_tests(test_nn_layers_gru_cuda)
