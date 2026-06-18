cmake_minimum_required(VERSION 3.16)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD  17)

add_executable(test_nn_cuda_wmma wmma.cu)
target_link_libraries(test_nn_cuda_wmma PRIVATE rl_tools_full)

add_executable(test_nn_cuda_wmma_forward forward.cu)
target_link_libraries(test_nn_cuda_wmma_forward PRIVATE rl_tools_full)
