cmake_minimum_required(VERSION 3.16)

project(test_rl_tools_rl_cuda CUDA)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD  17)

function(RL_TOOLS_TESTS_RL_CUDA_ADD_DEBUG_DEFINITIONS TARGET)
    #    if(RL_TOOLS_DEBUG)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#        target_compile_definitions(${TARGET} PRIVATE -DRL_TOOLS_DEBUG_CONTAINER_CHECK_BOUNDS)
#        target_compile_definitions(${TARGET} PRIVATE -DRL_TOOLS_DEBUG_CONTAINER_CHECK_MALLOC)
        target_compile_definitions(${TARGET} PRIVATE -DRL_TOOLS_DEBUG_CONTAINER_MALLOC_INIT_NAN)
        target_compile_definitions(${TARGET} PRIVATE -DRL_TOOLS_DEBUG_RL_COMPONENTS_OFF_POLICY_RUNNER_CHECK_INIT)
        target_compile_definitions(${TARGET} PRIVATE -DRL_TOOLS_DEBUG_DEVICE_CUDA_CHECK_INIT)
        target_compile_definitions(${TARGET} PRIVATE -DRL_TOOLS_DEBUG_DEVICE_CUDA_SYNCHRONIZE_STATUS_CHECK)
#        target_compile_definitions(${TARGET} PRIVATE -DRL_TOOLS_DEBUG_DEVICE_CUDA_CHECK_BOUNDS)
    endif()
endfunction()

add_executable(
        test_rl_cuda
        rl.cu
)
target_link_libraries(
        test_rl_cuda
        PRIVATE
        RLtools::Minimal
        rl_tools_test
)
#RL_TOOLS_TESTS_RL_CUDA_ADD_DEBUG_DEFINITIONS(test_rl_cuda)
#RL_TOOLS_TAG_IS_CUDA(test_rl_cuda)
gtest_discover_tests(test_rl_cuda)

add_executable(
        test_rl_cuda_td3_full_training
        td3_full_training.cu
)
target_link_libraries(
        test_rl_cuda_td3_full_training
        PRIVATE
        RLtools::Minimal
        rl_tools_test
)
#RL_TOOLS_TESTS_RL_CUDA_ADD_DEBUG_DEFINITIONS(test_rl_cuda_td3_full_training)
#RL_TOOLS_TAG_IS_CUDA(test_rl_cuda_td3_full_training)
gtest_discover_tests(test_rl_cuda_td3_full_training)

