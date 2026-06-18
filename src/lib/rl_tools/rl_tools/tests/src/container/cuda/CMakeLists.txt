project(test_rl_tools_container_cuda CUDA)


if(RL_TOOLS_BACKEND_ENABLE_CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CXX_STANDARD  17)
    add_executable(
            test_container_cuda
            test.cu
#            test.cpp
    )
    target_link_libraries(
            test_container_cuda
            PRIVATE
            RLtools::Minimal
            GTest::gtest_main
    )
    gtest_discover_tests(test_container_cuda)
endif()

if(RL_TOOLS_BACKEND_ENABLE_CUDA AND RL_TOOLS_BACKEND_ENABLE_MKL)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CXX_STANDARD  17)
    add_executable(
            test_container_cuda_mkl
            mkl.cu
    )
    target_link_libraries(
            test_container_cuda_mkl
            PRIVATE
            RLtools::Minimal
            GTest::gtest_main
    )

    gtest_discover_tests(test_container_cuda_mkl)
endif()
