project(test_rl_tools_rl_algorithms_sac_cuda CUDA)


if(RL_TOOLS_BACKEND_ENABLE_MKL)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CXX_STANDARD  17)
    set(CMAKE_CUDA_ARCHITECTURES "native")

    add_executable(test_rl_algorithms_sac_cuda sac.cu)
    target_link_libraries(test_rl_algorithms_sac_cuda PRIVATE rl_tools_full GTest::gtest_main)
    target_compile_options(test_rl_algorithms_sac_cuda PRIVATE -w)
    if(NOT CMAKE_BUILD_TYPE MATCHES Debug)
        target_compile_options(test_rl_algorithms_sac_cuda PRIVATE -O3 --fast-math)
    endif()
    gtest_discover_tests(test_rl_algorithms_sac_cuda)
endif()