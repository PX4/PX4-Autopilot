project(rl_environments_pendulum_cuda_sac CUDA)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD  17)

if(RL_TOOLS_BACKEND_ENABLE_CUDA)
    add_executable(
            rl_environments_pendulum_sac_cuda_init_test
            init_test.cu
    )
    target_link_libraries(
            rl_environments_pendulum_sac_cuda_init_test
            PRIVATE
            rl_tools_full # required for cuBLAS
    )
    target_compile_options(rl_environments_pendulum_sac_cuda_init_test PRIVATE -O3)
    target_compile_definitions(rl_environments_pendulum_sac_cuda_init_test PRIVATE RL_TOOLS_BACKEND_ENABLE_CUDA)
    target_compile_options(rl_environments_pendulum_sac_cuda_init_test PRIVATE -w)
endif()

if(RL_TOOLS_BACKEND_ENABLE_CUDA)
    add_executable(
            rl_environments_pendulum_sac_cuda
            sac.cu
    )
    target_link_libraries(
            rl_environments_pendulum_sac_cuda
            PRIVATE
            rl_tools_full # required for cuBLAS
    )
    target_compile_definitions(rl_environments_pendulum_sac_cuda PRIVATE RL_TOOLS_BACKEND_ENABLE_CUDA)
    target_compile_options(rl_environments_pendulum_sac_cuda PRIVATE -g -Xcompiler -fno-omit-frame-pointer --diag-suppress=186,177)
endif()


if(RL_TOOLS_BACKEND_ENABLE_CUDA)
    add_executable(
            rl_environments_pendulum_sac_cuda_benchmark
            sac.cu
    )
    target_link_libraries(
            rl_environments_pendulum_sac_cuda_benchmark
            PRIVATE
            rl_tools_full # required for cuBLAS
    )
    target_compile_definitions(rl_environments_pendulum_sac_cuda_benchmark PRIVATE RL_TOOLS_BACKEND_ENABLE_CUDA)
    target_compile_definitions(rl_environments_pendulum_sac_cuda_benchmark PRIVATE BENCHMARK)
if(CXX_COMPILER_ID MATCHES "GNU" OR CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(rl_environments_pendulum_sac_cuda_benchmark PRIVATE -O3)
    target_compile_options(rl_environments_pendulum_sac_cuda_benchmark PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Ofast -march=native -mtune=native -flto>)
endif()
endif()
