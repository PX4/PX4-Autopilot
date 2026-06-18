
project(rl_environments_mujoco_ant_cuda LANGUAGES CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD  17)
set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
message("CUDA Compiler: ${CMAKE_CUDA_COMPILER}")
message("CUDA Host Compiler: ${CMAKE_CUDA_HOST_COMPILER}")

# add_executable(
#         rl_environments_mujoco_ant_throughput_cuda
#         throughput.cu
# )
# target_link_libraries(
#         rl_environments_mujoco_ant_throughput_cuda
#         PRIVATE
#         rl_tools
#         HighFive
#         tensorboard_logger
#         CUDA::cublas
# )


if(RL_TOOLS_ENABLE_CLI11)
add_executable(
        rl_environments_mujoco_ant_ppo_cuda_standalone
        training_ppo.cu
)
target_link_libraries(
        rl_environments_mujoco_ant_ppo_cuda_standalone
        PRIVATE
        RLtools::RLtools
        mujoco::mujoco
        CLI11::CLI11
)
target_compile_definitions(rl_environments_mujoco_ant_ppo_cuda_standalone PRIVATE RL_TOOLS_BACKEND_DISABLE_BLAS)
target_compile_definitions(rl_environments_mujoco_ant_ppo_cuda_standalone PRIVATE RL_TOOLS_DISABLE_HDF5)
target_compile_definitions(rl_environments_mujoco_ant_ppo_cuda_standalone PRIVATE RL_TOOLS_DISABLE_CLI11)
target_compile_definitions(rl_environments_mujoco_ant_ppo_cuda_standalone PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
endif()

if(RL_TOOLS_BACKEND_ENABLE_BLAS AND RL_TOOLS_ENABLE_CLI11)
add_executable(
        rl_environments_mujoco_ant_ppo_cuda_benchmark
        training_ppo.cu
)
target_link_libraries(
        rl_environments_mujoco_ant_ppo_cuda_benchmark
        PRIVATE
        RLtools::RLtools
        mujoco::mujoco
        CLI11::CLI11
)

if(NOT WIN32)
# for some reason this does not work on windows and crashes with an access violation in MKL (strangely enough this works when linking tensorboard_logger)
#        RL_TOOLS_TARGET_LINK_DEPENDENCIES(rl_environments_mujoco_ant_ppo_cuda_benchmark)
else()
        target_compile_definitions(rl_environments_mujoco_ant_ppo_cuda_benchmark PRIVATE RL_TOOLS_BACKEND_DISABLE_BLAS)
endif()
target_compile_definitions(rl_environments_mujoco_ant_ppo_cuda_benchmark PRIVATE RL_TOOLS_DISABLE_HDF5)
target_compile_definitions(rl_environments_mujoco_ant_ppo_cuda_benchmark PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
target_compile_definitions(rl_environments_mujoco_ant_ppo_cuda_benchmark PRIVATE RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT_DISABLE_EVALUATION)
endif()


if(RL_TOOLS_BACKEND_ENABLE_BLAS AND RL_TOOLS_ENABLE_HDF5 AND RL_TOOLS_ENABLE_TENSORBOARD AND RL_TOOLS_ENABLE_CLI11)
add_executable(
        rl_environments_mujoco_ant_ppo_cuda_full
        training_ppo.cu
)
target_link_libraries(
        rl_environments_mujoco_ant_ppo_cuda_full
        PRIVATE
        RLtools::RLtools
        mujoco::mujoco
        CLI11::CLI11
)
endif()

# add_executable(
#         rl_environments_mujoco_ant_collection_cuda
#         collection.cu
# )
# target_link_libraries(
#         rl_environments_mujoco_ant_collection_cuda
#         PRIVATE
#         rl_tools
#         HighFive
#         tensorboard_logger
#         mujoco
#         CUDA::cublas
# )
# RL_TOOLS_TAG_IS_CUDA(rl_environments_mujoco_ant_collection_cuda)
