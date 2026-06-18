find_package(Threads REQUIRED)

add_executable(rl_environments_mujoco_ant_ppo_standalone standalone.cpp)
target_link_libraries(rl_environments_mujoco_ant_ppo_standalone PRIVATE RLtools::RLtools Threads::Threads)
# todo: link mujoco like this everywhere
target_compile_definitions(rl_environments_mujoco_ant_ppo_standalone PRIVATE RL_TOOLS_BACKEND_DISABLE_BLAS)
target_compile_definitions(rl_environments_mujoco_ant_ppo_standalone PRIVATE RL_TOOLS_DISABLE_HDF5)
target_compile_definitions(rl_environments_mujoco_ant_ppo_standalone PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)

if(RL_TOOLS_BACKEND_ENABLE_BLAS)
    add_executable(rl_environments_mujoco_ant_ppo_blas_benchmark standalone.cpp)
    target_link_libraries(rl_environments_mujoco_ant_ppo_blas_benchmark PRIVATE RLtools::RLtools Threads::Threads)
    # todo: link mujoco like this everywhere
    target_compile_definitions(rl_environments_mujoco_ant_ppo_blas_benchmark PRIVATE RL_TOOLS_DISABLE_HDF5)
    target_compile_definitions(rl_environments_mujoco_ant_ppo_blas_benchmark PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
endif()

if(RL_TOOLS_BACKEND_ENABLE_BLAS AND RL_TOOLS_ENABLE_HDF5 AND RL_TOOLS_ENABLE_TENSORBOARD)
add_executable(rl_environments_mujoco_ant_ppo_blas standalone.cpp)
target_link_libraries(rl_environments_mujoco_ant_ppo_blas PRIVATE RLtools::RLtools mujoco::mujoco Threads::Threads)
# todo: link mujoco like this everywhere
endif()

