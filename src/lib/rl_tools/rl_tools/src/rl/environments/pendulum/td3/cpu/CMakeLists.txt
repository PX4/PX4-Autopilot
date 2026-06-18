# RL Algorithm td3 test full training standalone
if(NOT MSVC)
add_executable(
        rl_environments_pendulum_td3_bare
        bare.cpp
)
target_link_libraries(
        rl_environments_pendulum_td3_bare
        PRIVATE
        RLtools::Core
)
target_compile_options(rl_environments_pendulum_td3_bare PRIVATE -nostdlib)
endif()

# RL Algorithm td3 test full training standalone
add_executable(
        rl_environments_pendulum_td3_standalone
        standalone.cpp
)
target_link_libraries(
        rl_environments_pendulum_td3_standalone
        PRIVATE
        RLtools::Core
)

# RL Algorithm td3 test full training blas
if(RL_TOOLS_BACKEND_ENABLE_BLAS)
add_executable(
        rl_environments_pendulum_td3_blas
        standalone.cpp
)
target_link_libraries(
        rl_environments_pendulum_td3_blas
        PRIVATE
        RLtools::RLtools
)
target_compile_definitions(rl_environments_pendulum_td3_blas PRIVATE RL_TOOLS_DISABLE_HDF5)
target_compile_definitions(rl_environments_pendulum_td3_blas PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
endif()

# RL Algorithm td3 test full training benchmark (disabling evaluation)
if(RL_TOOLS_BACKEND_ENABLE_BLAS)
add_executable(
        rl_environments_pendulum_td3_blas_benchmark
        benchmark.cpp
)
target_link_libraries(
        rl_environments_pendulum_td3_blas_benchmark
        PRIVATE
        RLtools::RLtools
)
target_compile_definitions(rl_environments_pendulum_td3_blas_benchmark PRIVATE RL_TOOLS_DISABLE_HDF5)
target_compile_definitions(rl_environments_pendulum_td3_blas_benchmark PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
target_compile_definitions(rl_environments_pendulum_td3_blas_benchmark PRIVATE RL_TOOLS_RL_ENVIRONMENTS_PENDULUM_DISABLE_EVALUATION)
target_compile_options(rl_environments_pendulum_td3_blas_benchmark PRIVATE -ffast-math)
    target_compile_definitions(rl_environments_pendulum_td3_blas_benchmark PRIVATE RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD)
endif()

# RL Algorithm td3 test full training tensorboard
if(RL_TOOLS_ENABLE_TENSORBOARD AND RL_TOOLS_BACKEND_ENABLE_BLAS)
add_executable(
        rl_environments_pendulum_td3_blas_tensorboard
        standalone.cpp
)
target_link_libraries(
        rl_environments_pendulum_td3_blas_tensorboard
        PRIVATE
        RLtools::RLtools
)
endif()
