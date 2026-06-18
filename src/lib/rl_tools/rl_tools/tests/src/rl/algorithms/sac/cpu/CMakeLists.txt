find_package(Threads)

if(Threads_FOUND AND RL_TOOLS_ENABLE_HDF5)
    add_executable(
            test_rl_algorithms_sac_pendulum_full_training_core
            test.cpp
    )
    target_link_libraries(
            test_rl_algorithms_sac_pendulum_full_training_core
            PRIVATE
            RLtools::Core
            Threads::Threads
            RLtools::Test
    )
    target_compile_definitions(test_rl_algorithms_sac_pendulum_full_training_core PRIVATE RL_TOOLS_BACKEND_DISABLE_BLAS)
    target_compile_definitions(test_rl_algorithms_sac_pendulum_full_training_core PRIVATE RL_TOOLS_DISABLE_HDF5)
    target_compile_definitions(test_rl_algorithms_sac_pendulum_full_training_core PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
    gtest_discover_tests(test_rl_algorithms_sac_pendulum_full_training_core)

    if(RL_TOOLS_BACKEND_ENABLE_BLAS)
        add_executable(
                test_rl_algorithms_sac_pendulum_full_training_blas
                test_blas.cpp
        )
        target_link_libraries(
                test_rl_algorithms_sac_pendulum_full_training_blas
                PRIVATE
                RLtools::RLtools
                RLtools::Test
        )
        target_compile_definitions(test_rl_algorithms_sac_pendulum_full_training_blas PRIVATE RL_TOOLS_DISABLE_HDF5)
        target_compile_definitions(test_rl_algorithms_sac_pendulum_full_training_blas PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
        gtest_discover_tests(test_rl_algorithms_sac_pendulum_full_training_blas)
    endif()
endif()
