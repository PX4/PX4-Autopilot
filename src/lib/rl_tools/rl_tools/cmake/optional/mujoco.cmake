if(RL_TOOLS_RL_ENVIRONMENTS_ENABLE_MUJOCO)
    set(MUJOCO_BUILD_EXAMPLES OFF)
    set(MUJOCO_BUILD_SIMULATE OFF)
    set(MUJOCO_BUILD_TESTS OFF)
    set(MUJOCO_TEST_PYTHON_UTIL OFF)
    find_package(mujoco QUIET)
    if(mujoco_FOUND)
        message(STATUS "Found existing/system mujoco ${mujoco_VERSION} at ${mujoco_DIR}")
    else()
        FetchContent_Declare(mujoco
                GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
                GIT_TAG   2.3.5
        )
        FetchContent_MakeAvailable(mujoco)
    endif()
    target_link_libraries(rl_tools_full INTERFACE mujoco::mujoco)
    target_compile_definitions(rl_tools_full INTERFACE RL_TOOLS_RL_ENVIRONMENTS_ENABLE_MUJOCO)

    find_package(glfw3 QUIET)
    if(glfw3_FOUND)
        message(STATUS "Found existing/system glfw3 ${glfw3_VERSION} at ${glfw3_DIR}")
    else()
        FetchContent_Declare(glfw3
                GIT_REPOSITORY https://github.com/glfw/glfw.git
                GIT_TAG   3fa2360720eeba1964df3c0ecf4b5df8648a8e52
        )
        FetchContent_MakeAvailable(glfw3)
    endif()
    target_link_libraries(rl_tools_full INTERFACE glfw)
    set(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ENABLE_UI ON)
    target_compile_definitions(rl_tools_full INTERFACE RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ENABLE_UI)
endif()
