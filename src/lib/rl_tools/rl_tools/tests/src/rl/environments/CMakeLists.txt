
if(RL_TOOLS_RL_ENVIRONMENTS_ENABLE_MUJOCO)
    add_subdirectory(mujoco)
endif()
add_subdirectory(car)
add_subdirectory(acrobot)
add_subdirectory(pendulum)
add_subdirectory(l2f)
add_subdirectory(memory)

