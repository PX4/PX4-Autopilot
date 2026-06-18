if(NOT RL_TOOLS_DISABLE_TENSORBOARD)
    find_package(Protobuf QUIET)
    find_package(Git QUIET)
    if(Protobuf_FOUND AND Protobuf_PROTOC_EXECUTABLE AND GIT_FOUND)
        FetchContent_Declare(tensorboard
                GIT_REPOSITORY https://github.com/rl-tools/tensorboard_logger.git
                GIT_TAG   d57f9887d2df19db6923b76f73fa6c06f2ecb3b3
        )
        FetchContent_MakeAvailable(tensorboard)
        target_link_libraries(rl_tools_full INTERFACE tensorboard_logger)
        target_compile_definitions(rl_tools_full INTERFACE RL_TOOLS_ENABLE_TENSORBOARD)
        set(RL_TOOLS_ENABLE_TENSORBOARD ON)
    else()
        message(STATUS "Protobuf not found, TensorBoard disabled.")
    endif()
endif()
