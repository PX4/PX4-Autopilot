find_package(GTest QUIET)
if(GTest_FOUND)
    message(STATUS "Found existing/system GTest ${GTest_VERSION} at ${GTest_DIR}")
    set(RL_TOOLS_ENABLE_GTEST ON)
else()
    find_package(Git QUIET)
    if(GIT_FOUND)
        FetchContent_Declare(googletest
                GIT_REPOSITORY https://github.com/google/googletest.git
                GIT_TAG   52eb8108c5bdec04579160ae17225d66034bd723
        )
        FetchContent_MakeAvailable(googletest)
        set(RL_TOOLS_ENABLE_GTEST ON)
    else()
        message(STATUS "Git not found - GTest disabled")
    endif()
endif()