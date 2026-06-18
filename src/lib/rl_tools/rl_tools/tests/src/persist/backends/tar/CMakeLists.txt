if(RL_TOOLS_ENABLE_TAR)
    add_executable(test_persist_backends_tar tar.cpp)
    target_link_libraries(test_persist_backends_tar RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_persist_backends_tar)

    add_executable(test_persist_backends_tar_posix test_tar_posix.cpp)
    target_link_libraries(test_persist_backends_tar_posix RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_persist_backends_tar_posix)
endif()
