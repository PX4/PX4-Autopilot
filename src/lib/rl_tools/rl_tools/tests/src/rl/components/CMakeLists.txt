

if(RL_TOOLS_ENABLE_HDF5)
    add_executable(test_rl_components_replay_buffer replay_buffer.cpp)
    target_link_libraries(test_rl_components_replay_buffer PRIVATE RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_components_replay_buffer)

    # On Policy Runner tests
    add_executable(test_rl_components_on_policy_runner on_policy_runner.cpp)
    target_link_libraries( test_rl_components_on_policy_runner RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_rl_components_on_policy_runner)
endif()

if(RL_TOOLS_ENABLE_HDF5)
#add_executable(
#        test_rl_components_running_normalizer
#        running_normalizer.cpp
#)
#target_link_libraries(
#        test_rl_components_running_normalizer
#        PRIVATE
#        RLtools::Core
#        GTest::gtest_main
#)
#gtest_discover_tests(test_rl_components_running_normalizer)
endif()


# RL Algorithm off_policy_runner test
add_executable(test_rl_components_off_policy_runner off_policy_runner.cpp)
target_link_libraries(test_rl_components_off_policy_runner RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_rl_components_off_policy_runner)



add_subdirectory(off_policy_runner)