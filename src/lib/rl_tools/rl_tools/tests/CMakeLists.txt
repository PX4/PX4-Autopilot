option(RL_TOOLS_DOWNLOAD_GTEST OFF)

option(RL_TOOLS_TEST_LOCAL "Local dev" OFF)

#option(RL_TOOLS_TEST_RL_ENVIRONMENTS_PENDULUM_EVALUATE_VISUALLY "Visual Evaluation" ${RL_TOOLS_TEST_LOCAL})
option(RL_TOOLS_TEST_RL_ENVIRONMENTS_PENDULUM_EVALUATE_VISUALLY "Visual Evaluation" OFF)

option(RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_OUTPUT_PLOTS "Visual Evaluation" OFF)
#option(RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_EVALUATE_VISUALLY "Visual Evaluation" ${RL_TOOLS_TEST_LOCAL})
option(RL_TOOLS_TEST_RL_ALGORITHMS_TD3_SECOND_STAGE_EVALUATE_VISUALLY "Visual Evaluation" OFF)

#option(RL_TOOLS_TEST_RL_ALGORITHMS_TD3_FULL_TRAINING_OUTPUT_PLOTS "Visual Evaluation" ON)
option(RL_TOOLS_TEST_RL_ALGORITHMS_TD3_FULL_TRAINING_OUTPUT_PLOTS "Visual Evaluation" OFF)
#option(RL_TOOLS_TEST_RL_ALGORITHMS_TD3_FULL_TRAINING_EVALUATE_VISUALLY "Visual Evaluation" ${RL_TOOLS_TEST_LOCAL})
option(RL_TOOLS_TEST_RL_ALGORITHMS_TD3_FULL_TRAINING_EVALUATE_VISUALLY "Visual Evaluation" OFF)

option(RL_TOOLS_TESTS_RL_ENVIRONMENTS_MULTIROTOR_UI_ENABLE "Multirotor UI" OFF)

option(RL_TOOLS_TESTS_CODE_COVERAGE "Enable coverage reporting" OFF)

if(RL_TOOLS_TESTS_CODE_COVERAGE)
    add_compile_options(--coverage -g -O0)
    add_link_options(--coverage)
    add_compile_definitions(-DRL_TOOLS_TESTS_CODE_COVERAGE)
endif()

include(GoogleTest)

add_library(rl_tools_test INTERFACE)
target_link_libraries(rl_tools_test INTERFACE GTest::gtest_main)
if(RL_TOOLS_TEST_DATA_PATH)
    target_compile_definitions(rl_tools_test INTERFACE RL_TOOLS_TEST_DATA_PATH=${RL_TOOLS_TEST_DATA_PATH})
endif()
add_library(RLtools::Test ALIAS rl_tools_test)





if(RL_TOOLS_TESTS_ENABLE_EIGEN)
find_package(Eigen3 REQUIRED)
endif()
if(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ENABLE_UI)
#find_package(Boost REQUIRED)
endif()

add_subdirectory(src)
