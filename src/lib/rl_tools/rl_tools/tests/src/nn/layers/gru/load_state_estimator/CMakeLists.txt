
if(RL_TOOLS_ENABLE_HDF5 AND RL_TOOLS_TEST_DATA_PATH)
    add_executable(test_nn_layers_gru_load_state_estimator load_gru.cpp)
    target_link_libraries(test_nn_layers_gru_load_state_estimator PRIVATE RLtools::RLtools RLtools::Test)
    gtest_discover_tests(test_nn_layers_gru_load_state_estimator)
endif()
