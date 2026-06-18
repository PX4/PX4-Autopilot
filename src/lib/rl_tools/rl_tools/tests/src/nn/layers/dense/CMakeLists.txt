
if(RL_TOOLS_BACKEND_ENABLE_MKL)
    add_executable(test_nn_layers_dense_mkl mkl.cpp)
    target_link_libraries(test_nn_layers_dense_mkl PRIVATE RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(test_nn_layers_dense_mkl)
endif()

add_executable(test_nn_layers_dense_tensor tensor.cpp)
target_link_libraries(test_nn_layers_dense_tensor PRIVATE RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_nn_layers_dense_tensor)

add_executable(test_nn_layers_dense_persist_code_store persist_code_store.cpp)
target_link_libraries(test_nn_layers_dense_persist_code_store PRIVATE RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_nn_layers_dense_persist_code_store)

if(RL_TOOLS_TEST_DATA_PATH)
    add_executable(test_nn_layers_dense_persist_code_compile persist_code_compile.cpp)
    target_link_libraries(test_nn_layers_dense_persist_code_compile PRIVATE RLtools::Core GTest::gtest_main)
    gtest_discover_tests(test_nn_layers_dense_persist_code_compile)
endif()
