# Container tests
add_executable(
        test_container
        container.cpp
)
target_link_libraries(
        test_container
        PRIVATE
        RLtools::Core
        GTest::gtest_main
)
gtest_discover_tests(test_container)

if(RL_TOOLS_BACKEND_ENABLE_CUDA)
#    add_subdirectory(cuda)
endif()

# Container tests
add_executable(test_container_persist_code_store persist_code_store.cpp)
target_link_libraries(test_container_persist_code_store RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_container_persist_code_store)

add_executable(tensor_test tensor.cpp)
target_link_libraries(tensor_test PRIVATE RLtools::Core GTest::gtest_main)
gtest_discover_tests(tensor_test)

if(RL_TOOLS_ENABLE_HDF5 AND RL_TOOLS_TEST_DATA_PATH)
    add_executable(tensor_persist_test tensor_persist.cpp)
    target_link_libraries(tensor_persist_test PRIVATE RLtools::RLtools GTest::gtest_main)
    gtest_discover_tests(tensor_persist_test)
endif()

add_executable(test_tensor_persist_code_store tensor_persist_code_store.cpp)
target_link_libraries(test_tensor_persist_code_store PRIVATE RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_tensor_persist_code_store)

if(RL_TOOLS_TEST_DATA_PATH)
    add_executable(test_tensor_persist_code_compile tensor_persist_code_compile.cpp)
    target_link_libraries(test_tensor_persist_code_compile PRIVATE RLtools::Core GTest::gtest_main)
    gtest_discover_tests(test_tensor_persist_code_compile)
endif()

add_executable(tensor_sum_test tensor_sum.cpp )
target_link_libraries(tensor_sum_test PRIVATE RLtools::Core)


add_executable(test_container_matmul matmul.cpp)
target_link_libraries(test_container_matmul RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_container_matmul)

add_executable(test_container_matmul_fast matmul_fast.cpp)
target_link_libraries(test_container_matmul_fast RLtools::RLtools)


add_subdirectory(tensor)
add_subdirectory(mixed_precision)
