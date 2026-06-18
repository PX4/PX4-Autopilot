add_executable(
        test_nn_layers_dense
        dense.cpp
)
target_link_libraries(
        test_nn_layers_dense
        PRIVATE
        RLtools::Core
        GTest::gtest_main
)
gtest_discover_tests(test_nn_layers_dense)

add_subdirectory(dense)
add_subdirectory(gru)
add_subdirectory(standardize)
add_subdirectory(embedding)
