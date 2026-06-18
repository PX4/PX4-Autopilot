
if(RL_TOOLS_EXPERIMENTAL)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    add_executable(test_container_mixed_precision_bf16 bf16.cpp)
    target_link_libraries(test_container_mixed_precision_bf16 PRIVATE RLtools::RLtools Python3::Python GTest::gtest_main)
    target_include_directories(test_container_mixed_precision_bf16 PRIVATE ${Python3_INCLUDE_DIRS})
    gtest_discover_tests(test_container_mixed_precision_bf16)
endif()