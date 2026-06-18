add_executable(test_math math.cpp)
target_link_libraries(test_math PRIVATE RLtools::Core GTest::gtest_main)
gtest_discover_tests(test_math)
