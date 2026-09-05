px4_add_module(
    MODULE modules__complementary_filter
    MAIN complementary_filter
    STACK_MAIN 1024
    SRCS
        complementary_filter.cpp
)