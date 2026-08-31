px4_add_module(
    MODULE modules__complementary_filter
    MAIN complementary_filter
    STACK_MAIN 2000
    SRCS
        complementary_filter.cpp
        Subsystem.c
        rt_nonfinite.c
        rtGetNaN.c
)

target_compile_options(modules__complementary_filter PRIVATE -Wno-float-equal)
