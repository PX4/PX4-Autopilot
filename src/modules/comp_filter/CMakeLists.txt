px4_add_module(
    MODULE modules__comp_filter
    MAIN comp_filter
    STACK_MAIN 2000
    SRCS
        ComplementaryFilter.cpp
        ComplementaryFilter.hpp
    DEPENDS
        px4_work_queue
        uORB
)