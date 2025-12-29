px4_add_module(
    MODULE drivers__winch_eayload
    MAIN winch_eayload
    SRCS
        winch_eayload.cpp
        winch_params.c
    DEPENDS
        px4_work_queue
)
