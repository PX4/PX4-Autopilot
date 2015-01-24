add_module(NAME uorb
    PATH src/modules/uORB 
    SRCS 
        uORB.cpp
        objects_common.cpp
        Publication.cpp
        Subscription.cpp
    STACK 4096
    MAIN uorb_main
    )

add_module(NAME flow_position_estimator
    PATH src/examples/flow_position_estimator
    SRCS 
        flow_position_estimator_main.c
        flow_position_estimator_params.c
    STACK 4096
    MAIN flow_position_estimator_main
    )

add_module(NAME px4_simple_app
    PATH src/examples/px4_simple_app
    SRCS
        px4_simple_app.c
    STACK 1024
    MAIN px4_simple_app_main
    )

# vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 ft=cmake:
