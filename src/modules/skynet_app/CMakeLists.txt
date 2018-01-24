

px4_add_module(
        MODULE modules__skynet_app
        MAIN skynet_app
        COMPILE_FLAGS
        STACK_MAIN 2000
        SRCS
                skynet_app.cpp
        DEPENDS
                platforms__common
        )
