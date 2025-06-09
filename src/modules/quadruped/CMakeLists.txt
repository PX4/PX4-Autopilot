px4_add_module(
        MODULE modules__quadruped
        MAIN quadruped
        SRCS
                Quadruped.cpp
        DEPENDS
                px4_work_queue
        MODULE_CONFIG
                module.yaml
)
