px4_add_module(
    MODULE indi_att_control
    MAIN indi_att_control_main.cpp
    SRCS
      indi_att_control_main.cpp
      indi_att_control.hpp
    DEPENDS
      mathlib
      px4_work_queue
)
