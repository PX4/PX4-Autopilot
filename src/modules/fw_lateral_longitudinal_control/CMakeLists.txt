set(CONTROL_DEPENDENCIES
        npfg
        tecs
)


px4_add_module(
        MODULE modules__fw_lateral_longitudinal_control
        MAIN fw_lat_lon_control
        SRCS
        FwLateralLongitudinalControl.cpp
        FwLateralLongitudinalControl.hpp
        DEPENDS
        ${CONTROL_DEPENDENCIES}
)
