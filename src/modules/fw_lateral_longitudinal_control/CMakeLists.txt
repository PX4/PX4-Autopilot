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
        SkidToTurnAllocator.hpp
        DEPENDS
        ${CONTROL_DEPENDENCIES}
	MODULE_CONFIG
		fw_lat_long_params.yaml
	)

px4_add_unit_gtest(SRC SkidToTurnAllocatorTest.cpp LINKLIBS mathlib)
