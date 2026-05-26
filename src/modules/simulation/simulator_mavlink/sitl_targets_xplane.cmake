# sitl_targets_xplane.cmake
#
# Custom make targets for X-Plane native SITL airframes.
#
# Each target gets its own isolated working directory under
#   build/px4_sitl_default/rootfs/<target_name>/
# so that switching between airframes does not mix saved parameter files.
# The px4 binary is launched with -w <working_dir> so parameters.bson lands
# in the per-airframe directory, not the shared rootfs/.
#
# A per-model params file (ROMFS/init.d-posix/models/<model>.params) is
# sourced by px4-rc.xplanesim at first boot to set airframe-specific defaults.
#
# Usage:
#   add_xplane_target(<target_name> <sys_autostart_id> <airframe_file>)

macro(add_xplane_target TARGET_NAME SYS_AUTOSTART AIRFRAME_FILE)
    # Per-airframe isolated working directory — keeps parameters.bson separate
    set(_xplane_working_dir ${PX4_BINARY_DIR}/rootfs/${TARGET_NAME})
    file(MAKE_DIRECTORY ${_xplane_working_dir})

    add_custom_target(${TARGET_NAME}
        COMMAND ${CMAKE_COMMAND} -E env
            PX4_SYS_AUTOSTART=${SYS_AUTOSTART}
            PX4_SIMULATOR=xplane
            $<TARGET_FILE:px4>
            ${PX4_BINARY_DIR}/etc
            -w ${_xplane_working_dir}
        WORKING_DIRECTORY ${_xplane_working_dir}
        USES_TERMINAL
        DEPENDS
            px4
            ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/${AIRFRAME_FILE}
        COMMENT "Launching PX4 with native X-Plane backend (${TARGET_NAME}, SYS_AUTOSTART=${SYS_AUTOSTART})"
    )

endmacro()

# X-Plane Cessna 172 Configuration
add_xplane_target(xplane_cessna172 5001 5001_xplane_cessna172)

# X-Plane TB 2 Fixed Wing A-Tail Configuration
add_xplane_target(xplane_tb2 5002 5002_xplane_tb2)

# X-Plane Generic V-Tail Fixed-Wing (no flaps)
add_xplane_target(xplane_vtail 5003 5003_xplane_vtail)

# X-Plane eVTOL Air Taxi (Quad Copter) - ehang184 Configuration
add_xplane_target(xplane_ehang184 5010 5010_xplane_ehang184)

# X-Plane 5-inch Quadrotor Configuration
add_xplane_target(xplane_quad 5011 5011_xplane_quad)

# X-Plane eVTOL Air Taxi (Quad VTOL) - Alia-250 Configuration
add_xplane_target(xplane_alia250 5020 5020_xplane_alia250)

# X-Plane Quad Tailsitter VTOL - Quantix Configuration
add_xplane_target(xplane_qtailsitter 5021 5021_xplane_qtailsitter)
