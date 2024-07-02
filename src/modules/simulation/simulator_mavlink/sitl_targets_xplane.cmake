# sitl_targets_xplane.cmake
#
# This CMake configuration file creates custom targets for SITL airframes
# that are specifically designed to work with the X-Plane simulator.
#
# To add a new X-Plane airframe target, use the macro add_xplane_target defined below.
# This macro simplifies the process of adding new airframe configurations and ensures
# that each target adheres to the required structure for SITL simulation with X-Plane.
#
# Usage:
#   add_xplane_target(<target_name> <sys_autostart_id> <airframe_file>)
# Where:
#   <target_name> is the custom target name used for the make command.
#   <sys_autostart_id> is the unique identifier for the airframe.
#   <airframe_file> is the name of the airframe file located in the PX4 ROMFS directory.

# Define a macro to add new X-Plane airframe targets.
macro(add_xplane_target TARGET_NAME SYS_AUTOSTART AIRFRAME_FILE)
    add_custom_target(${TARGET_NAME}
        COMMAND ${CMAKE_COMMAND} -E env PX4_SYS_AUTOSTART=${SYS_AUTOSTART} $<TARGET_FILE:px4>
        WORKING_DIRECTORY ${SITL_WORKING_DIR}
        USES_TERMINAL
        DEPENDS
            px4
            ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/${AIRFRAME_FILE}
        COMMENT "Launching PX4 with X-Plane airframe (SYS_AUTOSTART=${SYS_AUTOSTART})"
    )
endmacro()

# X-Plane Cessna 172 Configuration
add_xplane_target(xplane_cessna172 5001 5001_xplane_cessna172)

# X-Plane TB 2 Fixed Wing A-Tail Configuration
add_xplane_target(xplane_tb2 5002 5002_xplane_tb2)

# X-Plane eVTOL Air Taxi (Quad Copter) - ehang184 Configuration
add_xplane_target(xplane_ehang184 5010 5010_xplane_ehang184)

# X-Plane eVTOL Air Taxi (Quad VTOL) - Alia-250 Configuration
add_xplane_target(xplane_alia250 5020 5020_xplane_alia250)

# Instructions for adding more X-Plane configurations:
# To add additional airframe configurations for X-Plane, follow these steps:
#   1. Ensure the airframe file is present in the PX4 ROMFS directory under init.d-posix/airframes.
#   2. Use the add_xplane_target macro with appropriate parameters to add a new target.
# Example:
#   add_xplane_target(new_airframe 1234 1234_new_airframe)
# Replace 'new_airframe' with your target's name, '1234' with your SYS_AUTOSTART ID,
# and '1234_new_airframe' with the corresponding airframe file name.
