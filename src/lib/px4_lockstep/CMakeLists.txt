# PX4 lockstep harness library (Strategy A)
#
# Drop this folder into PX4-Autopilot/src/lib/px4_lockstep and add:
#   add_subdirectory(px4_lockstep)
# from an appropriate CMake scope (e.g. src/lib/CMakeLists.txt)
#
# This builds libpx4_lockstep.so for dlopen/ccall from Julia.

add_library(px4_lockstep SHARED
  src/px4_lockstep.cpp
  ${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/test_stubs/stub_daemon.cpp
)

target_include_directories(px4_lockstep
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    # Access module headers
    ${PX4_SOURCE_DIR}/src/modules/commander
    ${PX4_SOURCE_DIR}/src/modules/navigator
    ${PX4_SOURCE_DIR}/src/modules/mc_pos_control
    ${PX4_SOURCE_DIR}/src/modules/mc_att_control
    ${PX4_SOURCE_DIR}/src/modules/mc_att_control/AttitudeControl
    ${PX4_SOURCE_DIR}/src/modules/mc_rate_control
    ${PX4_SOURCE_DIR}/src/modules/flight_mode_manager
    ${PX4_BINARY_DIR}/src/modules/flight_mode_manager
	  ${PX4_SOURCE_DIR}/src/modules/control_allocator
    ${PX4_SOURCE_DIR}/src/modules/control_allocator/VehicleActuatorEffectiveness
    ${PX4_SOURCE_DIR}/src/lib/control_allocation/actuator_effectiveness
    ${PX4_SOURCE_DIR}/src/lib/control_allocation/control_allocation
    ${PX4_SOURCE_DIR}/src/modules/dataman
)

# Ensure symbols are visible for Julia's dlopen/ccall.
set_target_properties(px4_lockstep PROPERTIES
  C_VISIBILITY_PRESET default
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)

# You will likely want/need these module libs (names depend on PX4 build system generation):
# - modules__navigator
# - modules__mc_pos_control
# - modules__mc_att_control
# - modules__mc_rate_control
# - modules__dataman
# plus common libs like uORB, px4_platform, etc.
#
# Keep as a best-effort list; adjust as needed for your build graph.

target_link_libraries(px4_lockstep
  PRIVATE
    modules__commander
    modules__navigator
    modules__mc_pos_control
    modules__mc_att_control
    modules__mc_rate_control
    modules__flight_mode_manager
	  modules__control_allocator
    modules__dataman
)
