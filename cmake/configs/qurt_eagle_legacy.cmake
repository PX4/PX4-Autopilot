# Eagle is the code name of a board currently in development.
#
# This cmake config builds for QURT which is the operating system running on
# the DSP side.

# The config between different QURT builds is shared.
include(configs/qurt_sdflight_legacy)
add_definitions(
   -D__USING_SNAPDRAGON_LEGACY_DRIVER
   -D__PX4_QURT
   -D__PX4_QURT_EAGLE
   )
