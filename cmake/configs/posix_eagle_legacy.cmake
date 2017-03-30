# Eagle is the code name of a board currently in development.
#
# This cmake config builds for POSIX, so the part of the flight stack running
# on the Linux side of the Snapdragon.

include(configs/qcom/posix_eagle_common)
include(configs/qcom/posix_modules_default)

add_definitions(
    -D__USING_SNAPDRAGON_LEGACY_DRIVER
)
