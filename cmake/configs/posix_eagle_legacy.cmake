# The Eagle board is the first generation Snapdragon Flight board by Qualcomm
#
# This cmake config is for ARM POSIX and the resulting build runs on 
# the ARM application processor of the Snapdragon SoC

include(configs/qcom/posix_eagle_common)
include(configs/qcom/posix_modules_default)

# This config uses the legacy (binary) drivers
add_definitions(
    -D__USING_SNAPDRAGON_LEGACY_DRIVER
)
