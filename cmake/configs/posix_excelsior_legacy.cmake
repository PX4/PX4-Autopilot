# Excelsior is the code name of a board currently in development
#
# This cmake config is for ARM POSIX and the resulting build runs on 
# the ARM application processor of the Snapdragon SoC

include(configs/qcom/posix_excelsior_common)
include(configs/qcom/posix_modules_default)

# This config uses the legacy (binary) drivers
add_definitions(
    -D__USING_SNAPDRAGON_LEGACY_DRIVER
)
