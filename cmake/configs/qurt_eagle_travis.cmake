# The Eagle board is the first generation Snapdragon Flight board by Qualcomm
#
# This cmake config is for QuRT which is the RTOS running on the Hexagon DSP

# The config between different QuRT builds is shared
include(configs/qcom/qurt_eagle_common)
include(configs/qcom/qurt_modules_default)

# Run a full link with build stubs to make sure QuRT target isn't broken
set(QURT_ENABLE_STUBS "1")

