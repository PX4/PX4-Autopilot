# The Eagle board is the first generation Snapdragon Flight board by Qualcomm
#
# This cmake config is for QuRT which is the RTOS running on the Hexagon DSP

# The legacy drivers (binary) are used in this config
include(configs/qcom/qurt_eagle_common)
include(configs/qcom/qurt_modules_legacy)
