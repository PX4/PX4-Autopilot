# The Eagle board is the first generation Snapdragon Flight board by Qualcomm.
#
# This cmake config builds for QURT which is the operating system running on
# the DSP side.

# The config between different QURT builds is shared.
include(configs/qcom/qurt_excelsior_common)
include(configs/qcom/qurt_modules_default)
