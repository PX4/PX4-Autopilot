# The Eagle board is the first generation Snapdragon Flight board by Qualcomm

include(configs/qcom/qurt_common)

# This definition allows to differentiate the specific board
add_definitions(
   -D__PX4_QURT_EAGLE
   )
