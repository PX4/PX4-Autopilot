# The Eagle board is the first generation Snapdragon Flight board by Qualcomm.
#
# This cmake config builds for QURT which is the operating system running on
# the DSP side.

# The config between different QURT builds is shared.
include(sdflight/qurt-default)

# This definition allows to differentiate the specific board.
add_definitions(
	-D__PX4_QURT_EAGLE
)
