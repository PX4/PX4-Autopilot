# The Eagle board is the first generation Snapdragon Flight board by Qualcomm.
#
# This cmake config builds for POSIX, so the part of the flight stack running
# on the Linux side of the Snapdragon.
include(configs/posix_sdflight_default)

# This definition allows to differentiate the specific board.
add_definitions(
	-D__PX4_POSIX_EAGLE
)
