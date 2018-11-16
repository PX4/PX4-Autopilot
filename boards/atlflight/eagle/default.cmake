# The Eagle board is the first generation Snapdragon Flight board by Qualcomm.

# This cmake config builds for POSIX, so the part of the flight stack running
# on the Linux side of the Snapdragon.
include(sdflight/default)

set(CONFIG_SHMEM "1")

# This definition allows to differentiate if this just the usual POSIX build
# or if it is for the Snapdragon.
add_definitions(
	-D__PX4_POSIX_EAGLE
)
