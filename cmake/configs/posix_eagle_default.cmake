include(configs/posix_sdflight_default)

# This definition allows to differentiate if this just the usual POSIX build
# or if it is for the Snapdragon.
add_definitions(
	-D__PX4_POSIX_EAGLE
	)
