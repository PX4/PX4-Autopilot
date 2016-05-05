# Excelsior is the code name of a board currently in development.
#
# This cmake config builds for QURT which is the operating system running on
# the DSP side.

# The config between different QURT builds is shared.
include(configs/qurt_sdflight_default)

# This definition allows to differentiate the specific board.
add_definitions(
	-D__PX4_QURT_EXCELSIOR
)
