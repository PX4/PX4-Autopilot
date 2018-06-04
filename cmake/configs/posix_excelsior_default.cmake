# Excelsior is the code name of a board currently in development.
#
# This cmake config builds for POSIX, so the part of the flight stack running
# on the Linux side of the Snapdragon.
include(configs/posix_sdflight_default)

set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

# Get $QC_SOC_TARGET from environment if existing.
if (DEFINED ENV{QC_SOC_TARGET})
	set(QC_SOC_TARGET $ENV{QC_SOC_TARGET})
else()
	set(QC_SOC_TARGET "APQ8096")
endif()

set(CONFIG_SHMEM "1")

# This definition allows to differentiate the specific board.
add_definitions(
	-D__PX4_POSIX_EXCELSIOR
)
