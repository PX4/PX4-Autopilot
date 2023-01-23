

# upload script

# build px4_fmu-v5x
#  node checks version and uploads

include(ExternalProject)

ExternalProject_Add(px4_firmware_build
	DOWNLOAD_COMMAND rsync -a --safe-links --delete --exclude=build --exclude=*.o --exclude=*.a --exclude=.config ${PX4_SOURCE_DIR}/ ${PX4_BINARY_DIR}/external/Source/px4io_firmware_build/
	UPDATE_COMMAND   rsync -a --safe-links --delete --exclude=build --exclude=*.o --exclude=*.a --exclude=.config ${PX4_SOURCE_DIR}/ ${PX4_BINARY_DIR}/external/Source/px4io_firmware_build/
	DEPENDS git_nuttx git_nuttx_apps
	SOURCE_DIR ${PX4_BINARY_DIR}/external/Source/px4_firmware_build/
	CMAKE_ARGS -DCONFIG=px4_fmu-v5x_default
	INSTALL_COMMAND ""
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS true
)
