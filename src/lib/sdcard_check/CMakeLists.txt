# Generate checksums header from sdcard build directory
set(sdcard_checksums_header ${CMAKE_CURRENT_BINARY_DIR}/sdcard_checksums.h)

if(CONFIG_BOARD_ROOT_PATH)
	set(sdcard_dir ${PX4_BINARY_DIR}/sdcard)
	set(sdcard_dir_arg --sdcard-dir ${sdcard_dir})
	set(sdcard_deps sdcard_files)
else()
	set(sdcard_dir_arg)
	set(sdcard_deps)
endif()

add_custom_command(OUTPUT ${sdcard_checksums_header}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_sdcard_checksums.py
		${sdcard_dir_arg}
		--output ${sdcard_checksums_header}
	DEPENDS
		generate_sdcard_checksums.py
		${sdcard_deps}
	COMMENT "Generating SD card file checksums"
)
add_custom_target(sdcard_checksums_gen DEPENDS ${sdcard_checksums_header})

px4_add_library(sdcard_check sdcard_check.cpp)
target_include_directories(sdcard_check PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
add_dependencies(sdcard_check sdcard_checksums_gen)
