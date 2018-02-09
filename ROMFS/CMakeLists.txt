
message(STATUS "ROMFS: ${config_romfs_root}")

set(romfs_temp_dir ${PX4_BINARY_DIR}/ROMFS/${config_romfs_root})
set(romfs_src_dir ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root})

# directory setup
# copy all romfs files, process airframes, prune comments
file(GLOB_RECURSE init_airframes ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/*/[1-9]*)
add_custom_command(OUTPUT ${romfs_temp_dir}/init.d/rcS ${romfs_temp_dir}/init.d/rc.autostart
	COMMAND cmake -E copy_directory ${romfs_src_dir} ${romfs_temp_dir}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		-a ${romfs_temp_dir}/init.d
		-s ${romfs_temp_dir}/init.d/rc.autostart
		--board ${BOARD}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_romfs_pruner.py
		--folder ${romfs_temp_dir} --board ${BOARD}
	DEPENDS
		${romfs_src_files}
		${init_airframes}
		${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d/rcS
		${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
	)

set(romfs_dependencies)
list(APPEND romfs_dependencies
	${romfs_temp_dir}/init.d/rcS
	${romfs_temp_dir}/init.d/rc.autostart
	)
	
# copy px4io binary if configured
if (config_io_board)
	list(APPEND romfs_dependencies
		copy_px4io_bin
		${fw_io_bin}
		)
endif()

# create romfs.img
add_custom_command(OUTPUT romfs.img
	COMMAND ${CMAKE_COMMAND} -E remove -f romfs.img romfs.txt
	COMMAND ${GENROMFS} -f romfs.img -d ${romfs_temp_dir} -V "NSHInitVol" -v > romfs.txt 2>&1
	DEPENDS ${romfs_dependencies}
	)

# create nsh_romfsimg.c
add_custom_command(OUTPUT nsh_romfsimg.c
	COMMAND ${CMAKE_COMMAND} -E remove -f nsh_romfsimg.c
	COMMAND xxd -i romfs.img nsh_romfsimg.c
	COMMAND sed 's/unsigned/const unsigned/g' nsh_romfsimg.c > nsh_romfsimg.c.tmp && mv nsh_romfsimg.c.tmp nsh_romfsimg.c
	DEPENDS romfs.img
	)

add_library(romfs STATIC nsh_romfsimg.c)
add_dependencies(romfs prebuild_targets)
set_target_properties(romfs PROPERTIES LINKER_LANGUAGE C)
