
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
	
# create romfs.bin
add_custom_command(OUTPUT romfs.img
	COMMAND ${GENROMFS} -f romfs.img -d ${romfs_temp_dir} -V "NSHInitVol" -v > romfs.txt 2>&1
	DEPENDS ${romfs_dependencies}
	)

# create romfs.o
add_custom_command(OUTPUT romfs.o
	COMMAND ${OBJCOPY} -I binary -O elf32-littlearm -B arm romfs.img romfs.o
		--redefine-sym _binary_romfs_img_start=romfs_img
		--redefine-sym _binary_romfs_img_size=romfs_img_len
		--strip-symbol _binary_romfs_img_end
		--rename-section .data=.rodata
	DEPENDS romfs.img
	)

add_library(romfs STATIC romfs.o)
set_target_properties(romfs PROPERTIES LINKER_LANGUAGE C)
