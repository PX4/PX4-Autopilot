
find_program(JAVA_ANT_PATH "ant")
find_package(Java)

if(JAVA_ANT_PATH AND Java_JAVAC_EXECUTABLE AND Java_JAVA_EXECUTABLE)

	px4_add_git_submodule(TARGET git_jmavsim PATH "${PX4_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim")

	add_custom_target(jmavsim_run_symlink ALL
		COMMAND ${CMAKE_COMMAND} -E create_symlink ${PX4_SOURCE_DIR}/Tools/simulation/jmavsim/jmavsim_run.sh ${PX4_BINARY_DIR}/rootfs/jmavsim_run.sh
		BYPRODUCTS ${PX4_BINARY_DIR}/rootfs/jmavsim_run.sh
	)

	# build_jmavsim
	add_custom_command(
		OUTPUT ${PX4_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim/out/production/jmavsim_run.jar
		COMMAND ${JAVA_ANT_PATH} create_run_jar copy_res
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim/
		USES_TERMINAL
		DEPENDS git_jmavsim jmavsim_run_symlink
		COMMENT "building jMAVSim"
	)
	add_custom_target(build_jmavsim DEPENDS ${PX4_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim/out/production/jmavsim_run.jar)

	# launch helper
	add_custom_target(jmavsim_iris
		COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=jmavsim_iris PX4_SIMULATOR=jmavsim $<TARGET_FILE:px4>
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4 git_jmavsim build_jmavsim jmavsim_run_symlink
	)

endif()
