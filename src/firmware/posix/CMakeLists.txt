include_directories(${CMAKE_CURRENT_BINARY_DIR})

px4_posix_generate_builtin_commands(
	OUT apps
	MODULE_LIST ${module_libraries})

# Define build target
set(APP_NAME px4)
set(MAIN_SRC ${PX4_SOURCE_DIR}/src/platforms/posix/main.cpp)
set(UPLOAD_NAME upload)

if ("${BOARD}" STREQUAL "eagle" OR ("${BOARD}" STREQUAL "excelsior"))

	include(fastrpc)
	include(linux_app)

	FASTRPC_STUB_GEN(../qurt/px4muorb.idl)

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-prototypes -Wno-missing-declarations")
	LINUX_APP(
		APP_NAME ${APP_NAME}
		IDL_NAME px4muorb
		APPS_DEST "/home/linaro"
		SOURCES
			px4muorb_stub.c
			${MAIN_SRC}
			apps.cpp
		LINK_LIBS
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			${FASTRPC_ARM_LIBS}
			pthread m rt
			-Wl,--end-group
		)

	px4_add_adb_push(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}
				${PX4_SOURCE_DIR}/posix-configs/eagle/flight/mainapp.config
				DEPENDS ${APP_NAME}
				DEST /home/linaro)

elseif ("${BOARD}" STREQUAL "rpi")

	px4_add_executable(${APP_NAME}
		${MAIN_SRC}
		apps.cpp
		)

	target_link_libraries(${APP_NAME}
		-Wl,--start-group
		${module_libraries}
		df_driver_framework
		${df_driver_libs}
		pthread m rt
		-Wl,--end-group
		)

	file(GLOB RPI_CONFIG_FILES ${PX4_SOURCE_DIR}/posix-configs/rpi/*.config)
	px4_add_scp_push(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}
				${RPI_CONFIG_FILES}
				${PX4_SOURCE_DIR}/ROMFS
				DEPENDS ${APP_NAME}
				DEST /home/pi)

elseif ("${BOARD}" STREQUAL "bebop")

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static")

	px4_add_executable(${APP_NAME}
		${MAIN_SRC}
		apps.cpp
		)

	if (NOT APPLE)
		target_link_libraries(${APP_NAME}
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			pthread m rt
			-Wl,--end-group
			)
	else()
		target_link_libraries(${APP_NAME}
			${module_libraries}
			${df_driver_libs}
			pthread m
			)
	endif()

	px4_add_adb_push_to_bebop(OUT ${UPLOAD_NAME}
				OS ${OS}
				BOARD ${BOARD}
				FILES ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}
				DEPENDS ${APP_NAME}
				DEST /usr/bin)

elseif ("${BOARD}" STREQUAL "sitl")

	include(./sitl_target.cmake)

else()

	px4_add_executable(${APP_NAME}
		${MAIN_SRC}
		apps.cpp
		)

	if (NOT APPLE)
		target_link_libraries(${APP_NAME}
			-Wl,--start-group
			${module_libraries}
			${df_driver_libs}
			pthread m rt
			-Wl,--end-group
			)
	else()
		target_link_libraries(${APP_NAME}
			${module_libraries}
			${df_driver_libs}
			pthread m
			)
	endif()
endif()

#=============================================================================
# install
#

install(TARGETS px4 DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/ROMFS DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/posix-configs DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

#=============================================================================
# tests
#

#T ODO: find a way to keep this in sync with tests_main
set(tests
	autodeclination
	bson
	commander
	controllib
	conv
	file2
	float
	gpio
	hrt
	hysteresis
	int
	mathlib
	matrix
	mc_pos_control
	mixer
	param
	perf
	rc
	servo
	sf0x
	sleep
	uorb
	)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	list(REMOVE_ITEM tests
		hysteresis
		mixer
	)
endif()

foreach(test_name ${tests})
	configure_file(${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/test_template.in ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/${test_name}_generated)

	add_test(NAME ${test_name}
		COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
			$<TARGET_FILE:px4>
			posix-configs/SITL/init/test
			none
			none
			${test_name}_generated
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR}
			WORKING_DIRECTORY ${SITL_WORKING_DIR})

	set_tests_properties(${test_name} PROPERTIES PASS_REGULAR_EXPRESSION "${test_name} PASSED")
endforeach()

add_custom_target(test_results
		COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure -T Test
		DEPENDS px4
		USES_TERMINAL
		COMMENT "Running tests in sitl"
		WORKING_DIRECTORY ${PX4_BINARY_DIR})

add_custom_target(test_results_junit
		COMMAND xsltproc ${PX4_SOURCE_DIR}/Tools/CTest2JUnit.xsl Testing/`head -n 1 < Testing/TAG`/Test.xml > JUnitTestResults.xml
		DEPENDS test_results
		COMMENT "Converting ctest output to junit xml"
		WORKING_DIRECTORY ${PX4_BINARY_DIR})

add_custom_target(test_cdash_submit
		COMMAND ${CMAKE_CTEST_COMMAND} -D Experimental
		USES_TERMINAL
		WORKING_DIRECTORY ${PX4_BINARY_DIR})

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
