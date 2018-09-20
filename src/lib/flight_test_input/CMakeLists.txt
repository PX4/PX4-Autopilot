
set(generated_params_dir ${PX4_BINARY_DIR}/generated_params)
set(generated_fti_params_file ${generated_params_dir}/flight_test_input_params.c)
add_custom_command(OUTPUT ${generated_fti_params_file}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${generated_params_dir}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_parameters.py
		--params-file ${generated_fti_params_file}
		--groups
			ACC1X ACC1Y ACC1Z
			ACC2X ACC2Y ACC2Z
			GYRO1X GYRO1Y GYRO1Z
			GYRO2X GYRO2Y GYRO2Z
			MAG1X MAG1Y MAG1Z
			MAG2X MAG2Y MAG2Z
			BARO
			BAT_V BAT_C
			GPS_EPH GPS_EPV
			RC_R RC_P RC_Y RC_T
			PWM1 PWM2 PWM3 PWM4
		--verbose
	DEPENDS
		${CMAKE_CURRENT_SOURCE_DIR}/generate_parameters.py
	COMMENT "Generating flight_test_input_params.c"
)
add_custom_target(flight_test_input_parameters DEPENDS ${generated_fti_params_file})

px4_add_library(flight_test_input
	flight_test_input.cpp
	)
add_dependencies(flight_test_input flight_test_input_parameters)

target_compile_options(flight_test_input PRIVATE -Wno-float-equal)
