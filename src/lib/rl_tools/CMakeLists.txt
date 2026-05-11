if(CONFIG_LIB_RL_TOOLS)
	px4_add_git_submodule(TARGET git_rl_tools PATH "rl_tools")
	add_library(rl_tools INTERFACE)
	target_include_directories(rl_tools INTERFACE
		${CMAKE_CURRENT_SOURCE_DIR}/rl_tools/include
	)

	target_compile_features(rl_tools INTERFACE cxx_std_17)

	px4_target_compile_options_for_compiler(rl_tools INTERFACE
		GNU_LIKE
			-Wno-unused-parameter
			-Wno-unused-variable
			-Wno-unused-local-typedefs
	)
endif()
