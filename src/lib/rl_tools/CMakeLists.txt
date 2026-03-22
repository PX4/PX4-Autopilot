if(CONFIG_LIB_RL_TOOLS)
	px4_add_git_submodule(TARGET git_rl_tools PATH "rl_tools")
	add_library(rl_tools INTERFACE)
	target_include_directories(rl_tools INTERFACE
		${CMAKE_CURRENT_SOURCE_DIR}/rl_tools/include
	)

	target_compile_features(rl_tools INTERFACE cxx_std_17)

	target_compile_options(rl_tools INTERFACE
		-Wno-unused-parameter
		-Wno-unused-variable
		-Wno-unused-local-typedefs
	)
endif()
