include(px4_add_library)

px4_add_git_submodule(TARGET git_raptor_blob PATH "blob")

px4_add_module(
	MODULE modules__mc_raptor
	MAIN mc_raptor
	STACK_MAIN 4000
	SRCS
		mc_raptor.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
		rl_tools
		git_raptor_blob
	EXTERNAL
	)

target_compile_features(modules__mc_raptor PRIVATE cxx_std_17)
target_compile_options(modules__mc_raptor PRIVATE -Wno-unused-result)
