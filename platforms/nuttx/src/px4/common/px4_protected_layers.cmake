
# Build the user side px4_layer

add_library(px4_layer
	board_dma_alloc.c
	board_fat_dma_alloc.c
	tasks.cpp
	console_buffer_usr.cpp
	usr_mcu_version.cpp
	cdc_acm_check.cpp
	${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/print_load.cpp
	${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/cpuload.cpp
	usr_hrt.cpp
	px4_userspace_init.cpp
)

target_link_libraries(px4_layer
	PRIVATE
		m
		nuttx_c
		nuttx_xx
		nuttx_mm
)

# Build the interface library between user and kernel side
add_library(px4_board_ctrl
		board_ctrl.c
)

add_dependencies(px4_board_ctrl nuttx_context px4_kernel_builtin_list_target)

target_link_libraries(px4_layer
	PUBLIC
		board_bus_info
)

# Build the kernel side px4_kernel_layer

add_library(px4_kernel_layer
		${KERNEL_SRCS}
)

target_link_libraries(px4_kernel_layer
	PRIVATE
		${KERNEL_LIBS}
		nuttx_kc
		nuttx_karch
		nuttx_kmm
)

target_link_libraries(px4_kernel_layer
	PUBLIC
		board_bus_info
)

if (DEFINED PX4_CRYPTO)
	target_link_libraries(px4_kernel_layer PUBLIC crypto_backend)
endif()

target_compile_options(px4_kernel_layer PRIVATE -D__KERNEL__)

add_dependencies(px4_kernel_layer prebuild_targets)
