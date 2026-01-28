
# Build the user side px4_layer

add_library(px4_layer
	board_dma_alloc.c
	board_fat_dma_alloc.c
	tasks.cpp
	console_buffer_usr.cpp
	${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/print_load.cpp
	${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/cpuload.cpp
	px4_userspace_init.cpp
	px4_usr_crypto.cpp
	px4_mtd.cpp
	usr_board_ctrl.c
	usr_hrt.cpp
	usr_mcu_version.cpp
	${PX4_SOURCE_DIR}/platforms/common/Serial.cpp
	SerialImpl.cpp
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
	board_ioctl.c
	hrt_ioctl.c
)

add_dependencies(px4_board_ctrl nuttx_context px4_kernel_builtin_list_target)
target_compile_options(px4_board_ctrl PRIVATE -D__KERNEL__)

target_link_libraries(px4_layer
	PUBLIC
		board_bus_info
)

# Build the kernel side px4_kernel_layer

add_library(px4_kernel_layer
	${KERNEL_SRCS}
	SerialImpl.cpp
)

target_link_libraries(px4_kernel_layer
	PRIVATE
		${KERNEL_LIBS}
		nuttx_kc
		nuttx_karch
		nuttx_kmm
	PRIVATE
		kernel_events_interface # events_ioctl_init
)

target_link_libraries(px4_kernel_layer
	PUBLIC
		board_bus_info
)

if (DEFINED PX4_CRYPTO)
	target_link_libraries(px4_kernel_layer PUBLIC crypto_backend)
	target_link_libraries(px4_layer PUBLIC crypto_backend_interface)
endif()

add_dependencies(px4_kernel_layer prebuild_targets)
target_compile_options(px4_kernel_layer PRIVATE -D__KERNEL__)
target_link_libraries(px4_kernel_layer PUBLIC px4_board_ctrl)
