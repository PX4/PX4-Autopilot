px4_add_library(GL40IIProtocol
	GL40IIProtocol.cpp
	GL40IIProtocol.hpp
)
target_include_directories(GL40IIProtocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(
	SRC GL40IIProtocolTest.cpp
	LINKLIBS GL40IIProtocol
)

if("${PX4_PLATFORM}" STREQUAL "nuttx")
	px4_add_module(
		MODULE drivers__gl40ii
		MAIN gl40ii
		STACK_MAIN 3500
		COMPILE_FLAGS
			-DUAVCAN_STM32H7_NUTTX=1
			-DUAVCAN_STM32H7_NUM_IFACES=2
			-DUAVCAN_STM32H7_TIMER_NUMBER=2
			-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
			-DUAVCAN_IMPLEMENT_PLACEMENT_NEW=1
			-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
			-DUAVCAN_NO_ASSERTIONS
			-DUAVCAN_PLATFORM=generic
		SRCS
			GL40IIDriver.cpp
			GL40IIDriver.hpp
		INCLUDES
			${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan/libuavcan/include
			${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan/libuavcan/include/dsdlc_generated
			${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include
		MODULE_CONFIG module.yaml
		DEPENDS
			GL40IIProtocol
			px4_work_queue
	)
endif()
