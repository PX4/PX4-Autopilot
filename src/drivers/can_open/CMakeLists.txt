############################################################################
#
#   Copyright (c) 2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
set(OD "demo_device")

if(${PX4_PLATFORM} MATCHES "nuttx")
	if(${CONFIG_NET_CAN} MATCHES "y")
		set(CANOPEN_DRIVER "socket_can")
	elseif(${CONFIG_CAN} MATCHES "y")
		set(CANOPEN_DRIVER "char_can")
	endif()

	if(NOT DEFINED CANOPEN_DRIVER)
		message(FATAL_ERROR "CANOPEN_DRIVER not set")
	endif()

	if(${CANOPEN_DRIVER} MATCHES "socket_can")
		list(APPEND SRCS
			CO_driver_nuttx_socket.c
		)
	elseif(${CANOPEN_DRIVER} MATCHES "char_can")
		list(APPEND SRCS
			CO_driver_nuttx_char.c
		)
	endif()

	list(APPEND SRCS
		CO_driver_target.h
		CO_storage_nuttx.c
		CO_storage_nuttx.h
	)
endif()

if(${OD} MATCHES "demo_device")
    list(APPEND SRCS
		can_open_demo/demo/OD.h
                can_open_demo/demo/OD.c
     	        can_open_demo/demo/objectAccessOD.c
                can_open_demo/demo/objectAccessOD.h
                can_open_demo/demo/domainDemo.c
                can_open_demo/demo/domainDemo.h
                can_open_demo/demo/CO_identificators.c
                can_open_demo/demo/CO_identificators.h
                can_open_demo/demo/CO_application.c
    )
    add_definitions(
	-DCANOPENNODE_DEMO_DEVICE
    )
endif()

add_compile_options(
	# todo: needs a fix in canopennode code.
	-Wno-bad-function-cast
)

px4_add_module(
	MODULE drivers__can_open
	MAIN canopen
	STACK_MAIN 4096
	COMPILE_FLAGS
		-DDEBUG_BUILD
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/demoDevice
		${CMAKE_CURRENT_SOURCE_DIR}/can_open_node
		${CMAKE_CURRENT_SOURCE_DIR}/can_open_node/storage
		${CMAKE_CURRENT_SOURCE_DIR}/can_open_demo/demo
		${CMAKE_CURRENT_SOURCE_DIR}/can_open_demo/demo/linux
	SRCS
		OD.h
		can_open_node/CANopen.c
		can_open_node/CANopen.h
		${SRCS}
		can_open_node/301/CO_config.h
		can_open_node/301/CO_driver.h
		can_open_node/301/CO_ODinterface.c
		can_open_node/301/CO_ODinterface.h
		can_open_node/301/CO_NMT_Heartbeat.c
		can_open_node/301/CO_NMT_Heartbeat.h
		can_open_node/301/CO_HBconsumer.c
		can_open_node/301/CO_HBconsumer.h
		can_open_node/301/CO_Emergency.c
		can_open_node/301/CO_Emergency.h
		can_open_node/301/CO_fifo.c
		can_open_node/301/CO_fifo.h
		can_open_node/301/CO_SDOclient.c
		can_open_node/301/CO_SDOclient.h
		can_open_node/301/CO_SDOserver.c
		can_open_node/301/CO_SDOserver.h
		can_open_node/301/CO_TIME.c
		can_open_node/301/CO_TIME.h
		can_open_node/301/CO_SYNC.c
		can_open_node/301/CO_SYNC.h
		can_open_node/301/CO_PDO.c
		can_open_node/301/CO_PDO.h
		can_open_node/301/crc16-ccitt.c
		can_open_node/301/crc16-ccitt.h
		can_open_node/storage/CO_storage.c
		can_open_node/storage/CO_storage.h
		can_open_node.cpp
		can_open_node.hpp

	MODULE_CONFIG
		module.yaml
	)
