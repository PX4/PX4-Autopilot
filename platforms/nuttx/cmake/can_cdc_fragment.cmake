############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

# Inject NuttX SocketCAN + dual CDC/ACM into the working .config so every
# STM32H7 UAVCAN board gets DroneCAN GUI Tool support without copying
# defconfig snippets. Appended after the board defconfig is copied and
# before olddefconfig. Also written to px4_can_cdc.fragment so the
# NuttX build-time cat of extra_config_options does not drop the options.

set(PX4_NUTTX_CAN_CDC_FRAGMENT ${PX4_BINARY_DIR}/NuttX/px4_can_cdc.fragment)
file(MAKE_DIRECTORY ${PX4_BINARY_DIR}/NuttX)

set(_px4_can_cdc_fragment "")

if(NOT NUTTX_CONFIG STREQUAL "bootloader" AND NOT NUTTX_CONFIG STREQUAL "canbootloader")
	file(READ ${NUTTX_DEFCONFIG} _px4_nuttx_defconfig)

	set(_px4_h7 FALSE)
	if(_px4_nuttx_defconfig MATCHES "CONFIG_ARCH_CHIP_STM32H7=y")
		set(_px4_h7 TRUE)
	endif()

	set(_px4_uavcan FALSE)
	if(CONFIG_DRIVERS_UAVCAN OR CONFIG_DRIVERS_UAVCANNODE)
		set(_px4_uavcan TRUE)
	endif()

	set(_px4_gpio_can1 FALSE)
	set(_px4_gpio_can2 FALSE)

	foreach(_px4_hdr IN ITEMS
			${PX4_BOARD_DIR}/nuttx-config/include/board.h
			${PX4_BOARD_DIR}/src/board_config.h)
		if(EXISTS ${_px4_hdr})
			set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_px4_hdr})
			file(READ ${_px4_hdr} _px4_hdr_text)
			if(_px4_hdr_text MATCHES "GPIO_CAN1_TX")
				set(_px4_gpio_can1 TRUE)
			endif()
			if(_px4_hdr_text MATCHES "GPIO_CAN2_TX")
				set(_px4_gpio_can2 TRUE)
			endif()
		endif()
	endforeach()

	if(CONSTRAINED_FLASH)
		message(STATUS "NuttX: skipping SocketCAN/dual-CDC on constrained-flash ${PX4_CONFIG}")
	elseif(_px4_h7 AND _px4_uavcan AND (_px4_gpio_can1 OR _px4_gpio_can2))
		message(STATUS "NuttX: enabling SocketCAN FDCAN for ${PX4_CONFIG}")

		set(_px4_can_cdc_fragment "CONFIG_NET=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_CAN_CONNS=8\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NETDEV_CAN_BITRATE_IOCTL=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NETDEV_IFINDEX=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NETDEV_LATEINIT=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NET_CAN=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NET_CAN_CANFD=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NET_CAN_EXTID=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NET_CAN_NOTIFIER=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NET_CAN_RAW_FILTER_MAX=4\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NET_CAN_RAW_TX_DEADLINE=y\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_NET_CAN_SOCK_OPTS=y\n")
		# SO_TIMESTAMP stamps into d_appdata[d_len] in NuttX can_callback().
		# On STM32H7 FDCAN HPWORK that pointer can be NULL and hardfaults
		# wq:uavcan (IMPRECISERR). UAVCAN falls back to the local clock.
		string(APPEND _px4_can_cdc_fragment "# CONFIG_NET_TIMESTAMP is not set\n")
		string(APPEND _px4_can_cdc_fragment "# CONFIG_STM32H7_FDCAN_LPWORK is not set\n")
		string(APPEND _px4_can_cdc_fragment "CONFIG_STM32H7_FDCAN_HPWORK=y\n")

		if(_px4_gpio_can1)
			string(APPEND _px4_can_cdc_fragment "CONFIG_STM32H7_FDCAN1=y\n")
		endif()
		if(_px4_gpio_can2)
			string(APPEND _px4_can_cdc_fragment "CONFIG_STM32H7_FDCAN2=y\n")
		endif()

		# CAN-only boards must not pull in the default IPv4/Ethernet stack.
		if(NOT _px4_nuttx_defconfig MATCHES "CONFIG_STM32H7_ETHMAC=y"
				AND NOT _px4_nuttx_defconfig MATCHES "CONFIG_NET_ETHERNET=y"
				AND NOT _px4_nuttx_defconfig MATCHES "CONFIG_NET_IPv4=y")
			string(APPEND _px4_can_cdc_fragment "# CONFIG_NET_ETHERNET is not set\n")
			string(APPEND _px4_can_cdc_fragment "# CONFIG_NET_IPv4 is not set\n")
		endif()

		set(_px4_dual_cdc FALSE)
		if(CONFIG_DRIVERS_CDCACM_AUTOSTART
				AND _px4_nuttx_defconfig MATCHES "CONFIG_CDCACM=y"
				AND (_px4_nuttx_defconfig MATCHES "CONFIG_STM32H7_OTGFS=y"
					OR _px4_nuttx_defconfig MATCHES "CONFIG_STM32H7_OTGHS=y")
				AND NOT _px4_nuttx_defconfig MATCHES "CONFIG_USBDEV_COMPOSITE=y")
			set(_px4_dual_cdc TRUE)
		endif()

		if(_px4_dual_cdc)
			message(STATUS "NuttX: enabling dual CDC/ACM (MAVLink + SLCAN) for ${PX4_CONFIG}")

			set(_px4_cdc_vid "0x26ac")
			set(_px4_cdc_pid "0x0011")
			set(_px4_cdc_vstr "\"PX4\"")
			set(_px4_cdc_pstr "\"PX4 BOARD\"")

			if(_px4_nuttx_defconfig MATCHES "CONFIG_CDCACM_VENDORID=([^\n\r]+)")
				set(_px4_cdc_vid ${CMAKE_MATCH_1})
			endif()
			if(_px4_nuttx_defconfig MATCHES "CONFIG_CDCACM_PRODUCTID=([^\n\r]+)")
				set(_px4_cdc_pid ${CMAKE_MATCH_1})
			endif()
			if(_px4_nuttx_defconfig MATCHES "CONFIG_CDCACM_VENDORSTR=([^\n\r]+)")
				set(_px4_cdc_vstr ${CMAKE_MATCH_1})
			endif()
			if(_px4_nuttx_defconfig MATCHES "CONFIG_CDCACM_PRODUCTSTR=([^\n\r]+)")
				set(_px4_cdc_pstr ${CMAKE_MATCH_1})
			endif()

			string(APPEND _px4_can_cdc_fragment "CONFIG_BOARDCTL=y\n")
			string(APPEND _px4_can_cdc_fragment "CONFIG_BOARDCTL_USBDEVCTRL=y\n")
			string(APPEND _px4_can_cdc_fragment "CONFIG_USBDEV_COMPOSITE=y\n")
			string(APPEND _px4_can_cdc_fragment "CONFIG_CDCACM_COMPOSITE=y\n")
			string(APPEND _px4_can_cdc_fragment "CONFIG_COMPOSITE_IAD=y\n")
			string(APPEND _px4_can_cdc_fragment "CONFIG_COMPOSITE_VENDORID=${_px4_cdc_vid}\n")
			string(APPEND _px4_can_cdc_fragment "CONFIG_COMPOSITE_PRODUCTID=${_px4_cdc_pid}\n")
			string(APPEND _px4_can_cdc_fragment "CONFIG_COMPOSITE_VENDORSTR=${_px4_cdc_vstr}\n")
			string(APPEND _px4_can_cdc_fragment "CONFIG_COMPOSITE_PRODUCTSTR=${_px4_cdc_pstr}\n")
		endif()
	endif()
endif()

file(WRITE ${PX4_NUTTX_CAN_CDC_FRAGMENT} "${_px4_can_cdc_fragment}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PX4_NUTTX_CAN_CDC_FRAGMENT})

if(NOT _px4_can_cdc_fragment STREQUAL "")
	file(APPEND ${NUTTX_DIR}/.config "\n${_px4_can_cdc_fragment}")
endif()
