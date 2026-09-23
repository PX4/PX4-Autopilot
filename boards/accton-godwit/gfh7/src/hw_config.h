/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/

#pragma once

#define USB0_DEV    0x01
#define SERIAL0_DEV 0x02

#define APP_LOAD_ADDRESS       0x08020000
#define BOOTLOADER_DELAY       5000
#define INTERFACE_USB          1
#define INTERFACE_USB_CONFIG   "/dev/ttyACM0"
#define INTERFACE_USART        1
#define INTERFACE_USART_CONFIG "/dev/ttyS0,1500000" /* UART7: TEL1 */

#define BOOT_DELAY_ADDRESS     0x000001a0
#define BOARD_TYPE             7122
#define BOARD_FLASH_SECTORS    14
#define BOARD_FLASH_SIZE       (16 * 128 * 1024)
#define APP_RESERVATION_SIZE   (1 * 128 * 1024)
#define OSC_FREQ               16

#define BOARD_PIN_LED_ACTIVITY   GPIO_nLED_BLUE
#define BOARD_PIN_LED_BOOTLOADER GPIO_nLED_GREEN
#define BOARD_LED_ON             0
#define BOARD_LED_OFF            1

#define SERIAL_BREAK_DETECT_DISABLED 1

#if !defined(ARCH_SN_MAX_LENGTH)
# define ARCH_SN_MAX_LENGTH 12
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
# define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 1
#endif

#if !defined(USB_DATA_ALIGN)
# define USB_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
# define BOOT_DEVICES_SELECTION (USB0_DEV | SERIAL0_DEV)
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
# define BOOT_DEVICES_FILTER_ONUSB (USB0_DEV | SERIAL0_DEV)
#endif
