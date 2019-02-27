#! /bin/sh

killall JLinkGDBServerCLExe


# CONFIG_ARCH_CHIP_STM32F427V
# CONFIG_STM32_STM32F427
JLinkGDBServerCLExe -device STM32F427II -select usb -silent -endian little -if SWD -speed auto -ir -LocalhostOnly 1 -strict -vd -singlerun &
