#! /bin/sh

killall JLinkGDBServerCLExe

# TODO: set device from nuttx config
# eg CONFIG_ARCH_CHIP_STM32F427V or CONFIG_STM32_STM32F427
JLinkGDBServerCLExe -device STM32F427II -select usb -silent -endian little -if SWD -speed auto -ir -LocalhostOnly 1 -strict -vd -singlerun &
