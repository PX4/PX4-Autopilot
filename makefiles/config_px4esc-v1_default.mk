#
# Makefile for the px4esc_default configuration
#

#
# Board support modules
#
MODULES		+= drivers/stm32
MODULES		+= drivers/stm32/adc
MODULES		+= drivers/led
MODULES		+= drivers/boards/px4esc-v1

#
# System commands
#
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
MODULES		+= systemcmds/config
MODULES		+= systemcmds/nshterm
MODULES		+= systemcmds/ver


#
# Unit tests
#
#MODULES 	+= modules/unit_test
#MODULES 	+= modules/commander/commander_tests

#
# Demo apps
#

