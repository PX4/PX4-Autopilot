#
# Makefile for the EAGLE *default* configuration
#

#
# Board support modules
#
MODULES		+= drivers/device

#
# System commands
#
MODULES	+= systemcmds/param
MODULES	+= systemcmds/ver

#
# General system control
#
MODULES		+= modules/mavlink

#
# Estimation modules (EKF/ SO3 / other filters)
#

#
# Vehicle Control
#

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/uORB
MODULES		+= modules/dataman

#
# Libraries
#
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion
#
# Linux port
#
MODULES		+= platforms/posix/px4_layer
MODULES		+= platforms/posix/work_queue

#
# Unit tests
#

#
# muorb fastrpc changes.
#
MODULES		+= modules/muorb/krait
