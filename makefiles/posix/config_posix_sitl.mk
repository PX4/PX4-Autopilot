#
# Makefile for the SITL configuration
#

#
# Board support modules
#
MODULES		+= drivers/boards/sitl
MODULES		+= drivers/device
MODULES		+= drivers/blinkm
MODULES		+= drivers/pwm_out_sim
MODULES		+= drivers/rgbled
MODULES		+= drivers/led
MODULES		+= modules/sensors
MODULES		+= drivers/ms5611

#
# System commands
#
MODULES		+= systemcmds/param
MODULES		+= systemcmds/mixer
#MODULES 	+= systemcmds/esc_calib
MODULES		+= systemcmds/tests
#MODULES 	+= systemcmds/reboot
MODULES 	+= systemcmds/topic_listener
MODULES		+= systemcmds/ver
MODULES		+= systemcmds/esc_calib
MODULES		+= systemcmds/reboot

#
# General system control
#
MODULES		+= modules/mavlink

#
# Estimation modules (EKF/ SO3 / other filters)
#
MODULES		+= modules/attitude_estimator_ekf
MODULES		+= modules/attitude_estimator_q
MODULES		+= modules/ekf_att_pos_estimator
MODULES		+= modules/attitude_estimator_q
MODULES		+= modules/position_estimator_inav

#
# Vehicle Control
#
MODULES 	+= modules/navigator
MODULES 	+= modules/mc_pos_control
MODULES		+= modules/mc_att_control
MODULES 	+= modules/mc_pos_control_multiplatform
MODULES		+= modules/mc_att_control_multiplatform
MODULES		+= modules/land_detector
MODULES		+= modules/fw_att_control
MODULES		+= modules/fw_pos_control_l1



#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/uORB
MODULES		+= modules/dataman
MODULES		+= modules/sdlog2
MODULES		+= modules/simulator
MODULES		+= modules/commander
MODULES 	+= modules/controllib

#
# Libraries
#
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/ecl
MODULES		+= lib/external_lgpl
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion
MODULES		+= lib/launchdetection


#
# POSIX port
#
MODULES		+= platforms/posix/px4_layer
MODULES		+= platforms/posix/work_queue
MODULES		+= platforms/posix/drivers/accelsim
MODULES		+= platforms/posix/drivers/gyrosim
MODULES		+= platforms/posix/drivers/adcsim
MODULES		+= platforms/posix/drivers/barosim
MODULES		+= platforms/posix/drivers/tonealrmsim
MODULES		+= platforms/posix/drivers/airspeedsim
MODULES 	+= platforms/posix/drivers/gpssim

#
# Unit tests
#
#MODULES	+= platforms/posix/tests/hello
MODULES	+= platforms/posix/tests/vcdev_test
#MODULES	+= platforms/posix/tests/hrt_test
#MODULES	+= platforms/posix/tests/wqueue

#
# muorb fastrpc changes.
#
#MODULES	+= $(PX4_BASE)../muorb_krait
