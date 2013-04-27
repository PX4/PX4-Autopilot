#
# Makefile for the px4fmu_default configuration
#

#
# Use the configuration's ROMFS.
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_common

#
# Board support modules
#
MODULES		+= drivers/boards/px4fmuv2
MODULES		+= drivers/lsm303d
MODULES		+= drivers/l3gd20
MODULES		+= drivers/px4fmu
MODULES		+= drivers/rgbled

#
# System commands
#
MODULES		+= systemcmds/ramtron
MODULES		+= systemcmds/bl_update
MODULES		+= systemcmds/boardinfo
MODULES		+= systemcmds/mixer
MODULES		+= systemcmds/param
MODULES		+= systemcmds/perf
MODULES		+= systemcmds/preflight_check
MODULES		+= systemcmds/pwm
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top

#
# General system control
#
MODULES		+= modules/commander
MODULES		+= modules/mavlink
MODULES		+= modules/mavlink_onboard

#
# Estimation modules (EKF / other filters)
#
MODULES		+= modules/attitude_estimator_ekf

#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

#                  command                 priority                   stack  entrypoint
BUILTIN_COMMANDS := \
	$(call _B, adc,                    ,                          2048,  adc_main                   ) \
	$(call _B, blinkm,                 ,                          2048,  blinkm_main                ) \
	$(call _B, control_demo,           ,                          2048,  control_demo_main          ) \
	$(call _B, fixedwing_att_control,  SCHED_PRIORITY_MAX-30,     2048,  fixedwing_att_control_main ) \
	$(call _B, fixedwing_pos_control,  SCHED_PRIORITY_MAX-30,     2048,  fixedwing_pos_control_main ) \
	$(call _B, gps,                    ,                          2048,  gps_main                   ) \
	$(call _B, hil,                    ,                          2048,  hil_main                   ) \
	$(call _B, hott_telemetry,         ,                          2048,  hott_telemetry_main        ) \
	$(call _B, kalman_demo,            SCHED_PRIORITY_MAX-30,     2048,  kalman_demo_main           ) \
	$(call _B, math_demo,              ,                          8192,  math_demo_main             ) \
	$(call _B, multirotor_att_control, SCHED_PRIORITY_MAX-15,     2048,  multirotor_att_control_main) \
	$(call _B, multirotor_pos_control, SCHED_PRIORITY_MAX-25,     2048,  multirotor_pos_control_main) \
	$(call _B, position_estimator,     ,                          4096,  position_estimator_main    ) \
	$(call _B, sdlog,                  SCHED_PRIORITY_MAX-30,     2048,  sdlog_main                 ) \
	$(call _B, sensors,                SCHED_PRIORITY_MAX-5,      4096,  sensors_main               ) \
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                ) \
	$(call _B, tests,                  ,                          12000, tests_main                 ) \
	$(call _B, tone_alarm,             ,                          2048,  tone_alarm_main            ) \
	$(call _B, uorb,                   ,                          4096,  uorb_main                  )
