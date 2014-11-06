
#ifndef __MAVSTATION_FIRMWARE_SYS_STATE_H__
#define __MAVSTATION_FIRMWARE_SYS_STATE_H__

/*
 * System state structure.
 */
struct sys_state_s {

	volatile uint64_t	rc_channels_timestamp;

	/**
	 * Last FMU receive time, in microseconds since system boot
	 */
	volatile uint64_t	fmu_data_received_time;

};

extern struct sys_state_s system_state;

#endif // __MAVSTATION_FIRMWARE_SYS_STATE_H__
