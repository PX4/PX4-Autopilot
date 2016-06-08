/**
 * @file aerocore_rc.h
 *
 * General defines and structures for the AeroCore RC module.
 * Copied from px4io.h firmware
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <board_config.h>
#include <systemlib/pwm_limit/pwm_limit.h>

/*
 * Constants and limits.
 */
#define PX4IO_CONTROL_CHANNELS		8
#define PX4IO_RC_INPUT_CHANNELS		18
#define PX4IO_RC_MAPPED_CONTROL_CHANNELS		8 /**< This is the maximum number of channels mapped/used */


/* array of raw RC input values, microseconds */
#define PX4IO_P_RAW_RC_NRSSI			2	/* [2] Normalized RSSI value, 0: no reception, 255: perfect reception */
#define PX4IO_P_RAW_RC_DATA			3	/* [1] + [2] Details about the RC source (PPM frame length, Spektrum protocol type) */
#define PX4IO_P_RAW_FRAME_COUNT			4	/* Number of total received frames (wrapping counter) */
#define PX4IO_P_RAW_LOST_FRAME_COUNT		5	/* Number of total dropped frames (wrapping counter) */
#define PX4IO_P_RAW_RC_BASE			6	/* CONFIG_RC_INPUT_COUNT channels from here */

/* array of scaled RC input values, -10000..10000 */
#define PX4IO_PAGE_RC_INPUT		5
#define PX4IO_P_RC_VALID			0	/* bitmask of valid controls */
#define PX4IO_P_RC_BASE				1	/* CONFIG_RC_INPUT_COUNT controls from here */

/* array of raw RC input values, microseconds */
#define PX4IO_PAGE_RAW_RC_INPUT		4
#define PX4IO_P_RAW_RC_COUNT			0	/* number of valid channels */
#define PX4IO_P_RAW_RC_FLAGS			1	/* RC detail status flags */
#define PX4IO_P_RAW_RC_FLAGS_FRAME_DROP		(1 << 0) /* single frame drop */
#define PX4IO_P_RAW_RC_FLAGS_FAILSAFE		(1 << 1) /* receiver is in failsafe mode */
#define PX4IO_P_RAW_RC_FLAGS_RC_DSM11		(1 << 2) /* DSM decoding is 11 bit mode */
#define PX4IO_P_RAW_RC_FLAGS_MAPPING_OK		(1 << 3) /* Channel mapping is ok */
#define PX4IO_P_RAW_RC_FLAGS_RC_OK		(1 << 4) /* RC reception ok */

/* R/C channel config */
#define PX4IO_PAGE_RC_CONFIG			53		/**< R/C input configuration */
#define PX4IO_P_RC_CONFIG_MIN			0		/**< lowest input value */
#define PX4IO_P_RC_CONFIG_CENTER		1		/**< center input value */
#define PX4IO_P_RC_CONFIG_MAX			2		/**< highest input value */
#define PX4IO_P_RC_CONFIG_DEADZONE		3		/**< band around center that is ignored */
#define PX4IO_P_RC_CONFIG_ASSIGNMENT		4		/**< mapped input value */
#define PX4IO_P_RC_CONFIG_OPTIONS		5		/**< channel options bitmask */
#define PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH	100
#define PX4IO_P_RC_CONFIG_OPTIONS_ENABLED	(1 << 0)
#define PX4IO_P_RC_CONFIG_OPTIONS_REVERSE	(1 << 1)
#define PX4IO_P_RC_CONFIG_STRIDE		6		/**< spacing between channel config data */

/* dynamic status page */
#define PX4IO_PAGE_STATUS		1
#define PX4IO_P_STATUS_FREEMEM			0
#define PX4IO_P_STATUS_CPULOAD			1

#define PX4IO_P_STATUS_VBATT			4	/* [1] battery voltage in mV */
#define PX4IO_P_STATUS_IBATT			5	/* [1] battery current (raw ADC) */
#define PX4IO_P_STATUS_VSERVO			6	/* [2] servo rail voltage in mV */
#define PX4IO_P_STATUS_VRSSI			7	/* [2] RSSI voltage */
#define PX4IO_P_STATUS_PRSSI			8	/* [2] RSSI PWM value */

#define PX4IO_P_STATUS_FLAGS			2	 /* monitoring flags */
#define PX4IO_P_STATUS_FLAGS_RC_OK		(1 << 2) /* RC input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_PPM		(1 << 3) /* PPM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_DSM		(1 << 4) /* DSM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_SBUS		(1 << 5) /* SBUS input is valid */
#define PX4IO_P_STATUS_FLAGS_OVERRIDE		(1 << 1) /* in manual override */

#define PX4IO_P_STATUS_ALARMS			3	 /* alarm flags - alarms latch, write 1 to a bit to clear it */
#define PX4IO_P_STATUS_ALARMS_VBATT_LOW		(1 << 0) /* [1] VBatt is very close to regulator dropout */
#define PX4IO_P_STATUS_ALARMS_TEMPERATURE	(1 << 1) /* board temperature is high */
#define PX4IO_P_STATUS_ALARMS_SERVO_CURRENT	(1 << 2) /* [1] servo current limit was exceeded */
#define PX4IO_P_STATUS_ALARMS_ACC_CURRENT	(1 << 3) /* [1] accessory current limit was exceeded */
#define PX4IO_P_STATUS_ALARMS_FMU_LOST		(1 << 4) /* timed out waiting for controls from FMU */
#define PX4IO_P_STATUS_ALARMS_RC_LOST		(1 << 5) /* timed out waiting for RC input */
#define PX4IO_P_STATUS_ALARMS_PWM_ERROR		(1 << 6) /* PWM configuration or output was bad */
#define PX4IO_P_STATUS_ALARMS_VSERVO_FAULT	(1 << 7) /* [2] VServo was out of the valid range (2.5 - 5.5 V) */

#define PX4IO_P_SETUP_RC_THR_FAILSAFE_US	13	/**< the throttle failsafe pulse length in microseconds */

#define PX4IO_P_SETUP_ARMING			1	 /* arming controls */
#define PX4IO_P_SETUP_ARMING_IO_ARM_OK		(1 << 0) /* OK to arm the IO side */
#define PX4IO_P_SETUP_ARMING_FMU_ARMED		(1 << 1) /* FMU is already armed */
#define PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK	(1 << 2) /* OK to switch to manual override via override RC channel */
#define PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM	(1 << 3) /* use custom failsafe values, not 0 values of mixer */
#define PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK	(1 << 4) /* OK to try in-air restart */
#define PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE	(1 << 5) /* Output of PWM right after startup enabled to help ESCs initialize and prevent them from beeping */
#define PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED	(1 << 6) /* Disable the IO-internal evaluation of the RC */
#define PX4IO_P_SETUP_ARMING_LOCKDOWN		(1 << 7) /* If set, the system operates normally, but won't actuate any servos */

#define PX4IO_P_SETUP_FEATURES			0
#define PX4IO_P_SETUP_FEATURES_SBUS1_OUT	(1 << 0) /**< enable S.Bus v1 output */
#define PX4IO_P_SETUP_FEATURES_SBUS2_OUT	(1 << 1) /**< enable S.Bus v2 output */
#define PX4IO_P_SETUP_FEATURES_PWM_RSSI		(1 << 2) /**< enable PWM RSSI parsing */
#define PX4IO_P_SETUP_FEATURES_ADC_RSSI		(1 << 3) /**< enable ADC RSSI parsing */

#define PX4IO_P_SETUP_ARMING			1	 /* arming controls */
#define PX4IO_P_SETUP_ARMING_IO_ARM_OK		(1 << 0) /* OK to arm the IO side */
#define PX4IO_P_SETUP_ARMING_FMU_ARMED		(1 << 1) /* FMU is already armed */
#define PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK	(1 << 2) /* OK to switch to manual override via override RC channel */
#define PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM	(1 << 3) /* use custom failsafe values, not 0 values of mixer */
#define PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK	(1 << 4) /* OK to try in-air restart */
#define PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE	(1 << 5) /* Output of PWM right after startup enabled to help ESCs initialize and prevent them from beeping */
#define PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED	(1 << 6) /* Disable the IO-internal evaluation of the RC */
#define PX4IO_P_SETUP_ARMING_LOCKDOWN		(1 << 7) /* If set, the system operates normally, but won't actuate any servos */

#define PX4IO_P_SETUP_PWM_RATES			2	/* bitmask, 0 = low rate, 1 = high rate */
#define PX4IO_P_SETUP_PWM_DEFAULTRATE		3	/* 'low' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_PWM_ALTRATE		4	/* 'high' PWM frame output rate in Hz */

#define PX4IO_P_SETUP_VBATT_SCALE		6	/* hardware rev [1] battery voltage correction factor (float) */
#define PX4IO_P_SETUP_VSERVO_SCALE		6	/* hardware rev [2] servo voltage correction factor (float) */
#define PX4IO_P_SETUP_DSM			7	/* DSM bind state */
enum {							/* DSM bind states */
	dsm_bind_power_down = 0,
	dsm_bind_power_up,
	dsm_bind_set_rx_out,
	dsm_bind_send_pulses,
	dsm_bind_reinit_uart
};
			/* 8 */
#define PX4IO_P_SETUP_SET_DEBUG			9	/* debug level for IO board */

#define PX4IO_P_SETUP_REBOOT_BL			10	/* reboot IO into bootloader */
#define PX4IO_REBOOT_BL_MAGIC			14662	/* required argument for reboot (random) */

#define PX4IO_P_SETUP_CRC			11	/* get CRC of IO firmware */
						/* 12 occupied by CRC */
#define PX4IO_P_SETUP_RC_THR_FAILSAFE_US	13	/**< the throttle failsafe pulse length in microseconds */

# define POWER_SPEKTRUM(_s) stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))
#define REG_TO_SIGNED(_reg) ((int16_t)(_reg))
#define SIGNED_TO_REG(_signed) ((uint16_t)(_signed))

/*
 * Registers.
 */
extern uint16_t			r_page_status[];	/* PX4IO_PAGE_STATUS */
extern uint16_t			r_page_raw_rc_input[];	/* PX4IO_PAGE_RAW_RC_INPUT */
extern uint16_t			r_page_rc_input_config[]; /* PX4IO_PAGE_RC_INPUT_CONFIG */
extern uint16_t			r_page_rc_input[];	/* PX4IO_PAGE_RC_INPUT */

extern volatile uint16_t	r_page_setup[];		/* PX4IO_PAGE_SETUP */

#define r_status_flags		r_page_status[PX4IO_P_STATUS_FLAGS]
#define r_raw_rc_count		r_page_raw_rc_input[PX4IO_P_RAW_RC_COUNT]
#define r_raw_rc_values		(&r_page_raw_rc_input[PX4IO_P_RAW_RC_BASE])
#define r_raw_rc_flags		r_page_raw_rc_input[PX4IO_P_RAW_RC_FLAGS]
#define r_setup_rc_thr_failsafe	r_page_setup[PX4IO_P_SETUP_RC_THR_FAILSAFE_US]
#define r_rc_values		(&r_page_rc_input[PX4IO_P_RC_BASE])
#define r_rc_valid		r_page_rc_input[PX4IO_P_RC_VALID]
#define r_status_alarms		r_page_status[PX4IO_P_STATUS_ALARMS]
#define r_setup_arming		r_page_setup[PX4IO_P_SETUP_ARMING]

/*
 * System state structure.
 */
struct sys_state_s {

	volatile uint64_t	rc_channels_timestamp_received;
	volatile uint64_t	rc_channels_timestamp_valid;

	/**
	 * Last FMU receive time, in microseconds since system boot
	 */
	volatile uint64_t	fmu_data_received_time;

};
extern struct sys_state_s system_state;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * R/C receiver handling.
 *
 * Input functions return true when they receive an update from the RC controller.
 */

extern void	controls_init(void);
extern void	controls_tick(void);
extern int	dsm_init(const char *device);
extern bool	dsm_input(uint16_t *values, uint16_t *num_values);
extern void	dsm_bind(uint16_t cmd, int pulses);
extern int	sbus_init(const char *device);
extern bool	sbus_input(uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_channels);
extern void	sbus1_output(uint16_t *values, uint16_t num_values);
extern void	sbus2_output(uint16_t *values, uint16_t num_values);
#ifdef __cplusplus
}
#endif
