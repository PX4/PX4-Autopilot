
/*
 * I2C busses
 */
#define PX4_I2C_BUS_ESC		1
#define PX4_SIM_BUS_TEST	2
#define PX4_I2C_BUS_EXPANSION	3
#define PX4_I2C_BUS_LED		3

#define PX4_I2C_OBDEV_LED	0x55

#define BOARD_OVERRIDE_UUID "SIMULATIONID"
#define SIM_FORMATED_UUID "000000010000000200000003"
#define PX4_CPU_UUID_BYTE_LENGTH 12
#define PX4_CPU_UUID_WORD32_LENGTH 3
#define PX4_CPU_UUID_WORD32_LEGACY_FORMAT_SIZE  (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH))

#define BOARD_OVERRIDE_CPU_VERSION (-1)
#define board_mcu_version(rev, revstr, errata) BOARD_OVERRIDE_CPU_VERSION
typedef unsigned char raw_uuid_byte_t[PX4_CPU_UUID_BYTE_LENGTH];
typedef unsigned int raw_uuid_uint32_t[PX4_CPU_UUID_WORD32_LENGTH];

#define board_get_uuid_raw32(id, null) do {id[0]=0;id[1]=1;id[2]=2;} while(0)
#define board_get_uuid_formated32(format_buffer, size, format, seperator, optional_reorder) do { strcpy(format_buffer, SIM_FORMATED_UUID); } while(0)


#define CONFIG_NFILE_STREAMS 1
#define CONFIG_SCHED_WORKQUEUE 1
#define CONFIG_SCHED_HPWORK 1
#define CONFIG_SCHED_LPWORK 1
#define CONFIG_ARCH_BOARD_SITL 1

/** time in ms between checks for work in work queues **/
#define CONFIG_SCHED_WORKPERIOD 50000

#define CONFIG_SCHED_INSTRUMENTATION 1
#define CONFIG_MAX_TASKS 32
