/* Structure for storage of shared variables (extended( */

/* global_data_subsystem_info stores a buffer of the sensor info/status messages (sent by sensors or gps app), this is then handled by the commander. (The commander then writes to global_data_sys_status which is read by mavlink) */

/* (any app) --> global_data_subsystem_info (buffered) --> (commander app)  --> global_data_sys_status --> (mavlink app) */

#ifndef GLOBAL_DATA_SUBSYSTEM_INFO_T_H_ //adjust this line!
#define GLOBAL_DATA_SUBSYSTEM_INFO_T_H_ //adjust this line!

#define SUBSYSTEM_INFO_BUFFER_SIZE 10

#include "global_data_access.h"

typedef enum //values correspond to bitmasks of mavlink message sys_status, this is crucial for the underlying bitmask to work
{
	SUBSYSTEM_TYPE_GYRO = 0,
	SUBSYSTEM_TYPE_ACC = 1,
	SUBSYSTEM_TYPE_MAG = 2,
	SUBSYSTEM_TYPE_ABSPRESSURE = 3,
	SUBSYSTEM_TYPE_DIFFPRESSURE = 4,
	SUBSYSTEM_TYPE_GPS = 5,
	SUBSYSTEM_TYPE_OPTICALFLOW = 6,
	SUBSYSTEM_TYPE_CVPOSITION = 7,
	SUBSYSTEM_TYPE_LASERPOSITION = 8,
	SUBSYSTEM_TYPE_EXTERNALGROUNDTRUTH = 9,
	SUBSYSTEM_TYPE_ANGULARRATECONTROL = 10,
	SUBSYSTEM_TYPE_ATTITUDESTABILIZATION = 11,
	SUBSYSTEM_TYPE_YAWPOSITION = 12,
	SUBSYSTEM_TYPE_ALTITUDECONTROL = 13,
	SUBSYSTEM_TYPE_POSITIONCONTROL = 14,
	SUBSYSTEM_TYPE_MOTORCONTROL = 15

} subsystem_type_t;

typedef struct
{
	uint8_t present;
	uint8_t enabled;
	uint8_t health;

	subsystem_type_t subsystem_type;

} __attribute__((__packed__)) subsystem_info_t;

typedef struct
{
	/* Struct which stores the access configuration, this is the same for all shared structs */

	access_conf_t access_conf; //don't remove this line!

	/* use of a counter and timestamp recommended (but not necessary) */

	uint16_t counter; //incremented by the writing thread everytime new data is stored
	uint64_t timestamp; //in microseconds since system start, is set whenever the writing thread stores new data

	/* Actual data, this is specific to the type of data which is stored in this struct */

	//***** Start: Add your variables here *****
	subsystem_info_t info[SUBSYSTEM_INFO_BUFFER_SIZE];
	uint8_t current_info;

	//*****END: Add your variables here *****

} global_data_subsystem_info_t; //adjust this line!

extern global_data_subsystem_info_t* global_data_subsystem_info; //adjust this line!

#endif
