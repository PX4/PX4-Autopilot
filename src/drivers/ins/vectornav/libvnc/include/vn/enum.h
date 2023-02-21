#ifndef VNENUM_H_INCLUDED
#define VNENUM_H_INCLUDED

/** \brief The different types of UART packets. */
typedef enum
{
	PACKETTYPE_BINARY,		/**< Binary packet. */
	PACKETTYPE_ASCII,		/**< ASCII packet. */
	PACKETTYPE_UNKNOWN		/**< Unknown packet type. */
} PacketType;

/** \brief The available binary output groups. */
typedef enum
{
	BINARYGROUPTYPE_COMMON = 0x01,		/**< Common group. */
	BINARYGROUPTYPE_TIME = 0x02,		/**< Time group. */
	BINARYGROUPTYPE_IMU = 0x04,			/**< IMU group. */
	BINARYGROUPTYPE_GPS = 0x08,			/**< GPS group. */
	BINARYGROUPTYPE_ATTITUDE = 0x10,	/**< Attitude group. */
	BINARYGROUPTYPE_INS = 0x20,			/**< INS group. */
	BINARYGROUPTYPE_GPS2 = 0x40			/**< GPS2 group. */
} BinaryGroupType;

/** \brief Async modes for the Binary Output registers. */
typedef enum
{
	ASYNCMODE_NONE	= 0,	/**< None. */
	ASYNCMODE_PORT1	= 1,	/**< Serial port 1. */
	ASYNCMODE_PORT2	= 2,	/**< Serial port 2. */
	ASYNCMODE_BOTH	= 3		/**< Both serial ports. */
} AsyncMode;

/** \brief Flags for the binary group 1 'Common' in the binary output registers. */
typedef enum
{
	COMMONGROUP_NONE				= 0x0000,	/**< None. */
	COMMONGROUP_TIMESTARTUP = 0x0001,	/**< TimeStartup. */
	COMMONGROUP_TIMEGPS				= 0x0002,	/**< TimeGps. */
	COMMONGROUP_TIMESYNCIN			= 0x0004,	/**< TimeSyncIn. */
	COMMONGROUP_YAWPITCHROLL		= 0x0008,	/**< YawPitchRoll. */
	COMMONGROUP_QUATERNION			= 0x0010,	/**< Quaternion. */
	COMMONGROUP_ANGULARRATE			= 0x0020,	/**< AngularRate. */
	COMMONGROUP_POSITION			= 0x0040,	/**< Position. */
	COMMONGROUP_VELOCITY			= 0x0080,	/**< Velocity. */
	COMMONGROUP_ACCEL				= 0x0100,	/**< Accel. */
	COMMONGROUP_IMU					= 0x0200,	/**< Imu. */
	COMMONGROUP_MAGPRES				= 0x0400,	/**< MagPres. */
	COMMONGROUP_DELTATHETA			= 0x0800,	/**< DeltaTheta. */
	COMMONGROUP_INSSTATUS			= 0x1000,	/**< InsStatus. */
	COMMONGROUP_SYNCINCNT			= 0x2000,	/**< SyncInCnt. */
	COMMONGROUP_TIMEGPSPPS			= 0x4000	/**< TimeGpsPps. */
} CommonGroup;

/** \brief Flags for the binary group 2 'Time' in the binary output registers. */
typedef enum
{
	TIMEGROUP_NONE					= 0x0000,	/**< None. */
	TIMEGROUP_TIMESTARTUP			= 0x0001,	/**< TimeStartup. */
	TIMEGROUP_TIMEGPS				= 0x0002,	/**< TimeGps. */
	TIMEGROUP_GPSTOW				= 0x0004,	/**< GpsTow. */
	TIMEGROUP_GPSWEEK				= 0x0008,	/**< GpsWeek. */
	TIMEGROUP_TIMESYNCIN			= 0x0010,	/**< TimeSyncIn. */
	TIMEGROUP_TIMEGPSPPS			= 0x0020,	/**< TimeGpsPps. */
	TIMEGROUP_TIMEUTC				= 0x0040,	/**< TimeUTC. */
	TIMEGROUP_SYNCINCNT				= 0x0080,	/**< SyncInCnt. */
	TIMEGROUP_SYNCOUTCNT				= 0x0100,	/**< SyncOutCnt. */
	TIMEGROUP_TIMESTATUS				= 0x0200	/**< TimeStatus. */
} TimeGroup;

/** \brief Flags for the binary group 3 'IMU' in the binary output registers. */
typedef enum
{
	IMUGROUP_NONE					= 0x0000,	/**< None. */
	IMUGROUP_IMUSTATUS				= 0x0001,	/**< ImuStatus. */
	IMUGROUP_UNCOMPMAG				= 0x0002,	/**< UncompMag. */
	IMUGROUP_UNCOMPACCEL			= 0x0004,	/**< UncompAccel. */
	IMUGROUP_UNCOMPGYRO				= 0x0008,	/**< UncompGyro. */
	IMUGROUP_TEMP					= 0x0010,	/**< Temp. */
	IMUGROUP_PRES					= 0x0020,	/**< Pres. */
	IMUGROUP_DELTATHETA				= 0x0040,	/**< DeltaTheta. */
	IMUGROUP_DELTAVEL				= 0x0080,	/**< DeltaVel. */
	IMUGROUP_MAG					= 0x0100,	/**< Mag. */
	IMUGROUP_ACCEL					= 0x0200,	/**< Accel. */
	IMUGROUP_ANGULARRATE			= 0x0400,	/**< AngularRate. */
	IMUGROUP_SENSSAT				= 0x0800	/**< SensSat. */
	#ifdef EXTRA
	,
	IMUGROUP_RAW					= 0x1000	/**< Raw. */
	#endif
} ImuGroup;

/** \brief Flags for the binary group 4 'GPS' in the binary output registers. */
typedef enum
{
	GPSGROUP_NONE					= 0x0000,	/**< None. */
	GPSGROUP_UTC					= 0x0001,	/**< UTC. */
	GPSGROUP_TOW					= 0x0002,	/**< Tow. */
	GPSGROUP_WEEK					= 0x0004,	/**< Week. */
	GPSGROUP_NUMSATS				= 0x0008,	/**< NumSats. */
	GPSGROUP_FIX					= 0x0010,	/**< Fix. */
	GPSGROUP_POSLLA					= 0x0020,	/**< PosLla. */
	GPSGROUP_POSECEF				= 0x0040,	/**< PosEcef. */
	GPSGROUP_VELNED					= 0x0080,	/**< VelNed. */
	GPSGROUP_VELECEF				= 0x0100,	/**< VelEcef. */
	GPSGROUP_POSU					= 0x0200,	/**< PosU. */
	GPSGROUP_VELU					= 0x0400,	/**< VelU. */
	GPSGROUP_TIMEU					= 0x0800,	/**< TimeU. */
  GPSGROUP_TIMEINFO       = 0x1000,	/**< TimeInfo. */
  GPSGROUP_DOP            = 0x2000	/**< DOP. */

	#ifdef EXTRA
	,
	GPSGROUP_SVSTAT					= 0x1000	/**< SvStat. */
	#endif
} GpsGroup;

/** \brief Flags for the binary group 5 'Attitude' in the binary output registers. */
typedef enum
{
	ATTITUDEGROUP_NONE				= 0x0000,	/**< None. */
	ATTITUDEGROUP_VPESTATUS			= 0x0001,	/**< VpeStatus. */
	ATTITUDEGROUP_YAWPITCHROLL		= 0x0002,	/**< YawPitchRoll. */
	ATTITUDEGROUP_QUATERNION		= 0x0004,	/**< Quaternion. */
	ATTITUDEGROUP_DCM				= 0x0008,	/**< DCM. */
	ATTITUDEGROUP_MAGNED			= 0x0010,	/**< MagNed. */
	ATTITUDEGROUP_ACCELNED			= 0x0020,	/**< AccelNed. */
	ATTITUDEGROUP_LINEARACCELBODY	= 0x0040,	/**< LinearAccelBody. */
	ATTITUDEGROUP_LINEARACCELNED	= 0x0080,	/**< LinearAccelNed. */
	ATTITUDEGROUP_YPRU				= 0x0100	/**< YprU. */
	#ifdef EXTRA
	,
	ATTITUDEGROUP_YPRRATE			= 0x0200,	/**< YprRate. */
	ATTITUDEGROUP_STATEAHRS			= 0x0400,	/**< StateAhrs. */
	ATTITUDEGROUP_COVAHRS			= 0x0800	/**< CovAhrs. */
	#endif
} AttitudeGroup;

/** \brief Flags for the binary group 6 'INS' in the binary output registers. */
typedef enum
{
	INSGROUP_NONE					= 0x0000,	/**< None. */
	INSGROUP_INSSTATUS				= 0x0001,	/**< InsStatus. */
	INSGROUP_POSLLA					= 0x0002,	/**< PosLla. */
	INSGROUP_POSECEF				= 0x0004,	/**< PosEcef. */
	INSGROUP_VELBODY				= 0x0008,	/**< VelBody. */
	INSGROUP_VELNED					= 0x0010,	/**< VelNed. */
	INSGROUP_VELECEF				= 0x0020,	/**< VelEcef. */
	INSGROUP_MAGECEF				= 0x0040,	/**< MagEcef. */
	INSGROUP_ACCELECEF				= 0x0080,	/**< AccelEcef. */
	INSGROUP_LINEARACCELECEF		= 0x0100,	/**< LinearAccelEcef. */
	INSGROUP_POSU					= 0x0200,	/**< PosU. */
	INSGROUP_VELU					= 0x0400	/**< VelU. */
	#ifdef EXTRA
	,
	INSGROUP_STATEINS				= 0x0800,	/**< StateIns. */
	INSGROUP_COVINS					= 0x1000	/**< CovIns. */
	#endif
} InsGroup;

/** \brief Enumeration of the velocity types */
typedef enum
{
	CDVEL_None,
	CDVEL_GpsNed,
	CDVEL_GpsEcef,
	CDVEL_EstimatedNed,
	CDVEL_EstimatedEcef,
	CDVEL_EstimatedBody
} VelocityType;

/** \brief Errors that the VectorNav sensor can report. */
typedef enum
{
	ERR_HARD_FAULT				= 1,	/**< Hard fault. */
	ERR_SERIAL_BUFFER_OVERFLOW	= 2,	/**< Serial buffer overflow. */
	ERR_INVALID_CHECKSUM		= 3,	/**< Invalid checksum. */
	ERR_INVALID_COMMAND			= 4,	/**< Invalid command. */
	ERR_NOT_ENOUGH_PARAMETERS	= 5,	/**< Not enough parameters. */
	ERR_TOO_MANY_PARAMETERS		= 6,	/**< Too many parameters. */
	ERR_INVALID_PARAMETER		= 7,	/**< Invalid parameter. */
	ERR_INVALID_REGISTER		= 8,	/**< Invalid register. */
	ERR_UNAUTHORIZED_ACCESS		= 9,	/**< Unauthorized access. */
	ERR_WATCHDOG_RESET			= 10,	/**< Watchdog reset. */
	ERR_OUTPUT_BUFFER_OVERFLOW	= 11,	/**< Output buffer overflow. */
	ERR_INSUFFICIENT_BAUD_RATE	= 12,	/**< Insufficient baud rate. */
	ERR_ERROR_BUFFER_OVERFLOW	= 255	/**< Error buffer overflow. */
} SensorError;

/** \brief Enumeration of the various error messages used by the library. */
typedef enum
{
	/** Indicates there were no errors encountered. */
	E_NONE,

	/** Indicates an unknown error occurred. */
	E_UNKNOWN,

	/** Indicates a provided buffer was too small to complete the action. */
	E_BUFFER_TOO_SMALL,

	/** Indicates a provided value is not valid. */
	E_INVALID_VALUE,

	/** Indicates the requested functionality is currently not implemented. */
	E_NOT_IMPLEMENTED,

	/** Indicates the requested functionality is not supported. */
	E_NOT_SUPPORTED,

	/** Indicates the requested item was not found. */
	E_NOT_FOUND,

	/** Indicates the operation timed out. */
	E_TIMEOUT,

	/** Indicates insufficient permission to perform the operation. */
	E_PERMISSION_DENIED,

	/** Indicates an invalid operation was attempted. */
	E_INVALID_OPERATION,

	/** Indicates an event was signaled. */
	E_SIGNALED,

    /** Indicates either not enough memory is available or no memory was allocated */
    E_MEMORY_NOT_ALLOCATED,

	/** VectorNav sensor hard fault (Code 1). */
	E_SENSOR_HARD_FAULT = 1001,

	/** VectorNav sensor serial buffer overflow (Code 2). */
	E_SENSOR_SERIAL_BUFFER_OVERFLOW = 1002,

	/** VectorNav sensor invalid checksum (Code 3). */
	E_SENSOR_INVALID_CHECKSUM = 1003,

	/** VectorNav sensor invalid command (Code 4). */
	E_SENSOR_INVALID_COMMAND = 1004,

	/** VectorNav sensor not enough parameters (Code 5). */
	E_SENSOR_NOT_ENOUGH_PARAMETERS = 1005,

	/** VectorNav sensor too many parameters (Code 6). */
	E_SENSOR_TOO_MANY_PARAMETERS = 1006,

	/** VectorNav sensor invalid parameter (Code 7). */
	E_SENSOR_INVALID_PARAMETER = 1007,

	/** VectorNav sensor invalid register (Code 8). */
	E_SENSOR_INVALID_REGISTER = 1008,

	/** VectorNav sensor unauthorized access (Code 9). */
	E_SENSOR_UNAUTHORIZED_ACCESS = 1009,

	/** VectorNav sensor watchdog reset (Code 10). */
	E_SENSOR_WATCHDOG_RESET = 1010,

	/** VectorNav sensor output buffer overflow (Code 11). */
	E_SENSOR_OUTPUT_BUFFER_OVERFLOW = 1011,

	/** VectorNav sensor insufficient baud rate (Code 12). */
	E_SENSOR_INSUFFICIENT_BAUD_RATE = 1012,

	/** VectorNav sensor error buffer overflow (Code 13). */
	E_SENSOR_ERROR_BUFFER_OVERFLOW = 1013,

	E_DATA_NOT_ELLIPTICAL		= 2001,	/**< \brief Data set not elliptical. */
	E_ILL_CONDITIONED			= 2002,	/**< \brief Algorithm had a bad condition. */
	E_EXCEEDED_MAX_ITERATIONS	= 2003,	/**< \brief Algorithm exceeded the number of interations it is allowed. */
	E_BAD_FINAL_INTERATION		= 2004,	/**< \brief Algorithm's last interation changed exceeded threshold. */
	E_INSUFFICIENT_DATA			= 2005,	/**< \brief Not enough data points were provided. */

	/** \brief Errors that the VectorNav bootload can report. */
	E_BOOTLOADER_NONE					= 3000,	/**< No Error, send next record*/
	E_BOOTLOADER_INVALID_COMMAND		= 3001,	/**< Problem with VNX record, abort */
	E_BOOTLOADER_INVALID_RECORD_TYPE	= 3002,	/**< Problem with VNX record, abort */
	E_BOOTLOADER_INVALID_BYTE_COUNT		= 3003,	/**< Problem with VNX record, abort */
	E_BOOTLOADER_INVALID_MEMORY_ADDRESS	= 3004,	/**< Problem with VNX record, abort */
	E_BOOTLOADER_COMM_ERROR				= 3005,	/**< Checksum error, resend record */
	E_BOOTLOADER_INVALID_HEX_FILE		= 3006,	/**< Problem with VNX record, abort */
	E_BOOTLOADER_DECRYPTION_ERROR		= 3007,	/**< Invalid VNX file or record sent out of order, abort */
	E_BOOTLOADER_INVALID_BLOCK_CRC		= 3008,	/**< Data verification failed, abort */
	E_BOOTLOADER_INVALID_PROGRAM_CRC	= 3009,	/**< Problemw ith firmware on device */
	E_BOOTLOADER_INVALID_PROGRAM_SIZE	= 3010,	/**< Problemw ith firmware on device */
	E_BOOTLOADER_MAX_RETRY_COUNT		= 3011,	/**< Too many errors, abort */
	E_BOOTLOADER_TIMEOUT				= 3012,	/**< Timeout expired, reset */
	E_BOOTLOADER_RESERVED				= 3013	/**< Contact VectorNav, abort */

} VnError;

/** \brief Enumeration of the various error-detection algorithms used by the
 * library. */
typedef enum
{
	/** Signifies no error-detection should be performed. */
	VNERRORDETECTIONMODE_NONE,

	/** Signifies to use 8-bit XOR checksum. */
	VNERRORDETECTIONMODE_CHECKSUM,

	/** Signifies to use CRC16-CCITT algorithm. */
	VNERRORDETECTIONMODE_CRC

} VnErrorDetectionMode;

/** \brief Different modes for the SyncInMode field of the Synchronization Control register. */
typedef enum
{
	#ifdef EXTRA
	/** \brief Count the number of trigger events on SYNC_IN_2 pin.
	/ * \deprecated This option is obsolete for VN-100 firmware version 2.0 and greater and VN-200 firmware version 1.0 and greater.
	 */
	VNSYNCINMODE_COUNT2 = 0,
	/** \brief Start ADC sampling on trigger of SYNC_IN_2 pin.
	/ * \deprecated This option is obsolete for VN-100 firmware version 2.0 and greater and VN-200 firmware version 1.0 and greater.
	 */
	VNSYNCINMODE_ADC2 = 1,
	/** \brief Output asynchronous message on trigger of SYNC_IN_2 pin.
	/ * \deprecated This option is obsolete for VN-100 firmware version 2.0 and greater and VN-200 firmware version 1.0 and greater.
	 */
	VNSYNCINMODE_ASYNC2 = 2,
	#endif
	/** \brief Count number of trigger events on SYNC_IN pin.
	 */
	VNSYNCINMODE_COUNT = 3,
	/** \brief Start IMU sampling on trigger of SYNC_IN pin.
	 */
	VNSYNCINMODE_IMU = 4,
	/** \brief Output asynchronous message on trigger of SYNC_IN pin.
	 */
	VNSYNCINMODE_ASYNC = 5
} VnSyncInMode;

/** \brief Different modes for the SyncInEdge field of the Synchronization Control register. */
typedef enum
{
	/** \brief Trigger on the rising edge on the SYNC_IN pin.
	 */
	VNSYNCINEDGE_RISING = 0,
	/** \brief Trigger on the falling edge on the SYNC_IN pin.
	 */
	VNSYNCINEDGE_FALLING = 1
} VnSyncInEdge;

/** \brief Different modes for the SyncOutMode field of the Synchronization Control register. */
typedef enum
{
	/** \brief None.
	 */
	VNSYNCOUTMODE_NONE = 0,
	/** \brief Trigger at start of IMU sampling.
	 */
	VNSYNCOUTMODE_ITEMSTART = 1,
	/** \brief Trigger when IMU measurements are available.
	 */
	VNSYNCOUTMODE_IMUREADY = 2,
	/** \brief Trigger when attitude measurements are available.
	 */
	VNSYNCOUTMODE_INS = 3,
	/** \brief Trigger on GPS PPS event when a 3D fix is valid.
	 */
	VNSYNCOUTMODE_GPSPPS = 6
} VnSyncOutMode;

/** \brief Different modes for the SyncOutPolarity field of the Synchronization Control register. */
typedef enum
{
	/** \brief Negative pulse.
	 */
	VNSYNCOUTPOLARITY_NEGATIVE = 0,
	/** \brief Positive pulse.
	 */
	VNSYNCOUTPOLARITY_POSITIVE = 1
} VnSyncOutPolarity;

/** \brief Counting modes for the Communication Protocol Control register. */
typedef enum
{
	/** \brief Off.
	 */
	VNCOUNTMODE_NONE = 0,
	/** \brief SyncIn counter.
	 */
	VNCOUNTMODE_SYNCINCOUNT = 1,
	/** \brief SyncIn time.
	 */
	VNCOUNTMODE_SYNCINTIME = 2,
	/** \brief SyncOut counter.
	 */
	VNCOUNTMODE_SYNCOUTCOUNTER = 3,
	/** \brief GPS PPS time.
	 */
	VNCOUNTMODE_GPSPPS = 4
} VnCountMode;

/** \brief Status modes for the Communication Protocol Control register. */
typedef enum
{
	/** \brief Off.
	 */
	VNSTATUSMODE_OFF = 0,
	/** \brief VPE status.
	 */
	VNSTATUSMODE_VPESTATUS = 1,
	/** \brief INS status.
	 */
	VNSTATUSMODE_INSSTATUS = 2
} VnStatusMode;

/** \brief Checksum modes for the Communication Protocol Control register. */
typedef enum
{
	/** \brief Off.
	 */
	VNCHECKSUMMODE_OFF = 0,
	/** \brief 8-bit checksum.
	 */
	VNCHECKSUMMODE_CHECKSUM = 1,
	/** \brief 16-bit CRC.
	 */
	VNCHECKSUMMODE_CRC = 2
} VnChecksumMode;

/** \brief Error modes for the Communication Protocol Control register. */
typedef enum
{
	/** \brief Ignore error.
	 */
	VNERRORMODE_IGNORE = 0,
	/** \brief Send error.
	 */
	VNERRORMODE_SEND = 1,
	/** \brief Send error and set ADOR register to off.
	 */
	VNERRORMODE_SENDANDOFF = 2
} VnErrorMode;

/** \brief Filter modes for the IMU Filtering Configuration register. */
typedef enum
{
	/** \brief No filtering.
	 */
	VNFILTERMODE_NOFILTERING = 0,
	/** \brief Filtering performed only on raw uncompensated IMU measurements.
	 */
	VNFILTERMODE_ONLYRAW = 1,
	/** \brief Filtering performed only on compensated IMU measurements.
	 */
	VNFILTERMODE_ONLYCOMPENSATED = 2,
	/** \brief Filtering performed on both uncompensated and compensated IMU measurements.
	 */
	VNFILTERMODE_BOTH = 3
} VnFilterMode;

/** \brief Integration frames for the Delta Theta and Delta Velocity Configuration register. */
typedef enum
{
	/** \brief Body frame.
	 */
	VNINTEGRATIONFRAME_BODY = 0,
	/** \brief NED frame.
	 */
	VNINTEGRATIONFRAME_NED = 1
} VnIntegrationFrame;

/** \brief Compensation modes for the Delta Theta and Delta Velocity configuration register. */
typedef enum
{
	/** \brief None.
	 */
	VNCOMPENSATIONMODE_NONE = 0,
	/** \brief Bias.
	 */
	VNCOMPENSATIONMODE_BIAS = 1
} VnCompensationMode;

/** \brief GPS fix modes for the GPS Solution - LLA register. */
typedef enum
{
	/** \brief No fix.
	 */
	VNGPSFIX_NOFIX = 0,
	/** \brief Time only.
	 */
	VNGPSFIX_TIMEONLY = 1,
	/** \brief 2D.
	 */
	VNGPSFIX_2D = 2,
	/** \brief 3D.
	 */
	VNGPSFIX_3D = 3
} VnGpsFix;

/** \brief GPS modes for the GPS Configuration register. */
typedef enum
{
	/** \brief Use onboard GPS.
	 */
	VNGPSMODE_ONBOARDGPS = 0,
	/** \brief Use external GPS.
	 */
	VNGPSMODE_EXTERNALGPS = 1,
	/** \brief Use external VN-200 as GPS.
	 */
	VNGPSMODE_EXTERNALVN200GPS = 2
} VnGpsMode;

/** \brief GPS PPS mode for the GPS Configuration register. */
typedef enum
{
	/** \brief GPS PPS signal on GPS_PPS pin and triggered on rising edge.
	 */
	VNPPSSOURCE_GPSPPSRISING = 0,
	/** \brief GPS PPS signal on GPS_PPS pin and triggered on falling edge.
	 */
	VNPPSSOURCE_GPSPPSFALLING = 1,
	/** \brief GPS PPS signal on SyncIn pin and triggered on rising edge.
	 */
	VNPPSSOURCE_SYNCINRISING = 2,
	/** \brief GPS PPS signal on SyncIn pin and triggered on falling edge.
	 */
	VNPPSSOURCE_SYNCINFALLING = 3
} VnPpsSource;

/** \brief VPE Enable mode for the VPE Basic Control register. */
typedef enum
{
	/** \brief Disable
	 */
	VNVPEENABLE_DISABLE = 0,
	/** \brief Enable
	 */
	VNVPEENABLE_ENABLE = 1
} VnVpeEnable;

/** \brief VPE Heading modes used by the VPE Basic Control register. */
typedef enum
{
	/** \brief Absolute heading.
	 */
	VNHEADINGMODE_ABSOLUTE = 0,
	/** \brief Relative heading.
	 */
	VNHEADINGMODE_RELATIVE = 1,
	/** \brief Indoor heading.
	 */
	VNHEADINGMODE_INDOOR = 2
} VnHeadingMode;

/** \brief VPE modes for the VPE Basic Control register. */
typedef enum
{
	/** \brief Off.
	 */
	VNVPEMODE_OFF = 0,
	/** \brief Mode 1.
	 */
	VNVPEMODE_MODE1 = 1
} VnVpeMode;

/** \brief Different scenario modes for the INS Basic Configuration register. */
typedef enum
{
	/** \brief AHRS.
	 */
	VNSCENARIO_AHRS = 0,
	/** \brief General purpose INS with barometric pressure sensor.
	 */
	VNSCENARIO_INSWITHPRESSURE = 1,
	/** \brief General purpose INS without barometric pressure sensor.
	 */
	VNSCENARIO_INSWITHOUTPRESSURE = 2,
	/** \brief GPS moving baseline for dynamic applications.
	 */
	VNSCENARIO_GPSMOVINGBASELINEDYNAMIC = 3,
	/** \brief GPS moving baseline for static applications.
	 */
	VNSCENARIO_GPSMOVINGBASELINESTATIC = 4
} VnScenario;

/** \brief HSI modes used for the Magnetometer Calibration Control register. */
typedef enum
{
	/** \brief Real-time hard/soft iron calibration algorithm is turned off.
	 */
	VNHSIMODE_OFF = 0,
	/** \brief Runs the real-time hard/soft iron calibration algorithm.
	 */
	VNHSIMODE_RUN = 1,
	/** \brief Resets the real-time hard/soft iron solution.
	 */
	VNHSIMODE_RESET = 2
} VnHsiMode;

/** \brief HSI output types for the Magnetometer Calibration Control register. */
typedef enum
{
	/** \brief Onboard HSI is not applied to the magnetic measurements.
	 */
	VNHSIOUTPUT_NOONBOARD = 1,
	/** \brief Onboard HSI is applied to the magnetic measurements.
	 */
	VNHSIOUTPUT_USEONBOARD = 3
} VnHsiOutput;

/** \brief Type of velocity compensation performed by the VPE. */
typedef enum
{
	/** \brief Disabled
	 */
	VNVELOCITYCOMPENSATIONMODE_DISABLED = 0,
	/** \brief Body Measurement
	 */
	VNVELOCITYCOMPENSATIONMODE_BODYMEASUREMENT = 1
} VnVelocityCompensationMode;

/** \brief How the magnetometer is used by the filter. */
typedef enum
{
	/** \brief Magnetometer will only affect heading.
	 */
	VNMAGNETICMODE_2D = 0,
	/** \brief Magnetometer will affect heading, pitch, and roll.
	 */
	VNMAGNETICMODE_3D = 1
} VnMagneticMode;

/** \brief Source for magnetometer used by the filter. */
typedef enum
{
	/** \brief Use internal magnetometer.
	 */
	VNEXTERNALSENSORMODE_INTERNAL = 0,
	/** \brief Use external magnetometer. Will use measurement at every new time step.
	 */
	VNEXTERNALSENSORMODE_EXTERNAL200HZ = 1,
	/** \brief Use external magnetometer. Will only use when the measurement is updated.
	 */
	VNEXTERNALSENSORMODE_EXTERNALONUPDATE = 2
} VnExternalSensorMode;

/** \brief Options for the use of FOAM. */
typedef enum
{
	/** \brief FOAM is not used.
	 */
	VNFOAMINIT_NOFOAMINIT = 0,
	/** \brief FOAM is used to initialize only pitch and roll.
	 */
	VNFOAMINIT_FOAMINITPITCHROLL = 1,
	/** \brief FOAM is used to initialize heading, pitch and roll.
	 */
	VNFOAMINIT_FOAMINITHEADINGPITCHROLL = 2,
	/** \brief FOAM is used to initialize pitch, roll and covariance.
	 */
	VNFOAMINIT_FOAMINITPITCHROLLCOVARIANCE = 3,
	/** \brief FOAM is used to initialize heading, pitch, roll and covariance
	 */
	VNFOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE = 4
} VnFoamInit;

/** \brief The different types of processors. */
typedef enum
{
	VNPROCESSOR_NAV,	/**< Navigation Processor. */
	VNPROCESSOR_GPS,	/**< GPS Processor. */
	VNPROCESSOR_IMU   /**< IMU Processor. */
} VnProcessorType;

#endif
