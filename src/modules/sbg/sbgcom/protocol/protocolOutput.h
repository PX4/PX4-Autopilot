/*!
 *	\file		protocolOutput.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		20/07/07
 *
 *	\brief		Definitions and helpers for  GET_DEFAULT_OUTPUT and GET_SPECIFIC_OUTPUT commands.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef __PROTOCOL_OUTPUT_H__
#define __PROTOCOL_OUTPUT_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Output mask definitions                                            -//
//----------------------------------------------------------------------//
#define SBG_OUTPUT_QUATERNION					(0x00000001)			/*!< Enable quaternion attitude output */
#define SBG_OUTPUT_EULER						(0x00000002)			/*!< Enable euler angles attitude output */
#define SBG_OUTPUT_MATRIX						(0x00000004)			/*!< Enable matrix attitude output */

#define SBG_OUTPUT_GYROSCOPES					(0x00000008)			/*!< Enable calibrated gyroscopes output */
#define SBG_OUTPUT_ACCELEROMETERS				(0x00000010)			/*!< Enable calibrated acceleromters output */
#define SBG_OUTPUT_MAGNETOMETERS				(0x00000020)			/*!< Enable calibrated magnetometers output */
#define SBG_OUTPUT_TEMPERATURES					(0x00000040)			/*!< Enable calibrated temperatures sensors output */

#define SBG_OUTPUT_GYROSCOPES_RAW				(0x00000080)			/*!< Enable raw gyroscopes sensors output */
#define SBG_OUTPUT_ACCELEROMETERS_RAW			(0x00000100)			/*!< Enable raw accelerometers sensors output */
#define SBG_OUTPUT_MAGNETOMETERS_RAW			(0x00000200)			/*!< Enable raw magnetometers sensors output */
#define SBG_OUTPUT_TEMPERATURES_RAW				(0x00000400)			/*!< Enable raw temperatures sensors output */

#define SBG_OUTPUT_TIME_SINCE_RESET				(0x00000800)			/*!< Enable the time since the device is on in milliseconds */

#define SBG_OUTPUT_DEVICE_STATUS				(0x00001000)			/*!< Enable the Device status output */

#define SBG_OUTPUT_GPS_POSITION					(0x00002000)			/*!< Enable the raw GPS position data */
#define SBG_OUTPUT_GPS_NAVIGATION				(0x00004000)			/*!< Enable the raw GPS navigation data such as velocity, heading */
#define SBG_OUTPUT_GPS_ACCURACY					(0x00008000)			/*!< Enable the raw GPS accuracy for both position and navigation data */
#define SBG_OUTPUT_GPS_INFO						(0x00010000)			/*!< Enable the raw GPS information data such as number of used space vehicles */

#define SBG_OUTPUT_BARO_ALTITUDE				(0x00020000)			/*!< Enable the barometric altitude referenced on a user defined reference pressure */
#define SBG_OUTPUT_BARO_PRESSURE				(0x00040000)			/*!< Enable the raw pressure output in pascal */

#define SBG_OUTPUT_POSITION						(0x00080000)			/*!< Enable the Kalman Enhanced 3d position in NED */
#define SBG_OUTPUT_VELOCITY						(0x00100000)			/*!< Enable the Kalman Enhanced 3d veclotiy expressed in device space (vX,vY,vZ) */

#define SBG_OUTPUT_ATTITUDE_ACCURACY			(0x00200000)			/*!< Enable the Kalman estimated attitude accuracy */
#define SBG_OUTPUT_NAV_ACCURACY					(0x00400000)			/*!< Enable the Kalman estimated position and velocity accuracy */

#define SBG_OUTPUT_GYRO_TEMPERATURES			(0x00800000)			/*!< Enable calibrated gyroscopes temperatures sensors output */
#define SBG_OUTPUT_GYRO_TEMPERATURES_RAW		(0x01000000)			/*!< Enable raw gyroscopes temperatures sensors output */

#define SBG_OUTPUT_UTC_TIME_REFERENCE			(0x02000000)			/*!< Enable UTC time reference */

#define SBG_OUTPUT_MAG_CALIB_DATA				(0x04000000)			/*!< Enable Magnetometers Calibration data */

#define SBG_OUTPUT_GPS_TRUE_HEADING				(0x08000000)			/*!< Enable raw true heading output */

#define SBG_OUTPUT_ODO_VELOCITIES				(0x10000000)			/*!< Enable Odometer raw velocities output */

#define SBG_OUTPUT_DELTA_ANGLES					(0x20000000)			/*!< Enable delta angle output from coning integration */

#define SBG_OUTPUT_HEAVE						(0x40000000)			/*!< Enable Heave output */

//----------------------------------------------------------------------//
//- GPS status definitions and macros                                  -//
//----------------------------------------------------------------------//
#define SBG_GPS_NO_FIX							(0x00)					/*!< The GPS has no position solution. */
#define SBG_GPS_TIME_ONLY 						(0x01)					/*!< The GPS has only a time information. */
#define SBG_GPS_2D_FIX							(0x02)					/*!< The GPS has only a valid 2D position. */
#define SBG_GPS_3D_FIX							(0x03)					/*!< The GPS has a complete 3D position. */

#define	GPS_STD_FIX								(0x00)					/*!< Fix Quality extended : The fix is standard GPS Fix */
#define	GPS_DGPS_FIX							(0x40)					/*!< Fix Quality extended : The fix is a DGPS Fix */
#define	GPS_FRTK_FIX							(0x80)					/*!< Fix Quality extended : The fix is float RTK GPS Fix */
#define	GPS_RTK_FIX								(0xC0)					/*!< Fix Quality extended : The fix is float RTK GPS Fix */

#define SBG_GPS_VALID_TOW						(0x04)					/*!< Mask 1 = Valid Time of Week */
#define SBG_GPS_VALID_WKN						(0x08)					/*!< Mask 1 = Valid Week Number */
#define SBG_GPS_VALID_UTC						(0x10)					/*!< Mask 1 = Valid UTC (Leap Seconds already known) */

#define	SBG_GPS_TRUE_HEADING_VALID				(0x20)					/*!< 1 = True heading output is valid */

#define SBG_GPS_GET_FIX(flags)					(flags & 0x03)			/*!< Returns the fix status in the passed GPSFlags variable */
#define SBG_GPS_GET_FIX_EXT(flags)				(flags & 0xC0)			/*!< Returns the extended fix status in the passed GPSFlags variable */
#define SBG_GPS_GET_VALID_TOW(flags)			((flags >>2) & 0x01)	/*!< Returns 1 if the GPSFlags variable indicates a valid GPS Time Of Week */
#define SBG_GPS_GET_VALID_WKN(flags)			((flags >>3) & 0x01)	/*!< Returns 1 if the GPSFlags variable indicates a valid GPS Week number */
#define SBG_GPS_GET_VALID_UTC(flags)			((flags >>4) & 0x01)	/*!< Returns 1 if the GPSFlags variable indicates a valid UTC Time (Leap Seconds already known) */

//----------------------------------------------------------------------//
//- Device status Bitmask  definitions                                 -//
//----------------------------------------------------------------------//

//
// In the following bitmasks, a 1 signify normal operation, and a 0 means there is a warning
//
#define SBG_CALIB_INIT_STATUS_MASK				(0x00000001)					/*!< Bit mask for calibration loading result */
#define SBG_SETTINGS_INIT_STATUS_MASK			(0x00000002)					/*!< Bit mask for settings init result indication */

#define SBG_ACCEL_0_SELF_TEST_STATUS_MASK		(0x00000004)					/*!< Bit mask for accelerometer 0 self test result */
#define SBG_ACCEL_1_SELF_TEST_STATUS_MASK		(0x00000008)					/*!< Bit mask for accelerometer 1 self test result */
#define SBG_ACCEL_2_SELF_TEST_STATUS_MASK		(0x00000010)					/*!< Bit mask for accelerometer 2 self test result */
#define SBG_ACCEL_RANGE_STATUS_MASK				(0x00000020)					/*!< Bit mask for Accelerometer range status <br>
																					 - 1: in range operation; 
																					 - 0: over range */

#define SBG_GYRO_0_SELF_TEST_STATUS_MASK		(0x00000040)					/*!< Bit mask for gyroscope 0 self test result */
#define SBG_GYRO_1_SELF_TEST_STATUS_MASK		(0x00000080)					/*!< Bit mask for gyroscope 1 self test result */
#define SBG_GYRO_2_SELF_TEST_STATUS_MASK		(0x00000100)					/*!< Bit mask for gyroscope 2 self test result */
#define SBG_GYRO_RANGE_STATUS_MASK				(0x00000200)					/*!< Bit mask for Gyroscope range status <br>
																					 - 1: in range operation; 
																					 - 0: over range */

#define SBG_MAG_CALIBRATION_STATUS_MASK			(0x00000400)					/*!< Bit mask for magnetometers range status	<br>
																					 - 1: calibration seems to be OK;	<br>
																					 - 0: strong magnetic fields are applied, or magnetic field
																					    calibration has to be performed */

#define SBG_ALTI_INIT_STATUS_BIT_MASK			(0x00000800)					/*!< Bit mask for Altimeter initialization result */
#define SBG_GPS_STATUS_BIT_MASK					(0x00001000)					/*!< Bit mask for GPS initialization and communication status: <br>
																					- 1: Active communication with external device (GPS)
																					- 0: Bad GPS initialization / Communication lost */
																					
#define SBG_G_MEASUREMENT_VALID_MASK			(0x00002000)					/*!< Bit mask for proper gravity measurement status <br>
																					 - 1: valid gravity is measured over long time periods; 
																					 - 0: No gravity is observable over long time periods */
#define SBG_HEADING_MEASUREMENT_VALID_MASK		(0x00004000)					/*!< Bit mask for proper heading measurement status <br>
																					 - 1: valid heading is measured over long time periods; 
																					 - 0: No heading is observable over long time periods */
#define SBG_VEL_MEASUREMENT_VALID_MASK			(0x00008000)					/*!< Bit mask for proper velocity measurement status <br>
																					 - 1: valid heading is measured;
																					 - 0: No heading is observable */
#define SBG_POS_MEASUREMENT_VALID_MASK			(0x00010000)					/*!< Bit mask for proper position measurement status <br>
																					 - 1: valid heading is measured; 
																					 - 0: No heading is observable */
#define SBG_UTC_VALID_MASK						(0x00020000)					/*!< Bit mask for GPS UTC valid, leap seconds already known */
#define SBG_UTC_ROUGH_ACCURACY_MASK				(0x00040000)					/*!< Bit mask for UTC time with rough accuracy (about 250 ms)*/
#define SBG_UTC_FINE_ACURACY_MASK				(0x00080000)					/*!< Bit mask for UTC time synchronized with the GPS  */

#define SBG_PROTOCOL_OUTPUT_STATUS_MASK			(0x00100000)					/*!< Bit mask for output buffer saturation indicator:<br>
																		  			 - 1: Ouptput buffer in normal operation
																		 			 - 0:  Output buffer is saturated */

//----------------------------------------------------------------------//
//- Output structure definition                                        -//
//----------------------------------------------------------------------//

/*!
 *	Struct containing data outputted by the device.
 */
typedef struct _SbgOutput
{
	uint32	outputMask;				/*!< Mask that validate the output field */
	float	stateQuat[4];			/*!< The orientation quaternion w,x,y,z */
	float	stateEuler[3];			/*!< The orientation in euler angles roll, pitch, yaw in radians */
	float	stateMatrix[9];			/*!< The orientation 3x3 matrix stored like this:<br> */
									/*!< aX, aY, aZ, bX, bY, bZ, cX, cY, cZ */
	
	float	gyroscopes[3];			/*!< Calibrated version of the 3 gyroscopes, gX, gY, gZ */
	float	accelerometers[3];		/*!< Calibrated version of the 3 accelerometers, aX, aY, aZ */
	float	magnetometers[3];		/*!< Calibrated version of the 3 magnetometers, mX, mY, mZ */
	float	temperatures[2];		/*!< Calibrated temperature values, t0, t1 */
	
	uint16	gyroscopesRaw[3];		/*!< Raw values of the 3 gyroscopes */
	uint16	accelerometersRaw[3];	/*!< Raw values of the 3 accelerometers */
	uint16	magnetometersRaw[3];	/*!< Raw values of the 3 magnetometers */
	uint16	temperaturesRaw[2];		/*!< Raw temperature value for sensors 0 and 1 */

	uint32	timeSinceReset;			/*!< Elapsed time in ms since the imu is running */

	uint32	deviceStatus;			/*!< Status information about device startup */

	int32	gpsLatitude;			/*!< Latitude in degress (1e-7)	*/
	int32	gpsLongitude;			/*!< Longitude in degrees (1e-7) */
	int32	gpsAltitude;			/*!< Height above sea level in mm */

	int32	gpsVelocity[3];			/*!< North, East, Down velocity in cm/s (NED coordinate system) */
	int32	gpsHeading;				/*!< 2D heading in degrees (1e-5) */
	
	uint32	gpsHorAccuracy;			/*!< Horizontal accuracy in mm */
	uint32	gpsVertAccuracy;		/*!< Vertical accuracy in mm */
	uint32	gpsSpeedAccuracy;		/*!< Speed accuracy in cm/s */
	uint32	gpsHeadingAccuracy;		/*!< Heading accuracy in degrees (1e-5) */

	uint32	gpsTimeMs;				/*!< GPS Millisecond Time of Week */
	uint8	gpsFlags;				/*!< GPS basic information : 2D/3D fix or time only, etc				<br>
									<b> bit [0-1]: GPS fix Status:		</b>								<br>
									0x00 SBG_NO_FIX	-> The GPS receiver does not have a fix at all			<br>
									0x01 SBG_TIME_ONLY -> The GPS receiver has a time only fix				<br>
									0x02 SBG_2D_FIX	-> The GPS receiver has a 2D fix only					<br>
									0x03 SBG_3D_FIX	-> Full 3D fix reception on the GPS receiver			<br>
									<b>bit 2:</b> validTOW: 1= Valid Time of Week							<br>
									<b>bit 3:</b> validWKN: 1= Valid week number							<br>
									<b>bit 4:</b> validUTC: 1= Valid UTC (Leap Seconds already known)		<br>*/
	uint8	gpsNbSats;				/*!< GPS number of satellites used in position estimation */

	int32	gpsTrueHeading;			/*!< GPS True heading value. in  (1e-5 ° unit) */
	uint32	gpsTrueHeadingAccuracy;	/*!< GPS True heading accuracy value. in  (1e-5 ° unit) */

	int32	baroAltitude;			/*!< Altitude above reference zero based on pressure measurement in cms */
	uint32	baroPressure;			/*!< Absolute pressure measurement in pascals*/

	double	position[3];			/*!< 3d position North East Height above ellipsoid in (deg, deg, meters) */
	float	velocity[3];			/*!< 3d velocity in device coordinate and in m/s */

	float	attitudeAccuracy;		/*!< Attitude accuracy in rad estimated by the Kalman Filter */
	float	positionAccuracy;		/*!< Position accuracy in meters estimated by the Kalman Filter */
	float	velocityAccuracy;		/*!< Velocity accuracy in m/s estimated by the Kalman Filter */

	float	gyroTemperatures[3];	/*!< Calibrated internal gyro temperatures sensors output */
	uint16	gyroTemperaturesRaw[3];	/*!< Raw internal gyro temperatures sensors output */

	uint8	utcYear;				/*!< UTC time reference: Year = 2000+ year */
	uint8	utcMonth;				/*!< Month, range 1..12 (UTC) */
	uint8	utcDay; 				/*!< Day of Month, range 1..31 (UTC) */
	uint8	utcHour;				/*!< Hour of Day, range 0..23 (UTC) */
	uint8	utcMin;					/*!< Minute of Hour, range 0..59 (UTC) */
	uint8	utcSec;					/*!< Seconds of Minute, range 0..59 (UTC) */
	uint32	utcNano;				/*!< Nanoseconds of second, range 0 .. 1 000 000 000(UTC) */

	uint8	magCalibData[12];		/*!< Magnetometers Soft and Hard Iron calibration data */

	float	odoRawVelocity[2];		/*!< Odometer two channels velocities in m/s */

	float	deltaAngles[3];			/*!< Delta angle output in rad/s */
	float	heave;					/*!< Heave output in meters */
} SbgOutput;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Fill the SbgOutput struct from a target based formatte raw buffer.<br>
 *	\param[in]	targetOutputMode		The output mode used by the target.<br>
 *										Defines if data stored in pBuffer are stored in Big/Little endian<br>
 *										and if real values are stored in float or in fixed format.
 *	\param[in]	outputMask				Mask combinaison that defines which outputs are contained in the pBuffer.
 *	\param[in]	pBuffer					Raw buffer that contains the data to extract into SbgOutput.
 *	\param[in]	bufferSize				The size of the pBuffer.
 *	\param[out]	pOutput					Pointer to a SbgOutut struct used to hold extracted data.
 *	\return								SBG_NO_ERROR if we have sucessfully extracted data from pBuffer and filled the pOutput struct.
 */
SbgErrorCode sbgFillOutputFromBuffer(uint8 targetOutputMode, uint32 outputMask, void *pBuffer, uint16 bufferSize, SbgOutput *pOutput);

/*!
 *	Calculate and returns the size of a buffer containing the data enabled in oututMask.
 *	\param[in]	targetOutputMode		The output mode used by the target.<br>
 *										Defines if the data are stored in Big/Little endian <br>
 *										and if real values are stored in float or in fixed format.
 *	\param[in]	outputMask				Mask combinaison that defines which outputs are contained in the pBuffer.
 *	\return								The size of the buffer needed to hold enabled outputs.
 */
uint16 sbgCalculateOutputBufferSize(uint8 targetOutputMode, uint32 outputMask);

#endif	// __PROTOCOL_OUTPUT_H__

