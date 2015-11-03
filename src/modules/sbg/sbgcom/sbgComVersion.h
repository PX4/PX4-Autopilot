/*!
 *	\file		sbgComVersion.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		03/11/08
 *
 *	\brief		Version header file for the sbgCom library.<br>
 *				You can find in this file all change logs.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef __SBG_COM_VERSION_H__
#define __SBG_COM_VERSION_H__

#include "sbgCommon.h"

//----------------------------------------------------------------------//
//- Version definitions                                                -//
//----------------------------------------------------------------------//



/*!
 *	Change log:<br>
 *	============<br>
 *	<br>
 *	<table border="0">
 *
 *	<tr><td valign="top">16/05/12:</td><td>
 *	Version 2.0.0.0:                
 *	<h2>        Improvement
 *	</h2>
 *	<ul>
 *	<li>[SBGCOM-1] -         Obsolete &quot;permanent&quot; parametrer in SET commands
 *	</li>
 *	<li>[SBGCOM-6] -         Aiding commands (sbgSendxxx) do not wait for ACK anymore
 *	</li>
 *	<li>[SBGCOM-14] -         The num sbgCalibAction has been split into two enums, one for magnetometers and one for gyroscopes 
 *	</li>
 *	<li>[SBGCOM-38] -         Changed sbgComGetVersionAsString method behavior
 *	</li>
 *	</ul>
 *	   
 *	<h2>        New Feature
 *	</h2>
 *	<ul>
 *	<li>[SBGCOM-2] -         sbgSetFilterHeadingSource: GPS + acceleration heading source added
 *	</li>
 *	<li>[SBGCOM-4] -         sbgSetOdoConfig: Automatic odmometer gain calibration by GPS parameter added
 *	</li>
 *	<li>[SBGCOM-15] -         Motion profiles management
 *	</li>
 *	<li>[SBGCOM-16] -         sbgSaveSettings for centralized settings saving to flash memory
 *	</li>
 *	<li>[SBGCOM-17] -         sbgSetAdvancedOptions and sbgGetAdvancedOptions added
 *	</li>
 *	<li>[SBGCOM-18] -         sbgSetHeaveConf sbgGetHeaveConf added for heave configuration
 *	</li>
 *	<li>[SBGCOM-19] -         sbgSetAsciiOutputConf and sbgGetAsciiOutputConf
 *	</li>
 *	<li>[SBGCOM-20] -         sbgGetLowPowerModes now handles a sensor high performance mode
 *	</li>
 *	<li>[SBGCOM-40] -         Add Virtual odometer output trigger event
 *	</li>
 *	<li>[SBGCOM-45] -         NMEA device added option in sbgExtNmeaSetOptions: SBG_NMEA_OPT_HDT_AFTER_RMC for HDT frame timestamp management.
 *	</li>
 *	</ul>
 *	   
 *	<h2>        Removed feature
 *	</h2>
 *	<ul>
 *	<li>[SBGCOM-7] -         sbgSetVelocityConstraints and sbgGetVelocityConstraitns are now obsolete
 *	</li>
 *	<li>[SBGCOM-8] -         sbgCalibMag does not support onboard magnetic calibration anymore
 *	</li>
 *	<li>[SBGCOM-9] -         sbgSetFilterAttitudeErrors and sbgGetFilterAttitudeErrors are obsolete
 *	</li>
 *	<li>[SBGCOM-10] -         sbgSetFilterAttitudeOptions is obsolete IG-500
 *	</li>
 *	<li>[SBGCOM-11] -         sbgGetErrorLog command is obsolete
 *	</li>
 *	<li>[SBGCOM-13] -         sbgSetGpsOptions and sbgGetGpsOptions are now obsolete on IG-500
 *	</li>
 *	<li>[SBGCOM-42] -         Remote IG-Device Automatic orientation offset is obsolete
 *	</li>
 *	<li>[SBGCOM-44] -         Remote IG-Device &quot;remote frame&quot; system is obsolete
 *	</li>
 *	</ul>
 *	   
 *	<h2>        Task
 *	</h2>
 *	<ul>
 *	<li>[SBGCOM-5] -         sbgRestoreDefaultSettings behavior change
 *	</li>
 *	<li>[SBGCOM-21] -         User buffer size changed to 64 bytes
 *	</li>
 *	<li>[SBGCOM-37] -         sbgCanRestoreSettings now completely reset the sensor (full reboot)
 *	</li>
 *	<li>[SBGCOM-39] -         Now sbgCalibGyroBias command reception time out is always the default one
 *	</li>
 *	</ul>
 *	
 *	</td></tr>
 *
 *	<tr><td valign="top">23/03/11:</td><td>
 *	Version 1.5.0.0 sbgCom 1.5 released with support for IG-500E, triggers, syncIn/syncOut and RS-422.
 *	</td></tr>
 *
 *	<tr><td valign="top">18/03/11:</td><td>
 *	Version 1.4.2.6	Changed sbgProtocolReceive behavior. Now, SBG_NOT_READY is returned even if we have received an invalid frame.<br>
 *	Removed SBG_IN_BARO item in SbgLogicInType enumeration.
 *	Changed gpsTrueHeading type to int32.
 *	</td></tr>
 *
 *  <tr><td valign="top">17/03/11:</td><td>
 *	Version 1.4.2.5	Added fast/slow slew rate support for new device, the method sbgSetProtocolMode has changed!<br>
 *	The method sbgSetProtocolMode now returns the theoretical baud rate instead of the practical one and the prototype has changed, please update your code!<br>
 *  Added RS-232/RS-422 selection for external devices. <br>
 *	Added SYncIN/OUT features <br>
 *  Added Odometer commands output <br>
 *  Added True heading output <br>
 *  Removed Warnings in W4 compile mode <br>
 *  Added Remote IG device support for IG-500E <br>
 *  Added extended GPS fix info (DGPS/RTK data) <br>
 *  Added Sync IN location selection for sync IN channel 0 (IG-500E only)
 *  Added Data log parsing capability
 *  Removed external command for getting NMEA yaw offset.
 *  Added odometer direction intput in syncIn settings
 *  Added new IG-500N power saving modes
 *  Added GPS true heading valid flag in GPS info field
 *	</td></tr>
 *	<tr><td valign="top">05/07/10:</td><td>
 *	Version 1.4.2.2	Changed library initialization for better handling of serial com overloads.
 *	</td></tr>
 *
 *	<tr><td valign="top">21/06/10:</td><td>
 *	Version 1.4.2.2	Added sbgSetEscapeComm to comWrapper interface.
 *	</td></tr>
 *
 *	<tr><td valign="top">29/03/10:</td><td>
 *	Version 1.4.2.0	Added support for IG-500E:<br>
 *  Commands for selecting the external device and its options <br>
 *  Triggered output management and updated continuous mode configuration
 *	</td></tr>
 *
 *	<tr><td valign="top">16/11/09:</td><td>
 *	Version 1.4.0.0	New commands and output supported:<br>
 *  Added error log management <br>
 *	Added Velocity constraints management <br>
 *	Updated gyro calibration procedure: Added Medium and Fine procedures with longer bias calculation time <br>
 *	Added Device status output management: Self tests and in run status <br>
 *	Removed obsolete Kalman filter frequency output <br>
 *	Fixed a potential bug in sbgGetOrientationOffset when we receive an ACK instead of the real command answer.<br>
 *	Changed magnetometers calibration procedure.<br>
 *	Removed sbgCalibMagnetometersGetTempCompensatedValues function and SBG_CALIB_MAG_GET_TEMP_COMP_VALUES/SBG_CALIB_MAG_RET_TEMP_COMP_VALUES commands. You should use the output mask SBG_OUTPUT_MAG_CALIB_DATA. <br>
 *	Fixed a potential bug in sbgGetOrientationOffset when we receive an ACK instead of the real command answer.<br>
 *	Fixed sbgSetAutoOrientationOffset time out to 1 second.
 *	</td></tr>
 *
 *	<tr><td valign="top">24/08/09:</td><td>
 *	Version 1.3.1.0	Robustness improvements and bug fixes:<br>
 *  Fixed a syntax error in function sbgWaitForAck.<br>
 *	sbgWaitForAck now returns SBG_INVALID_FRAME if we have received an other frame than an ACK one.<br>
 *	Improved error handling for continuous mode operations by adding ContinuousErrorCallback function.<br>
 *	Changed ContinuousModeCallback parameters syntaxe.<br>
 *	Added SBG_NOT_CONTINUOUS_FRAME error code for sbgProtocolContinuousModeHandle function.<br>
 *	Fixed documentation error for baro altitude unit (mms to cms).
 *	</td></tr>
 *
 *	<tr><td valign="top">17/07/09:</td><td>
 *	Version 1.3.0.0 New commands and outputs supported:<br>
 *	Added support for user buffer management using commands SBG_SET_USER_BUFFER and SBG_GET_USER_BUFFER<br>
 *	Added support of Altitude above MSL or above Ellipsoid using GPS option mask SBG_GPS_ALTITUDE_ABOVE_MSL<br>
 *	Added support for Low power management using commands SBG_SET_LOW_POWER_MODE and SBG_GET_LOW_POWER_MODE<br>
 *	Added output UTC time using the mask SBG_OUTPUT_UTC_TIME_REFERENCE<br>
 *	Added output individual gyro temperature using the two masks SBG_OUTPUT_GYRO_TEMPERATURES and SBG_OUTPUT_GYRO_TEMPERATURES_RAW<br>
 *	Added SBG_PLATFORM_BIG_ENDIAN and SBG_PLATFORM_LITTLE_ENDIAN definitions for endianness support in sbgCom.<br>
 *	Changed gpsFlags specification, added SBG_GPS_VALID_TOW, SBG_GPS_VALID_WKN and SBG_GPS_VALID_UTC masks.<br>
 *	Fixed incorrect gpsLatitude and gpsLongitude in sbgOutput structure. These are now int32 instead of uint32.
 *	</td></tr>
 *
 *	<tr><td valign="top">18/02/09:</td><td>
 *	Version 1.2.0.0	sbgRestoreDefaultSettings has now 1 second time out.
 *	</td></tr>
 *
 *	<tr><td valign="top">08/01/09:</td><td>
 *	Version 1.1.7.5	bug fixes in read functions for unix platform.<br>
 *	Added a 1 second sleep in the command sbgRestoreDefaultSettings to wait until the device has changed it's settings.<br>
 *	Added sbgComErrorToString function used to convert an error code into a human readable string.<br>
 *	Added sbgComGetVersionAsString function used to retreive the library version using a string.<br>
 *	Changed SBG_SET_SENSORS_KALMAN_FREQUENCIES, SBG_GET_SENSORS_KALMAN_FREQUENCIES, SBG_RET_SENSORS_KALMAN_FREQUENCIES to<br>
 *	SBG_SET_FILTER_FREQUENCIES, SBG_GET_FILTER_FREQUENCIES and SBG_RET_FILTER_FREQUENCIES.
 *	</td></tr>
 *
 *	<tr><td valign="top">16/12/08:</td><td>
 *	Version 1.1.6.0	bug fixes in sbgProtocolContinuousModeHandle and sbgProtocolReceiveTimeOut.<br>
 *	When a non continous frame is received by sbgProtocolContinuousModeHandle, the frame is ignored.<br>
 *	When we receive an invalid frame in sbgProtocolReceiveTimeOut, we keep trying to get an answer until we reach the time out.
 *	</td></tr>
 *
 *	<tr><td valign="top">12/12/08:</td><td>
 *	Version 1.1.5.0	bug fixes in sbgProtocolReceive.<br>
 *	When a frame wasn't recevied completely, the function returns SBG_INVALID_FRAME instead of SBG_NOT_READY.
 *	</td></tr>
 *
 *	<tr><td valign="top">01/12/08:</td><td>
 *	Version 1.1.0.0	added support for IG-500N device and new doxygen documentation.<br><br>
 *	
 *	New commands for IG-30G and IG-500N only:
 *	- SBG_SET_REFERENCE_PRESSURE, SBG_GET_REFERENCE_PRESSURE, SBG_RET_REFERENCE_PRESSURE
 *	- SBG_GET_GPS_SVINFO, SBG_RET_GPS_SVINFO
 *	- SBG_SET_GPS_OPTIONS, SBG_GET_GPS_OPTIONS, SBG_RET_GPS_OPTIONS
 *
 *	New commands for IG-500N only:
 *	- SBG_SET_NAV_VELOCITY_SRC, SBG_GET_NAV_VELOCITY_SRC, SBG_RET_NAV_VELOCITY_SRC
 *	- SBG_SET_NAV_POSITION_SRC, SBG_GET_NAV_POSITION_SRC, SBG_RET_NAV_POSITION_SRC
 *	- SBG_SET_GPS_LEVER_ARM, SBG_GET_GPS_LEVER_ARM, SBG_RET_GPS_LEVER_ARM
 *	- SBG_SET_GRAVITY_MAGNITUDE, SBG_GET_GRAVITY_MAGNITUDE, SBG_RET_GRAVITY_MAGNITUDE
 *	- SBG_SEND_NAV_VELOCITY, SBG_SEND_NAV_POSITION
 *	
 *	New commands IG-30 and for IG-500 devices:
 *	- SBG_SEND_FILTER_HEADING
 *	- SBG_SET_MAGNETIC_DECLINATION, SBG_GET_MAGNETIC_DECLINATION, SBG_RET_MAGNETIC_DECLINATION
 *
 *	New outputs:
 *	- SBG_OUTPUT_POSITION
 *	- SBG_OUTPUT_VELOCITY
 *	- SBG_OUTPUT_ATTITUDE_ACCURACY
 *	- SBG_OUTPUT_NAVIGATION_ACCURACY
 *	
 *	</td></tr>
 *
 *	<tr><td valign="top">02/06/08:</td><td>
 *	Version 1.0.0.0 initial release.<br>
 *	</td></tr>
 *	</table>
 */
#define SBG_COM_VERSION		"2.0.0.0"
#define SBG_COM_VERSION_U	SBG_VERSION(2,0,0,0)
#define SBG_COM_VERSION_R	"2, 0, 0, 0\0"
#define SBG_COM_VERSION_W	2,0,0,0

#endif
