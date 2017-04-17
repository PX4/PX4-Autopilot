/*!
 *	\file		sbgCommon.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		18/07/07
 *
 *	\brief		Generic header file for all SBG Systems projects.<br>
 *				It contains all types definitions, error codes.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef __SBG_COMMON_H__
#define __SBG_COMMON_H__

#define SBG_PLATFORM_LITTLE_ENDIAN

//----------------------------------------------------------------------//
//- Global definitions                                                 -//
//----------------------------------------------------------------------//
#ifndef FALSE
	#define FALSE	0
#endif

#ifndef TRUE
	#define TRUE	1
#endif

#ifndef NULL
	#define NULL	0
#endif

#ifndef SBG_INVALID_HANDLE
	#define SBG_INVALID_HANDLE	0xFFFFFFFF
#endif

#ifndef SBG_NOT_FOUND
	#define SBG_NOT_FOUND			0xFFFFFFFF
#endif

#ifdef __cplusplus
	#define SBG_DELETE(p)		if (p){delete (p); (p) = NULL;}
	#define SBG_DELETE_ARRAY(p)	if (p){delete[] (p); (p) = NULL;}
	#define SBG_FREE(p)			if (p){free(p); (p) = NULL;}
	#define SBG_FREE_ARRAY(p)	if (p){free(p); (p) = NULL;}
#else
	#define SBG_DELETE			if (p){free(p); (p) = NULL;}
	#define SBG_DELETE_ARRAY	if (p){free(p); (p) = NULL;}
	#define SBG_FREE(p)			if (p){free(p); (p) = NULL;}
	#define SBG_FREE_ARRAY(p)	if (p){free(p); (p) = NULL;}
#endif

//----------------------------------------------------------------------//
//- Custom types definitions                                           -//
//----------------------------------------------------------------------//
#ifdef WIN32
#ifndef __cplusplus
typedef	unsigned	__int8		bool;	//  8 bits
#endif
typedef unsigned	__int8	uint8;		//  8 bits
typedef unsigned	__int16	uint16;		// 16 bits
typedef unsigned	__int32	uint32;		// 32 bits
typedef unsigned	__int64	uint64;		// 64 bits

typedef signed      __int8	int8;		//  8 bits
typedef signed		__int16	int16;		// 16 bits
typedef signed		__int32	int32;		// 32 bits
typedef signed		__int64	int64;		// 64 bits

typedef int32				fixed32;	// 32 bits fixed point
typedef int64				fixed64;	// 64 bits fixed point
#else
#include <sys/types.h>
#ifndef __cplusplus
#ifndef bool
typedef	uint8_t	bool;				//  8 bits
#endif
#endif
typedef uint8_t	uint8;				//  8 bits
typedef uint16_t	uint16;				// 16 bits
typedef uint32_t	uint32;				// 32 bits
typedef uint64_t	uint64;				// 64 bits

typedef int8_t  	int8;				//  8 bits
typedef int16_t		int16;				// 16 bits
typedef int32_t		int32;				// 32 bits
typedef int64_t		int64;				// 64 bits

typedef int32				fixed32;	// 32 bits fixed point
typedef int64				fixed64;	// 64 bits fixed point
#endif

//----------------------------------------------------------------------//
//  MAths definitions                                                   //
//----------------------------------------------------------------------//
#ifndef SBG_PI
#define SBG_PI 3.14159265358979323846
#endif

#define SBG_RAD_TO_DEG(x)		((x)*180.0f/SBG_PI)	/*!< Convert a radian value into a degree value */
#define SBG_DEG_TO_RAD(x)		((x)/180.0f*SBG_PI)	/*!< Convert a degree value into a radian value */

//----------------------------------------------------------------------//
//- Errors code definitions                                            -//
//----------------------------------------------------------------------//

/*!
 *	Generic errors definitions for SBG Systems projects.
 */
typedef enum _SbgErrorCode
{
	SBG_NO_ERROR = 0,						/*!< The operation was successfully executed. */
	SBG_ERROR,								/*!< We have a generic error. */
	SBG_NULL_POINTER,						/*!< A pointer is null. */
	SBG_INVALID_CRC,						/*!< The received frame has an invalid CRC. */
	SBG_INVALID_FRAME,						/*!< The received frame is invalid <br> */
											/*!<	We have received an unexpected frame (not the cmd we are waiting for or with an invalid data size.<br> */
											/*!<	This could be caused by a desync between questions and answers.<br> */
											/*!<	You should flush the serial port to fix this. */
	SBG_TIME_OUT,							/*!< We have started to receive a frame but not the end. */
	SBG_WRITE_ERROR,						/*!< All bytes hasn't been written. */
	SBG_READ_ERROR,							/*!< All bytes hasn't been read. */
	SBG_BUFFER_OVERFLOW,					/*!< A buffer is too small to contain so much data. */
	SBG_INVALID_PARAMETER,					/*!< An invalid parameter has been found. */
	SBG_NOT_READY,							/*!< A device isn't ready (Rx isn't ready for example). */
	SBG_MALLOC_FAILED,						/*!< Failed to allocate a buffer. */
	SGB_CALIB_MAG_NOT_ENOUGH_POINTS,		/*!< Not enough points were available to perform magnetometers calibration. */
	SBG_CALIB_MAG_INVALID_TAKE,				/*!< The calibration procedure could not be properly executed due to insufficient precision. */
	SBG_CALIB_MAG_SATURATION,				/*!< Saturation were detected when attempt to calibrate magnetos. */
	SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE,	/*!< 2D calibration procedure could not be performed. */

	SBG_DEVICE_NOT_FOUND,					/*!< A device couldn't be founded or opened PC only error code */
	SBG_OPERATION_CANCELLED,				/*!< An operation was cancelled.  PC only error code*/
	SBG_NOT_CONTINUOUS_FRAME,				/*!< We have received a frame that isn't a continuous one. PC only error code*/

	SBG_INCOMPATIBLE_HARDWARE				/*!< Hence valid; the command cannot be executed because of hardware incompatibility */

} SbgErrorCode;

//----------------------------------------------------------------------//
//- Versions system definitions                                        -//
//----------------------------------------------------------------------//
#define SBG_VERSION_MAJOR(maj)		((maj)<<24)
#define SBG_VERSION_MINOR(min)		((min)<<16)
#define SBG_VERSION_REV(rev)		((rev)<<8)
#define SBG_VERSION_BUILD(build)	(build)
#define SBG_VERSION(maj,min,rev,build)	SBG_VERSION_MAJOR(maj)|SBG_VERSION_MINOR(min)|SBG_VERSION_REV(rev)|SBG_VERSION_BUILD(build)

#define SBG_VERSION_GET_MAJOR(version)	(((version)>>24)&0xFF)	/*!< Extract the major number from a version integer. */
#define SBG_VERSION_GET_MINOR(version)	(((version)>>16)&0xFF)	/*!< Extract the minor number from a version integer. */
#define SBG_VERSION_GET_REV(version)	(((version)>>8 )&0xFF)	/*!< Extract the revision number from a version integer. */
#define SBG_VERSION_GET_BUILD(version)	( (version)    &0xFF)	/*!< Extract the build number from a version integer. */

#endif	// __SBG_COMMON_H__
