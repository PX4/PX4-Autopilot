/*!
 *	\file		commandsSync.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		06/09/10
 *
 *	\brief		Commands used to configure the IG-500 Synchronization features.<br>
 *				Copyright 2007-2010 SBG Systems. All rights reserved.
 */
#ifndef __COMMANDS_SYNC_H__
#define __COMMANDS_SYNC_H__

#include "../sbgCommon.h"
#include "protocol.h"

//----------------------------------------------------------------------//
// Logic inputs Internal definitions                                    //
//----------------------------------------------------------------------//

/*! Bit mask for isolating the sensitivity in the sync in options param ( rising/falling edge) */
#define SBG_IN_SENSE_MASK			(0x03)

//----------------------------------------------------------------------//
// Logic inputs types definition                                        //
//----------------------------------------------------------------------//

/*!
 * Logic and synchronization inputs types
 */
typedef enum _SbgLogicInType
{
	SBG_IN_DISABLED				= 0x00,			/*!< Input channel disabled */
	SBG_IN_EVENT				= 0x01,			/*!< General purpose event trigger */
	SBG_IN_TIME_PULSE			= 0x03,			/*!< GPS PPS time input */
	SBG_IN_ODOMETER				= 0x05,			/*!< Odometer input */
	SBG_IN_ODOMETER_DIRECTION	= 0x06			/*!< Direction sense for the other odometer channel. <br>
												 *   Compatible with quadrature output encoders (signal B) */
} SbgLogicInType;

/*!
 * Logic input sensitivity types
 */
typedef enum _SbgLogicInSensitivity
{
	SBG_IN_FALLING_EDGE			= 0x00,			/*!< The trigger will be activated by a falling edge */
	SBG_IN_RISING_EDGE			= 0x01,			/*!< The trigger will be activated by a rising edge */
	SBG_IN_LEVEL_CHANGE			= 0x02			/*!< The trigger is activated by a level change (rising or falling edge) */
} SbgLogicInSensitivity;

/*!
 * Logic input channel 0 physical location on IG-500E
 */
typedef enum _SbgSyncInLocation
{
	SBG_IN_STD_LOCATION			= 0x00,			/*!< Sync In ch0 is located on the main connector */
	SBG_IN_EXT_LOCATION			= 0x04			/*!< Sync In ch0 is located on the extended connector */
} SbgLogicInLocation;

/*!
 * Logic output and synchronization  types
 */
typedef enum _SbgLogicOutputType
{
	SBG_OUT_DISABLED			= 0x00,			/*!< Input channel disabled */
	SBG_OUT_MAIN_LOOP_START		= 0x01,			/*!< Main loop starting trigger */
	SBG_OUT_MAIN_LOOP_DIVIDER	= 0x02,			/*!< Trigger activated at the beginning of each main loop where a continuous output is generated */
	SBG_OUT_TIME_PULSE_COPY		= 0x03,			/*!< Copy of the GPS time pulse input trigger */
	SBG_OUT_VIRTUAL_ODO			= 0x05			/*!< Virtual odometer logic output: Enabled each x meters of travel */
} SbgLogicOutType;

/*!
 * Logic output polarity
 */
typedef enum _SbgLogicOutPolarity
{
	SBG_OUT_FALLING_EDGE		= 0x00,			/*!< The output pin will generate a falling edge */
	SBG_OUT_RISING_EDGE			= 0x01,			/*!< The output pin will generate a rising edge */
	SBG_OUT_TOGGLE				= 0x02			/*!< The pulse is a level change */
} SbgLogicOutPolarity;

//----------------------------------------------------------------------//
//- Synchronization input and output operations                        -//
//----------------------------------------------------------------------//

/*!
 *	Set a logic input channel setting
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	channel				Channel to configure. May Be 0 or 1
 *	\param[in]	inputType			Type of the logic input, may be <br>
 * 										- SBG_IN_DISABLED
 *										- SBG_IN_EVENT
 *										- SBG_IN_TIME_PULSE
 * 										- SBG_IN_ODOMETER
 * \param[in]	sensitivity			Sensitivity of the trigger. It may be:<br>
 * 										- SBG_IN_FALLING_EDGE
 * 										- SBG_IN_RISING_EDGE
 * 										- SBG_IN_LEVEL_CHANGE
 * \param[in]	location			Physical location of the input pin for the syncIN channel 0 on IG-500E. <br>
 *										- SBG_IN_STD_LOCATION (default value, leave to this value if not used)
 *										- SBG_IN_EXT_LOCATION
 * \param[in]	nsDelay				Delay to be added before the actual trigger is taken into account (in nano seconds) delays up to 2seconds are allowed:<br>
 *									This delay is only used when the input is set to:
 * 										- SBG_IN_EVENT
 * 										- SBG_IN_MAIN_LOOP_START
 *										- SBG_IN_TIME_PULSE <br>
 * 									When used with the time pulse event, this delay can be negative, in order to simulate the time pulse propagation time
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgSetLogicInChannel(SbgProtocolHandle handle, uint8 channel, SbgLogicInType inputType, SbgLogicInSensitivity sensitivity, SbgLogicInLocation location, int32 nsDelay);

/*!
 *	Get a logic input channel setting
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	channel				Channel to configure. May Be 0 or 1
 *	\param[out]	pInputType			Type of the logic input, may be <br>
 * 										- SBG_IN_DISABLED
 *										- SBG_IN_EVENT
 *										- SBG_IN_TIME_PULSE
 * 										- SBG_IN_ODOMETER	
 * \param[out]	pSensitivity		Sensitivity of the trigger. It may be:<br>
 * 										- SBG_IN_FALLING_EDGE
 * 										- SBG_IN_RISING_EDGE
 * 										- SBG_IN_LEVEL_CHANGE
 * \param[out]	pLocation			Physical location of the input pin for the syncIN channel 0 on IG-500E. <br>
 *										- SBG_IN_STD_LOCATION
 *										- SBG_IN_EXT_LOCATION
 * \param[out]	pNsDelay			Delay added before the actual trigger is taken into account (in nano seconds) delays up to +2seconds are possible:<:<br>
 *									This delay is only valid when the input is set to:
 * 										- SBG_IN_EVENT
 * 										- SBG_IN_MAIN_LOOP_START
 *										- SBG_IN_TIME_PULSE <br>
 * 									When used with the time pulse event, this delay can be negative, in order to simulate the time pulse propagation time
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgGetLogicInChannel(SbgProtocolHandle handle, uint8 channel, SbgLogicInType *pInputType, SbgLogicInSensitivity *pSensitivity, SbgLogicInLocation *pLocation, int32 *pNsDelay);


/*!
 *	Set a logic output channel setting
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	channel				Channel to configure. Set always 0
 *	\param[in]	outputType			Type of the logic output, may be <br>
 * 										- SBG_OUT_DISABLED
 *										- SBG_OUT_MAIN_LOOP_START
 *										- SBG_OUT_MAIN_LOOP_DIVIDER
 *										- SBG_OUT_TIME_PULSE_COPY
 *										- SBG_OUT_VIRTUAL_ODO
 * \param[in]	polarity			Polarity of the out pulse. It may be:
 * 										- SBG_OUT_FALLING_EDGE
 * 										- SBG_OUT_RISING_EDGE
 * 										- SBG_OUT_TOGGLE
 *  \param[in]	duration			When the polarity is set to SBG_OUT_FALLING_EDGE or SBG_OUT_RISING_EDGE, this is the pulse duration in ms. <br>
 *									Leave to 0 if not used.
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgSetLogicOutChannel(SbgProtocolHandle handle, uint8 channel, SbgLogicOutType outputType, SbgLogicOutPolarity polarity, uint8 duration);

/*!
 *	Get a logic output channel setting
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	channel				Channel to configure. Leave always to 0
 *	\param[out]	pOutputType			Type of the logic output, may be <br>
 * 										- SBG_OUT_DISABLED
 *										- SBG_OUT_MAIN_LOOP_START
 *										- SBG_OUT_MAIN_LOOP_DIVIDER
 *										- SBG_OUT_TIME_PULSE_COPY
 *										- SBG_OUT_VIRTUAL_ODO
 * \param[out]	pPolarity			Polarity of the out pulse. It may be:
 * 										- SBG_OUT_FALLING_EDGE
 * 										- SBG_OUT_RISING_EDGE
 * 										- SBG_OUT_TOGGLE
 *  \param[out]	pDuration			When the polarity is set to SBG_OUT_FALLING_EDGE or SBG_OUT_RISING_EDGE, this is the pulse duration in ms. <br>
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgGetLogicOutChannel(SbgProtocolHandle handle, uint8 channel, SbgLogicOutType *pOutputType, SbgLogicOutPolarity *pPolarity, uint8 *pDuration);

//----------------------------------------------------------------------//
//- Virtual odometer configuration                                     -//
//----------------------------------------------------------------------//

/*!
 *	Defines the distance between two pulses when sync out is configured as a virtual odometer
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	distance				Distance in meters
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetVirtualOdoConf(SbgProtocolHandle handle, float distance);

/*!
 *	Retrieves the distance between two pulses when sync out is configured as a virtual odometer
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pDistance				Distance in meters
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetVirtualOdoConf(SbgProtocolHandle handle, float *pDistance);

#endif
