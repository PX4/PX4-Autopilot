/*!
 *	\file		protocolOutputMode.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		19/07/07
 *
 *	\brief		Output format handling system for protocol use.<br>
 *				This file handles endianness and fixed/float issues.<br>
 *				Using one of the two definitions SBG_PLATFORM_BIG_ENDIAN and SBG_PLATFORM_LITTLE_ENDIAN,<br>
 *				you can define the endianness of the platform you are compiling sbgCom for.<br>
 *				If your platform is in little endian, just define the preprocessor define SBG_PLATFORM_LITTLE_ENDIAN.<br>
 *				If your platform is in big endian, you should define the define SBG_PLATFORM_BIG_ENDIAN.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef __PROTOCOL_OUPUT_MODE_H__
#define __PROTOCOL_OUPUT_MODE_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Methods used to convert data from host to target format            -//
//----------------------------------------------------------------------//

/*!
 *	Convert a 16 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint16 sbgHostToTarget16(uint8 targetOutputMode, uint16 val);

/*!
 *	Convert a 32 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint32 sbgHostToTarget32(uint8 targetOutputMode, uint32 val);

/*!
 *	Convert a 64 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint64 sbgHostToTarget64(uint8 targetOutputMode, uint64 val);

/*!
 *	Convert a 32 bits float from host endianness to a float or fixed32 in target endianness
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in host endianness in float.
 *	\return								Value in target endianness in fixed32 or float.
 */
uint32 sbgHostToTargetFloat(uint8 targetOutputMode, float val);

/*!
 *	Convert a 64 bits double from host endianness to a double or fixed64 in target endianness
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in host endianness in double.
 *	\return								Value in target endianness in fixed64 or double.
 */
uint64 sbgHostToTargetDouble(uint8 targetOutputMode, double val);

//----------------------------------------------------------------------//
//-	Methods used to convert data from target to host format            -//
//----------------------------------------------------------------------//

/*!
 *	Convert a 16 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint16 sbgTargetToHost16(uint8 targetOutputMode, uint16 val);

/*!
 *	Convert a 32 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint32 sbgTargetToHost32(uint8 targetOutputMode, uint32 val);

/*!
 *	Convert a 64 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness (little endian).
 *	\return								Value in host endianness.
 */
uint64 sbgTargetToHost64(uint8 targetOutputMode, uint64 val);

/*!
 *	Convert a 32 bits float or fixed32 from target endianness to a float in host endianness
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness in fixed32 or float.
 *	\return								Value in host endianness in float.
 */
float sbgTargetToHostFloat(uint8 targetOutputMode, uint32 val);

/*!
 *	Convert a 64 bits fixed or double in big or little endian value into a 64 bits double value in little endian.
 *	\param[in]	targetOutputMode		The output mode we are using.
 *	\param[in]	val						The value in double/fixed in little/big endian to convert.
 *	\return								A 64 bits double value in little endian.
 */
double sbgTargetToHostDouble(uint8 targetOutputMode, uint64 val);

//----------------------------------------------------------------------//
//-	Internal methods                                                   -//
//----------------------------------------------------------------------//

/*!
 * Swap a 16 bits integer
 * \param[in]	x			Input integer
 * \return					x bytewise swapped
 */
uint16 swap16(uint16 x);

/*!
 * Swap a 32 bits integer
 * \param[in]	x			Input integer
 * \return					x bytewise swapped
 */
uint32 swap32(uint32 x);

/*!
 * Swap a 64 bits integer
 * \param[in]	x			Input integer
 * \return					x bytewise swapped
 */
uint64 swap64(uint64 x);


#endif	// __PROTOCOL_OUT_FORMAT_H__
