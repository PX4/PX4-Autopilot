#include "protocolOutputMode.h"
#include "commands.h"

//----------------------------------------------------------------------//
//- Internal swap functions                                            -//
//----------------------------------------------------------------------//

// Union used to store both float and integer in raw format (usefull to avoid types conversions)
typedef union _SwapUnion32
{
	float valFloat;
	int32 valInt;
} SwapUnion32;

// Union used to store both double and integer in raw format (usefull to avoid types conversions)
typedef union _SwapUnion64
{
	double valFloat;
	int64 valInt;
} SwapUnion64;

// Swap a 16 bits integer
uint16 swap16(uint16 x) 
{
	return((x<<8)|(x>>8));
}

// Swap a 32 bits integer
uint32 swap32(uint32 x) 
{
	return((x<<24)|((x<<8)&0x00FF0000)|((x>>8)&0x0000FF00)|(x>>24));
}

// Swap a 64 bits integer
uint64 swap64(uint64 x) 
{
	uint32 hi, lo;

	//
	// Separate into high and low 32-bit values
	//
	lo = (uint32)(x&0xFFFFFFFF);
	x >>= 32;
	hi = (uint32)(x&0xFFFFFFFF);

	//
	// Swap each part and rebuild the 64 bit vale
	//
	x = swap32(lo);
	x <<= 32;
	x |= swap32(hi);

	return x;
}

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
uint16 sbgHostToTarget16(uint8 targetOutputMode, uint16 val)
{
#if defined SBG_PLATFORM_BIG_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian and the platform is in big endian
		return swap16(val);
	}
	else
	{
		// The target is in big endian so is the platform
		return val;
	}
#elif defined SBG_PLATFORM_LITTLE_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian so is the platform
		return val;
	}
	else
	{
		// The target is in big endian and the platform is in little endian
		return swap16(val);
	}
#else
	#error	sbgHostToTarget16: You have to define your platform endianness!
#endif
}

/*!
 *	Convert a 32 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint32 sbgHostToTarget32(uint8 targetOutputMode, uint32 val)
{
#if defined SBG_PLATFORM_BIG_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian and the platform is in big endian
		return swap32(val);
	}
	else
	{
		// The target is in big endian so is the platform
		return val;
	}
#elif defined SBG_PLATFORM_LITTLE_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian so is the platform
		return val;
	}
	else
	{
		// The target is in big endian and the platform is in little endian
		return swap32(val);
	}
#else
	#error	sbgHostToTarget32: You have to define your platform endianness!
#endif
}

/*!
 *	Convert a 64 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint64 sbgHostToTarget64(uint8 targetOutputMode, uint64 val)
{
#if defined SBG_PLATFORM_BIG_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian and the platform is in big endian
		return swap64(val);
	}
	else
	{
		// The target is in big endian so is the platform
		return val;
	}
#elif defined SBG_PLATFORM_LITTLE_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian so is the platform
		return val;
	}
	else
	{
		// The target is in big endian and the platform is in little endian
		return swap64(val);
	}
#else
	#error	sbgHostToTarget64: You have to define your platform endianness!
#endif
}

/*!
 *	Convert a 32 bits float from host endianness to a float or fixed32 in target endianness
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in host endianness in float.
 *	\return								Value in target endianness in fixed32 or float.
 */
uint32 sbgHostToTargetFloat(uint8 targetOutputMode, float val)
{
	SwapUnion32 rawVal;

	//
	// First convert the value into fixed point if needed
	//
	if (targetOutputMode&SBG_OUTPUT_MODE_FIXED)
	{
		//
		// The target is using fixed point so we have to convert the float into fixed point format
		//
		rawVal.valInt = (uint32)(val*0x100000);
	}
	else
	{
		//
		// The target is using float numbers so we just have to store the float in a raw uint32
		//
		rawVal.valFloat = val;
	}

	//
	// Handle endianess issues
	//
	return sbgHostToTarget32(targetOutputMode, rawVal.valInt);
}

/*!
 *	Convert a 64 bits double from host endianness to a double or fixed64 in target endianness
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in host endianness in double.
 *	\return								Value in target endianness in fixed64 or double.
 */
uint64 sbgHostToTargetDouble(uint8 targetOutputMode, double val)
{
	SwapUnion64 rawVal;

	//
	// First convert the value into fixed point if needed
	//
	if (targetOutputMode&SBG_OUTPUT_MODE_FIXED)
	{
		//
		// The target is using fixed point so we have to convert the double into fixed point format
		//
		rawVal.valInt = (uint64)(val*0x100000000LL);
	}
	else
	{
		//
		// The target is using float numbers so we just have to store the float in a raw uint32
		//
		rawVal.valFloat = val;
	}

	//
	// Handle endianess issues
	//
	return sbgHostToTarget64(targetOutputMode, rawVal.valInt);
}

//----------------------------------------------------------------------//
//- Methods used to convert data from target to host format            -//
//----------------------------------------------------------------------//

/*!
 *	Convert a 16 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint16 sbgTargetToHost16(uint8 targetOutputMode, uint16 val)
{
#if defined SBG_PLATFORM_BIG_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian and the platform is in big endian
		return swap16(val);
	}
	else
	{
		// The target is in big endian so is the platform
		return val;
	}
#elif defined SBG_PLATFORM_LITTLE_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian so is the platform
		return val;
	}
	else
	{
		// The target is in big endian and the platform is in little endian
		return swap16(val);
	}
#else
	#error	sbgHostToTarget16: You have to define your platform endianness!
#endif
}

/*!
 *	Convert a 32 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint32 sbgTargetToHost32(uint8 targetOutputMode, uint32 val)
{
#if defined SBG_PLATFORM_BIG_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian and the platform is in big endian
		return swap32(val);
	}
	else
	{
		// The target is in big endian so is the platform
		return val;
	}
#elif defined SBG_PLATFORM_LITTLE_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian so is the platform
		return val;
	}
	else
	{
		// The target is in big endian and the platform is in little endian
		return swap32(val);
	}
#else
	#error	sbgHostToTarget32: You have to define your platform endianness!
#endif
}

/*!
 *	Convert a 64 bits integer from target to host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness (little endian).
 *	\return								Value in host endianness.
 */
uint64 sbgTargetToHost64(uint8 targetOutputMode, uint64 val)
{
#if defined SBG_PLATFORM_BIG_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian and the platform is in big endian
		return swap64(val);
	}
	else
	{
		// The target is in big endian so is the platform
		return val;
	}
#elif defined SBG_PLATFORM_LITTLE_ENDIAN
	if (targetOutputMode&SBG_OUTPUT_MODE_LITTLE_ENDIAN)
	{
		// The target is in little endian so is the platform
		return val;
	}
	else
	{
		// The target is in big endian and the platform is in little endian
		return swap64(val);
	}
#else
	#error	sbgHostToTarget64: You have to define your platform endianness!
#endif
}

/*!
 *	Convert a 32 bits float or fixed32 from target endianness to a float in host endianness
 *	\param[in]	targetOutputMode		The output mode we are using, could be a combinaison of:<br>
 *											- #SBG_OUTPUT_MODE_BIG_ENDIAN or #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *											- #SBG_OUTPUT_MODE_FLOAT or #SBG_OUTPUT_MODE_FIXED<br>
 *	\param[in]	val						Value in target endianness in fixed32 or float.
 *	\return								Value in host endianness in float.
 */
float sbgTargetToHostFloat(uint8 targetOutputMode, uint32 val)
{
	SwapUnion32 rawVal;

	//
	// First handle endianess issues
	//
	rawVal.valInt = sbgTargetToHost32(targetOutputMode, val);

	//
	// Then convert the value into float if needed
	//
	if (targetOutputMode&SBG_OUTPUT_MODE_FIXED)
	{
		//
		// The target is using fixed point so we have to convert the fixed point into float format
		//
		return ((float)rawVal.valInt)/1048576.0f;
	}
	else
	{
		//
		// The target is using float numbers so we just have to return the float value
		//
		return rawVal.valFloat;
	}
}

/*!
 *	Convert a 64 bits fixed or double in big or little endian value into a 64 bits double value in little endian.
 *	\param[in]	targetOutputMode		The output mode we are using.
 *	\param[in]	val						The value in double/fixed in little/big endian to convert.
 *	\return								A 64 bits double value in little endian.
 */
double sbgTargetToHostDouble(uint8 targetOutputMode, uint64 val)
{
	SwapUnion64 rawVal;

	//
	// First handle endianess issues
	//
	rawVal.valInt = sbgTargetToHost64(targetOutputMode, val);

	//
	// Then convert the value into float if needed
	//
	if (targetOutputMode&SBG_OUTPUT_MODE_FIXED)
	{
		//
		// The target is using fixed point so we have to convert the fixed point into float format
		//
		return ((double)rawVal.valInt)/4294967296.0;
	}
	else
	{
		//
		// The target is using float numbers so we just have to return the float value
		//
		return rawVal.valFloat;
	}
}
