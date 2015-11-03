#include "commandsGps.h"
#include "protocolOutputMode.h"
#include "commands.h"
#include <string.h>

/*!
 *	Configures the reference pressure used for altitude calculation. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	reference			Reference pressure at ground in pascals.<br>
 *									If set to 0, the current pressure is considered as ground pressure.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetReferencePressure(SbgProtocolHandle handle, bool permanent, uint32 reference)
{
	uint8 buffer[sizeof(uint8)+sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	*buffer = permanent;
	*(uint32*)(buffer+sizeof(uint8)) = sbgHostToTarget32(handle->targetOutputMode, reference);

	//
	// Send the command used to set the reference pressure
	//
	error = sbgProtocolSend(handle, SBG_SET_REFERENCE_PRESSURE, buffer, sizeof(uint8)+sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// Wait for an answer
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}
	
	return error;
}

/*!
 *	Get the reference pressure used for altitude calculation. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	pReference			Pointer to the reference pressure in pascales.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetReferencePressure(SbgProtocolHandle handle, uint32 *pReference)
{
	uint8 cmd;
	uint16 size;
	uint32 referencePressure;
	SbgErrorCode error;

	//
	// Send the command used to get our reference pressure
	//
	error = sbgProtocolSend(handle, SBG_GET_REFERENCE_PRESSURE, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &referencePressure, &size, sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_REFERENCE_PRESSURE) && (size == sizeof(uint32)) )
			{
				//
				// Returns, if possible, our reference pressure
				//
				if (pReference)
				{
					*pReference = sbgTargetToHost32(handle->targetOutputMode, referencePressure);
				}
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}

/*!
 *	Configures the advanced GPS options. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *	\param[in]	model				Dynamic platform model:
 * 									- SBG_GPS_MODEL_STATIONARY<br>
 * 									- SBG_GPS_MODEL_PEDESTRIAN<br>
 * 									- SBG_GPS_MODEL_AUTOMOTIVE<br>
 * 									- SBG_GPS_MODEL_SEA<br>
 * 									- SBG_GPS_MODEL_AIRBONE_1G<br>
 * 									- SBG_GPS_MODEL_AIRBONE_2G<br>
 * 									- SBG_GPS_MODEL_AIRBONE_4G<br>
 *  \param[in]	options				GPS options using masks, such as SBAS corrections.<br>
 *									Available options masks are:
 *									- SBG_GPS_DISABLE_SBAS<br>
 *									- SBG_GPS_ENABLE_SBAS_DIFF_CORRECTIONS<br>
 *									- SBG_GPS_ENABLE_SBAS_RANGING<br>
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgSetGpsOptions(SbgProtocolHandle handle, bool permanent, SbgGpsDynamicModel model, uint8 options)
{
	uint8 buffer[3*sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	buffer[0] = permanent;
	buffer[1] = (uint8)model;
	buffer[2] = options;

	//
	// Send the command used to set the GPS options
	//
	error = sbgProtocolSend(handle, SBG_SET_GPS_OPTIONS, buffer, 3*sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Wait for an answer
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}
	
	return error;
}

/*!
 *	Get the advanced GPS options. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	pModel				Dynamic platform model:
 * 									- SBG_GPS_MODEL_STATIONARY<br>
 * 									- SBG_GPS_MODEL_PEDESTRIAN<br>
 * 									- SBG_GPS_MODEL_AUTOMOTIVE<br>
 * 									- SBG_GPS_MODEL_SEA<br>
 * 									- SBG_GPS_MODEL_AIRBONE_1G<br>
 * 									- SBG_GPS_MODEL_AIRBONE_2G<br>
 * 									- SBG_GPS_MODEL_AIRBONE_4G<br>
 *  \param[out]	pOptions			GPS options using masks, such as SBAS corrections.<br>
 *									Available options masks are:
 *									- SBG_GPS_DISABLE_SBAS<br>
 *									- SBG_GPS_ENABLE_SBAS_DIFF_CORRECTIONS<br>
 *									- SBG_GPS_ENABLE_SBAS_RANGING<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetGpsOptions(SbgProtocolHandle handle, SbgGpsDynamicModel *pModel, uint8 *pOptions)
{
	uint8 cmd;
	uint8 buffer[2];
	uint16 size;
	SbgErrorCode error;

	//
	// Send the command used to get our GPS options
	//
	error = sbgProtocolSend(handle, SBG_GET_GPS_OPTIONS, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, 2 * sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_GPS_OPTIONS) && (size == 2*sizeof(uint8)) )
			{
				//
				// Returns, if possible, our GPS dynamic model
				//
				if (pModel)
				{
					*pModel = (SbgGpsDynamicModel)buffer[0];
				}

				//
				// Returns, if possible, our GPS options
				//
				if (pOptions)
				{
					*pOptions = buffer[1];
				}
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}
