#include "commandsOdo.h"
#include "protocolOutputMode.h"


//----------------------------------------------------------------------//
//- Odometer commands                                                  -//
//----------------------------------------------------------------------//

/*!
 *	Set the main configuration of the external odometer channels
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	axis				Odometer measurement axis (in sensor coordinate frame) May be:
 *										- SBG_ODO_X
 *										- SBG_ODO_Y
 *										- SBG_ODO_Z
 *  \param[in]	pulsesPerMeter		decimal number of pulses per meter
 *  \param[in]	gainError			Error in percent on the previous gain value
 *  \param[in]	gpsGainCorrection	If set to TRUE, the GPS will automatically enhance the odometer gain 
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetOdoConfig(SbgProtocolHandle handle, uint8 channel, SbgOdoAxis axis, float pulsesPerMeter, uint8 gainError, bool gpsGainCorrection)
{
	uint8 buffer[4*sizeof(uint8)+sizeof(uint32)];
	uint8 gpsGainOpts;
	SbgErrorCode error;

	//
	// Set last field depending on GPS automatic correction
	//
	if (gpsGainCorrection)
	{
		gpsGainOpts = SBG_ODO_AUTO_GPS_GAIN | (gainError & (~SBG_ODO_AUTO_GPS_GAIN));
	}
	else
	{
		gpsGainOpts =  (gainError & (~SBG_ODO_AUTO_GPS_GAIN));
	}

	//
	// Build the params buffer
	//
	buffer[0]								= 0;
	buffer[1]								= channel;
	buffer[2]								= (uint8)axis;
	*(uint32*)(buffer + 3*sizeof(uint8))	= sbgHostToTargetFloat(handle->targetOutputMode, pulsesPerMeter);
	buffer[3*sizeof(uint8)+sizeof(uint32)]	= gpsGainOpts;
	
	//
	// Send the command used to set the odometer channel general config
	//
	error = sbgProtocolSend(handle, SBG_SET_ODO_CONFIG, buffer, 4*sizeof(uint8)+sizeof(uint32));

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
 *	Get the main configuration of the external odometer channels
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	pAxis				Odometer measurement axis (in sensor coordinate frame) May be:
 *										- SBG_ODO_X
 *										- SBG_ODO_Y
 *										- SBG_ODO_Z
 *  \param[out]	pPulsesPerMeter		decimal number of pulses per meter
 *  \param[out]	pGainError			Error in percent on the previous gain value
 *	\param[out]	pGpsGainCorrection	If set to TRUE, the GPS will automatically enhance the odometer gain
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOdoConfig(SbgProtocolHandle handle, uint8 channel, SbgOdoAxis *pAxis, float *pPulsesPerMeter, uint8 *pGainError, bool *pGpsGainCorrection)
{
	uint8 cmd;
	uint16 size;
	uint8 frame[3*sizeof(uint8)+sizeof(uint32)];
	SbgErrorCode error;

	//
	// Send the command used to get the external device params
	//
	error = sbgProtocolSend(handle, SBG_GET_ODO_CONFIG, &channel, sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, frame, &size, 3*sizeof(uint8)+sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			//
			// Check frame integrity
			//
			if ( (cmd == SBG_RET_ODO_CONFIG) && (size == 3*sizeof(uint8)+sizeof(uint32)) && (frame[0] == channel) )
			{
				//
				// Returns, if possible, the output parameters
				//
				if (pAxis)
				{
					*pAxis = (SbgOdoAxis)(frame[1]);
				}
				if (pPulsesPerMeter)
				{
					*pPulsesPerMeter = sbgTargetToHostFloat(handle->targetOutputMode, *(uint32*)(frame+2*sizeof(uint8)));
				}
				if (pGainError)
				{
					*pGainError = frame[2*sizeof(uint8) + sizeof(uint32)] & (~SBG_ODO_AUTO_GPS_GAIN);
				}
				if (pGpsGainCorrection)
				{
					if (frame[2*sizeof(uint8) + sizeof(uint32)] & SBG_ODO_AUTO_GPS_GAIN)
					{
						*pGpsGainCorrection = TRUE;
					}
					else
					{
						*pGpsGainCorrection = FALSE;
					}
				}
			}

			else if ((cmd == SBG_ACK) && (size == sizeof(uint8)))
			{
				//
				// Get the error code contained in the ACK frame
				//
				error = (SbgErrorCode)(frame[0]);

				//
				// We should receive an NACK so check if it's the case and return the error
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have received an invalid frame, maybe a question/answer desync!
					//
					error = SBG_INVALID_FRAME;
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
 *  Configures the odometer direction for the corresponding channel
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	odoDirection		Direction of the odometer. May be:
 *									- SBG_ODO_DIR_POSITIVE
 *									- SBG_ODO_DIR_NEGATIVE
 *									- SBG_ODO_DIR_AUTO
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetOdoDirection(SbgProtocolHandle handle, uint8 channel, SbgOdoDirection odoDirection)
{
	uint8 buffer[3*sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build the params buffer
	//
	buffer[0] = 0;
	buffer[1] = channel;
	buffer[2] = (uint8)odoDirection;

	//
	// Send the command used to set the odometer channel general config
	//
	error = sbgProtocolSend(handle, SBG_SET_ODO_DIRECTION, buffer, 3*sizeof(uint8));

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
 *  Get the odometer direction for the corresponding channel
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	pOdoDirection		Direction of the odometer. May be:
 *									- SBG_ODO_DIR_POSITIVE
 *									- SBG_ODO_DIR_NEGATIVE
 *									- SBG_ODO_DIR_AUTO
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOdoDirection(SbgProtocolHandle handle, uint8 channel, SbgOdoDirection *pOdoDirection)
{
	uint8 cmd;
	uint16 size;
	uint8 frame[2*sizeof(uint8)];
	SbgErrorCode error;

	//
	// Send the command used to get the external device params
	//
	error = sbgProtocolSend(handle, SBG_GET_ODO_DIRECTION, &channel, sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, frame, &size, 2*sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			//
			// Check frame integrity
			//
			if ( (cmd == SBG_RET_ODO_DIRECTION) && (size == 2*sizeof(uint8)) && (frame[0] == channel) )
			{
				//
				// Returns, if possible, the output parameters
				//
				if (pOdoDirection)
				{
					*pOdoDirection = (SbgOdoDirection)(frame[1]);
				}
			}
			else if ((cmd == SBG_ACK) && (size == sizeof(uint8)))
			{
				//
				// Get the error code contained in the ACK frame
				//
				error = (SbgErrorCode)(frame[0]);

				//
				// We should receive an NACK so check if it's the case and return the error
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have received an invalid frame, maybe a question/answer desync!
					//
					error = SBG_INVALID_FRAME;
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
 *  Configures the odometer lever arm for the corresponding channel
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	odoLeverArm			Lever arm of the odometer channel, with respect to the device. in meters.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetOdoLeverArm(SbgProtocolHandle handle, uint8 channel, const float odoLeverArm[3])
{
	uint8 buffer[2*sizeof(uint8)+3*sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build the params buffer
	//
	buffer[0]											= 0;
	buffer[1]											= channel;
	*(uint32*)(buffer+2*sizeof(uint8)+0*sizeof(uint32))	= sbgHostToTargetFloat(handle->targetOutputMode, odoLeverArm[0]);
	*(uint32*)(buffer+2*sizeof(uint8)+1*sizeof(uint32))	= sbgHostToTargetFloat(handle->targetOutputMode, odoLeverArm[1]);
	*(uint32*)(buffer+2*sizeof(uint8)+2*sizeof(uint32))	= sbgHostToTargetFloat(handle->targetOutputMode, odoLeverArm[2]);



	//
	// Send the command used to set the odometer channel general config
	//
	error = sbgProtocolSend(handle, SBG_SET_ODO_LEVER_ARM, buffer, 2*sizeof(uint8)+3*sizeof(uint32));

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
 *  Get the odometer lever arm for the corresponding channel
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	odoLeverArm			Lever arm of the odometer channel, with respect to the device. in meters.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOdoLeverArm(SbgProtocolHandle handle, uint8 channel, float odoLeverArm[3])
{
	uint8 cmd;
	uint16 size;
	uint8 frame[sizeof(uint8)+3*sizeof(uint32)];
	SbgErrorCode error;

	//
	// Send the command used to get the external device params
	//
	error = sbgProtocolSend(handle, SBG_GET_ODO_LEVER_ARM, &channel, sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, frame, &size, sizeof(uint8)+3*sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			//
			// Check frame integrity
			//
			if ( (cmd == SBG_RET_ODO_LEVER_ARM) && (size == (sizeof(uint8)+3*sizeof(uint32))) && (frame[0] == channel) )
			{
				//
				// Returns, if possible, the output parameters
				//
				if (odoLeverArm)
				{
					odoLeverArm[0] =  sbgTargetToHostFloat(handle->targetOutputMode, *(uint32*)(frame+sizeof(uint8)));
					odoLeverArm[1] =  sbgTargetToHostFloat(handle->targetOutputMode, *(uint32*)(frame+sizeof(uint8)+sizeof(uint32)));
					odoLeverArm[2] =  sbgTargetToHostFloat(handle->targetOutputMode, *(uint32*)(frame+sizeof(uint8)+2*sizeof(uint32)));
				}
			}
			else if ((cmd == SBG_ACK) && (size == sizeof(uint8)))
			{
				//
				// Get the error code contained in the ACK frame
				//
				error = (SbgErrorCode)(frame[0]);

				//
				// We should receive an NACK so check if it's the case and return the error
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have received an invalid frame, maybe a question/answer desync!
					//
					error = SBG_INVALID_FRAME;
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
