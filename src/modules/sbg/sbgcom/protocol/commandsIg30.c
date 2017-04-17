#include "commandsIg30.h"
#include "protocolOutputMode.h"
#include "commands.h"
#include <string.h>

//----------------------------------------------------------------------//
//- Sensor sampling and orientation computation relative commands      -//   
//----------------------------------------------------------------------//

/*!
 *	Defines some options regarding the Kalman Filter.<br>
 *	It's possible, for example, to enable/disable the attitude computation or to enable/disable gyro-bias estimation.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	filterOptions			The Kalman filter options mask.<br>
 *										Possible values are:<br>
 *										- #SBG_FILTER_OPTION_FILTER_ACCELERATIONS
 *										- #SBG_FILTER_OPTION_FILTER_MAG_DISTORTIONS
 *										- #SBG_FILTER_OPTION_ESTIMATE_GYRO_BIAS
 *										- #SBG_FILTER_OPTION_ESTIMATE_ACCEL_BIAS
 *										- #SBG_FILTER_OPTION_ENABLE_ATTITUDE
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterAttitudeOptions(SbgProtocolHandle handle, uint32 kalmanMode)
{
	uint8 buffer[sizeof(uint8) + sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	*buffer = 0;
	*(uint32*)(buffer + sizeof(uint8)) = sbgHostToTarget32(handle->targetOutputMode,kalmanMode);

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_SET_FILTER_ATTITUDE_OPTIONS, buffer, sizeof(uint8) + sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// We should receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 *	Retreives some options regarding the Kalman Filter.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pFilterOptions			The Kalman filter options mask.<br>
 *										Possible values are:<br>
 *										- #SBG_FILTER_OPTION_FILTER_ACCELERATIONS
 *										- #SBG_FILTER_OPTION_FILTER_MAG_DISTORTIONS
 *										- #SBG_FILTER_OPTION_ESTIMATE_GYRO_BIAS
 *										- #SBG_FILTER_OPTION_ESTIMATE_ACCEL_BIAS
 *										- #SBG_FILTER_OPTION_ENABLE_ATTITUDE
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterAttitudeOptions(SbgProtocolHandle handle, uint32 *pFilterOptions)
{
	uint8 cmd;
	uint16 size;
	uint32 buffer;
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_FILTER_ATTITUDE_OPTIONS, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, (uint8*)&buffer, &size, sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_FILTER_ATTITUDE_OPTIONS) && (size == sizeof(uint32)) )
			{
				//
				// Returns, if possible, the Kalman filter options
				//
				if (pFilterOptions)
				{
					*pFilterOptions = sbgTargetToHost32(handle->targetOutputMode, buffer);
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
 *	Defines the sensors filter cut-off frequencies and the update rate for the Kalman Filter.<br>
 *	If you set a setting to 0.0, the value will remain unchanged.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	gyroAccelsSampling		Reserved for backward compatibility. Leave to 0.
 *	\param[in]	cutoffGyro				Gyroscopes low-pass filter cut-off frequency in Hz.
 *	\param[in]	cutoffAccel				Accelerometers low-pass filter cut-off frequency in Hz.
 *	\param[in]	cutoffMagneto			Magnetometers low-pass filter cut-off frequency in Hz.
 *	\param[in]	kalmanFreq				The Kalman filter refresh rate.<br>
 *										Max 100 Hz for IG-500N and 160 Hz for IG-500A.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterFrequencies(SbgProtocolHandle handle, float gyroAccelsSampling, float cutoffGyro, float cutoffAccel, float cutoffMagneto, float kalmanFreq)
{
	uint8 buffer[sizeof(uint8) + 5*sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	*buffer = 0;
	*(uint32*)(buffer + sizeof(uint8))                    = sbgHostToTargetFloat(handle->targetOutputMode, gyroAccelsSampling);
	*(uint32*)(buffer + sizeof(uint8) +   sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode, cutoffGyro);
	*(uint32*)(buffer + sizeof(uint8) + 2*sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode, cutoffAccel);
	*(uint32*)(buffer + sizeof(uint8) + 3*sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode, cutoffMagneto);
	*(uint32*)(buffer + sizeof(uint8) + 4*sizeof(uint32)) = sbgHostToTargetFloat(handle->targetOutputMode, kalmanFreq);

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_SET_FILTER_FREQUENCIES, buffer, sizeof(uint8) + 5*sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// We should receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 *	Retrives the sensors filter cut-off frequencies and the update rate for the Kalman Filter.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pGyroAccelsSampling		The accelerometers and gyroscopes sampling frequency in Hz.
 *	\param[out]	pCutoffGyro				Gyroscopes low-pass filter cut-off frequency in Hz.
 *	\param[out]	pCutoffAccel			Accelerometers low-pass filter cut-off frequency in Hz.
 *	\param[out]	pCutoffMagneto			Magnetometers low-pass filter cut-off frequency in Hz.
 *	\param[out]	pKalmanFreq				The Kalman filter refresh rate.<br>
 *										Max 100 Hz for IG-500N and 160 Hz for IG-500A.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterFrequencies(SbgProtocolHandle handle, float *pGyroAccelsSampling, float *pCutoffGyro, float *pCutoffAccel, float *pCutoffMagneto, float *pKalmanFreq)
{
	uint8 cmd;
	uint16 size;
	uint32 frequencies[5];
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_FILTER_FREQUENCIES, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, (uint8*)frequencies, &size, 5*sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_FILTER_FREQUENCIES) && (size == 5*sizeof(uint32)) )
			{
				//
				// Returns, if possible, the frequencies settings
				//
				if (pGyroAccelsSampling)
				{
					*pGyroAccelsSampling = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[0]);
				}
				if (pCutoffGyro)
				{
					*pCutoffGyro = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[1]);
				}
				if (pCutoffAccel)
				{
					*pCutoffAccel = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[2]);
				}
				if (pCutoffMagneto)
				{
					*pCutoffMagneto = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[3]);
				}
				if (pKalmanFreq)
				{
					*pKalmanFreq = sbgTargetToHostFloat(handle->targetOutputMode, frequencies[4]);
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

//----------------------------------------------------------------------//
//- GPS and altimeter relative commands                                -//   
//----------------------------------------------------------------------//

/*!
 *	Configures the advanced GPS options. (IG-30G only)
 *	\param[in]	handle				A valid sbgCom library handle.
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
SbgErrorCode sbgSetGpsOptions(SbgProtocolHandle handle, SbgGpsDynamicModel model, uint8 options)
{
	uint8 buffer[3*sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	buffer[0] = 0;
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
 *	Get the advanced GPS options. (IG-30G only)
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
	// Send the command used to get the GPS options
	//
	error = sbgProtocolSend(handle, SBG_GET_GPS_OPTIONS, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, 2 * sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_GPS_OPTIONS) && (size == 2*sizeof(uint8)) )
			{
				//
				// Returns, if possible, the GPS dynamic model
				//
				if (pModel)
				{
					*pModel = (SbgGpsDynamicModel)buffer[0];
				}

				//
				// Returns, if possible, the GPS options
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
