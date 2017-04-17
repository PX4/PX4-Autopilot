#include "protocolOutput.h"
#include "protocolOutputMode.h"
#include <string.h>

//----------------------------------------------------------------------//
//- Opertaions                                                         -//
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
SbgErrorCode sbgFillOutputFromBuffer(uint8 targetOutputMode, uint32 outputMask, void *pBuffer, uint16 bufferSize, SbgOutput *pOutput)
{
	uint16 bufferIndex = 0;
	uint8 *pBuffer8;
	uint16 *pBuffer16;
	uint32 *pBuffer32;
	fixed32 *pBufferFixed32;
	fixed64 *pBufferFixed64;
	uint32 i;

	//
	// The ouputMask define wich output are enabled and also the order
	//
	if (pOutput)
	{
		//
		// Fill the mask each time we add a value
		//
		pOutput->outputMask = 0;

		//
		// Handle quaternion
		//
		if (outputMask & SBG_OUTPUT_QUATERNION)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*4 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*4;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<4; i++)
				{
					pOutput->stateQuat[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_QUATERNION;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle euler
		//
		if (outputMask & SBG_OUTPUT_EULER)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->stateEuler[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_EULER;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle matrix
		//
		if (outputMask & SBG_OUTPUT_MATRIX)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*9 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*9;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<9; i++)
				{
					pOutput->stateMatrix[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_MATRIX;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle calibrated and unbiased gyroscopes data
		//
		if (outputMask & SBG_OUTPUT_GYROSCOPES)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->gyroscopes[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GYROSCOPES;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle calibrated accelerometers data
		//
		if (outputMask & SBG_OUTPUT_ACCELEROMETERS)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->accelerometers[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_ACCELEROMETERS;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}		

		//
		// Handle calibrated magnetometers data
		//
		if (outputMask & SBG_OUTPUT_MAGNETOMETERS)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->magnetometers[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_MAGNETOMETERS;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle temperature
		//
		if (outputMask & SBG_OUTPUT_TEMPERATURES)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*2 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*2;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<2; i++)
				{
					pOutput->temperatures[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_TEMPERATURES;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle gyroscopes raw
		//
		if (outputMask & SBG_OUTPUT_GYROSCOPES_RAW)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint16)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer16 = (uint16*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint16)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->gyroscopesRaw[i] = sbgTargetToHost16(targetOutputMode, *(pBuffer16++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GYROSCOPES_RAW;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle accelerometers raw
		//
		if (outputMask & SBG_OUTPUT_ACCELEROMETERS_RAW)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint16)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer16 = (uint16*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint16)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->accelerometersRaw[i] = sbgTargetToHost16(targetOutputMode, *(pBuffer16++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_ACCELEROMETERS_RAW;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle magnetometers raw
		//
		if (outputMask & SBG_OUTPUT_MAGNETOMETERS_RAW)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint16)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer16 = (uint16*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint16)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->magnetometersRaw[i] = sbgTargetToHost16(targetOutputMode, *(pBuffer16++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_MAGNETOMETERS_RAW;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle temperature raw
		//
		if (outputMask & SBG_OUTPUT_TEMPERATURES_RAW)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint16)*2 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer16 = (uint16*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint16)*2;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<2; i++)
				{
					pOutput->temperaturesRaw[i] = sbgTargetToHost16(targetOutputMode, *(pBuffer16++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_TEMPERATURES_RAW;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle time since reset
		//
		if (outputMask & SBG_OUTPUT_TIME_SINCE_RESET)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32);
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->timeSinceReset = sbgTargetToHost32(targetOutputMode, *(pBuffer32));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_TIME_SINCE_RESET;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle device status
		//
		if (outputMask & SBG_OUTPUT_DEVICE_STATUS)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32);
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->deviceStatus = sbgTargetToHost32(targetOutputMode, *(pBuffer32));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_DEVICE_STATUS;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle raw GPS position
		//
		if (outputMask & SBG_OUTPUT_GPS_POSITION)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->gpsLatitude	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsLongitude	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsAltitude	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GPS_POSITION;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle raw GPS velocity
		//
		if (outputMask & SBG_OUTPUT_GPS_NAVIGATION)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32)*4 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32)*4;
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->gpsVelocity[0]	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsVelocity[1]	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsVelocity[2]	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsHeading		= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GPS_NAVIGATION;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle raw GPS heading
		//
		if (outputMask & SBG_OUTPUT_GPS_ACCURACY)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32)*4 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32)*4;
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->gpsHorAccuracy		= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsVertAccuracy	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsSpeedAccuracy	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsHeadingAccuracy	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GPS_ACCURACY;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
		
		//
		// Handle raw GPS basic information
		//
		if (outputMask & SBG_OUTPUT_GPS_INFO)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32) + 2*sizeof(uint8) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32);
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->gpsTimeMs = sbgTargetToHost32(targetOutputMode, *(pBuffer32));

				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer8 = (uint8*)pBuffer+bufferIndex;
				bufferIndex += sizeof(uint8) * 2;
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->gpsFlags = *(pBuffer8++);
				pOutput->gpsNbSats = *(pBuffer8++);

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GPS_INFO;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle raw barometric altitude
		//
		if (outputMask & SBG_OUTPUT_BARO_ALTITUDE)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32);
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->baroAltitude = sbgTargetToHost32(targetOutputMode, *(pBuffer32));


				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_BARO_ALTITUDE;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle raw barometric pressure
		//
		if (outputMask & SBG_OUTPUT_BARO_PRESSURE)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32);
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->baroPressure = sbgTargetToHost32(targetOutputMode, *(pBuffer32));


				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_BARO_PRESSURE;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle 3d position
		//
		if (outputMask & SBG_OUTPUT_POSITION)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed64)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed64 = (fixed64*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed64)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->position[i] = sbgTargetToHostDouble(targetOutputMode, *(pBufferFixed64++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_POSITION;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle 3d velocity
		//
		if (outputMask & SBG_OUTPUT_VELOCITY)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->velocity[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_VELOCITY;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle attitude accuracy
		//
		if (outputMask & SBG_OUTPUT_ATTITUDE_ACCURACY)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32);
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->attitudeAccuracy = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_ATTITUDE_ACCURACY;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle navigation accuracy
		//
		if (outputMask & SBG_OUTPUT_NAV_ACCURACY)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*2 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*2;
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->positionAccuracy = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				pOutput->velocityAccuracy = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_NAV_ACCURACY;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle gyroscopes temperatures
		//
		if (outputMask & SBG_OUTPUT_GYRO_TEMPERATURES)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(fixed32)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->gyroTemperatures[i] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GYRO_TEMPERATURES;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle gyroscopes raw temperatures
		//
		if (outputMask & SBG_OUTPUT_GYRO_TEMPERATURES_RAW)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint16)*3 <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer16 = (uint16*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint16)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				for (i=0; i<3; i++)
				{
					pOutput->gyroTemperaturesRaw[i] = sbgTargetToHost16(targetOutputMode, *(pBuffer16++));
				}

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GYRO_TEMPERATURES_RAW;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle UTC Time reference output
		//
		if (outputMask & SBG_OUTPUT_UTC_TIME_REFERENCE)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if (bufferIndex + sizeof(uint32) + 6*sizeof(uint8) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer8 = (uint8*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32) + 6*sizeof(uint8);
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->utcYear	= *(pBuffer8++);
				pOutput->utcMonth	= *(pBuffer8++);
				pOutput->utcDay		= *(pBuffer8++);
				pOutput->utcHour	= *(pBuffer8++);
				pOutput->utcMin		= *(pBuffer8++);
				pOutput->utcSec		= *(pBuffer8++);

				// Here we copy nano seconds in uint32 format
				// and finally the delay of calculation
				pBuffer32 = (uint32*)pBuffer8;
				pOutput->utcNano	= sbgTargetToHost32(targetOutputMode, *pBuffer32);

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_UTC_TIME_REFERENCE;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle Magnetometers Calibration data
		//
		if (outputMask & SBG_OUTPUT_MAG_CALIB_DATA)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if ((bufferIndex + 12*sizeof(uint8)) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer8 = (uint8*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += 12*sizeof(uint8);
				
				//
				// Copy and format the data to the destination buffer
				//
				memcpy(pOutput->magCalibData, pBuffer8, 12*sizeof(uint8));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_MAG_CALIB_DATA;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle GPS True heading output
		//
		if (outputMask & SBG_OUTPUT_GPS_TRUE_HEADING)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if ((bufferIndex + 2*sizeof(uint32)) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBuffer32 = (uint32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(uint32)*2;
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->gpsTrueHeading			= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));
				pOutput->gpsTrueHeadingAccuracy	= sbgTargetToHost32(targetOutputMode, *(pBuffer32++));


				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_GPS_TRUE_HEADING;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle Odometer velocity output
		//
		if (outputMask & SBG_OUTPUT_ODO_VELOCITIES)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if ((bufferIndex + 2*sizeof(uint32)) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*2;
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->odoRawVelocity[0] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				pOutput->odoRawVelocity[1] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));


				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_ODO_VELOCITIES;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle delta angle output
		//
		if (outputMask & SBG_OUTPUT_DELTA_ANGLES)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if ((bufferIndex + 3*sizeof(uint32)) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32)*3;
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->deltaAngles[0] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				pOutput->deltaAngles[1] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));
				pOutput->deltaAngles[2] = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));


				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_DELTA_ANGLES;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}

		//
		// Handle heave output
		//
		if (outputMask & SBG_OUTPUT_HEAVE)
		{
			//
			// Check if we can store the data in the destination buffer
			//
			if ((bufferIndex + sizeof(uint32)) <= bufferSize)
			{
				//
				// Gets a pointer to store the data and increment the destination size
				//
				pBufferFixed32 = (fixed32*)((uint8*)pBuffer+bufferIndex);
				bufferIndex += sizeof(fixed32);
				
				//
				// Copy and format the data to the destination buffer
				//
				pOutput->heave = sbgTargetToHostFloat(targetOutputMode, *(pBufferFixed32++));

				//
				// Mark this data field as valid
				//
				pOutput->outputMask |= SBG_OUTPUT_HEAVE;
			}
			else
			{
				return SBG_BUFFER_OVERFLOW;
			}
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}

	return SBG_NO_ERROR;
}

/*!
 *	Calculate and returns the size of a buffer containing the data enabled in oututMask.
 *	\param[in]	targetOutputMode		The output mode used by the target.<br>
 *										Defines if the data are stored in Big/Little endian <br>
 *										and if real values are stored in float or in fixed format.
 *	\param[in]	outputMask				Mask combinaison that defines which outputs are contained in the pBuffer.
 *	\return								The size of the buffer needed to hold enabled outputs.
 */
uint16 sbgCalculateOutputBufferSize(uint8 targetOutputMode, uint32 outputMask)
{
	uint16 bufferSize = 0;

	//
	// Just used to avoid compilation warning
	//
	(void)targetOutputMode;

	//
	// According to each enabled mask, compute the output buffer size
	//
	if (outputMask & SBG_OUTPUT_QUATERNION)
	{
		bufferSize += sizeof(fixed32)*4;
	}

	if (outputMask & SBG_OUTPUT_EULER)
	{
		bufferSize += sizeof(fixed32)*3;
	}

	if (outputMask & SBG_OUTPUT_MATRIX)
	{
		bufferSize += sizeof(fixed32)*9;
	}

	if (outputMask & SBG_OUTPUT_GYROSCOPES)
	{
		bufferSize += sizeof(fixed32)*3;
	}

	if (outputMask & SBG_OUTPUT_ACCELEROMETERS)
	{
		bufferSize += sizeof(fixed32)*3;
	}

	if (outputMask & SBG_OUTPUT_MAGNETOMETERS)
	{
		bufferSize += sizeof(fixed32)*3;
	}

	if (outputMask & SBG_OUTPUT_TEMPERATURES)
	{
		bufferSize += sizeof(fixed32)*2;
	}

	if (outputMask & SBG_OUTPUT_GYROSCOPES_RAW)
	{
		bufferSize += sizeof(uint16)*3;
	}

	if (outputMask & SBG_OUTPUT_ACCELEROMETERS_RAW)
	{
		bufferSize += sizeof(uint16)*3;
	}

	if (outputMask & SBG_OUTPUT_MAGNETOMETERS_RAW)
	{
		bufferSize += sizeof(uint16)*3;
	}

	if (outputMask & SBG_OUTPUT_TEMPERATURES_RAW)
	{
		bufferSize += sizeof(uint16);
	}

	if (outputMask & SBG_OUTPUT_TIME_SINCE_RESET)
	{
		bufferSize += sizeof(uint32);
	}

	if (outputMask & SBG_OUTPUT_DEVICE_STATUS)
	{
		bufferSize += sizeof(uint32);
	}

	if (outputMask & SBG_OUTPUT_GPS_POSITION)
	{
		bufferSize += sizeof(uint32)*3;
	}

	if (outputMask & SBG_OUTPUT_GPS_NAVIGATION)
	{
		bufferSize += sizeof(uint32)*4;
	}

	if (outputMask & SBG_OUTPUT_GPS_ACCURACY)
	{
		bufferSize += sizeof(uint32)*4;
	}
	if (outputMask & SBG_OUTPUT_GPS_INFO)
	{
		bufferSize += sizeof(uint32) + 2*sizeof(uint8);
	}

	if (outputMask & SBG_OUTPUT_BARO_ALTITUDE)
	{
		bufferSize += sizeof(uint32);
	}
	if (outputMask & SBG_OUTPUT_BARO_PRESSURE)
	{
		bufferSize += sizeof(uint32);
	}

	if (outputMask & SBG_OUTPUT_POSITION)
	{
		bufferSize += sizeof(fixed64)*3;
	}

	if (outputMask & SBG_OUTPUT_VELOCITY)
	{
		bufferSize += sizeof(fixed32)*3;
	}

	if (outputMask & SBG_OUTPUT_ATTITUDE_ACCURACY)
	{
		bufferSize += sizeof(fixed32);
	}

	if (outputMask & SBG_OUTPUT_NAV_ACCURACY)
	{
		bufferSize += sizeof(fixed32)*2;
	}

	if (outputMask & SBG_OUTPUT_GYRO_TEMPERATURES)
	{
		bufferSize += sizeof(fixed32)*3;
	}

	if (outputMask & SBG_OUTPUT_GYRO_TEMPERATURES_RAW)
	{
		bufferSize += sizeof(uint16)*3;
	}

	if (outputMask & SBG_OUTPUT_UTC_TIME_REFERENCE)
	{
		bufferSize += sizeof(uint32) + 6*sizeof(uint8);
	}

	if (outputMask & SBG_OUTPUT_MAG_CALIB_DATA)
	{
		bufferSize += 12 * sizeof(uint8);
	}

	if (outputMask & SBG_OUTPUT_GPS_TRUE_HEADING)
	{
		bufferSize += sizeof(uint32)*2;
	}

	if (outputMask & SBG_OUTPUT_ODO_VELOCITIES)
	{
		bufferSize += sizeof(fixed32)*2;
	}
	
	if (outputMask & SBG_OUTPUT_DELTA_ANGLES)
	{
		bufferSize += sizeof(fixed32)*3;
	}

	if (outputMask & SBG_OUTPUT_HEAVE)
	{
		bufferSize += sizeof(fixed32);
	}

	return bufferSize;
}
