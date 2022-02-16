/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Provides status codes for the AFBR-S50 API.
 *
 * @copyright
 *
 * Copyright (c) 2021, Broadcom Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef ARGUS_STATUS_H
#define ARGUS_STATUS_H

#include <stdint.h>

/*!***************************************************************************
 * @defgroup    status Status Codes
 * @brief       Status and Error Code Definitions
 * @details     Defines status and error codes for function return values.
 *              Basic status number structure:
 *              - 0 is OK or no error.
 *              - negative values determine errors.
 *              - positive values determine warnings or status information.
 *              .
 * @addtogroup  status
 * @{
 *****************************************************************************/

/*!***************************************************************************
 * @brief  	Type used for all status and error return values.
 * @details	Basic status number structure:
 *           - 0 is OK or no error.
 *           - negative values determine errors.
 *           - positive values determine warnings or status information.
 *           .
 *****************************************************************************/
typedef int32_t status_t;

/*! AFBR-S50 API status and error return codes. */
enum Status {
	/**********************************************************************************************
	 ********** Generic Status ********************************************************************
	 *********************************************************************************************/

	/*! 0: Status for success/no error. */
	STATUS_OK = 0,

	/*! 0: Status for device/module/hardware idle. Implies #STATUS_OK. */
	STATUS_IDLE = 0,

	/*! 1: Status to be ignored. */
	STATUS_IGNORE = 1,

	/*! 2: Status for device/module/hardware busy. */
	STATUS_BUSY = 2,

	/*! 3: Status for device/module/hardware is currently initializing. */
	STATUS_INITIALIZING = 3,

	/*! -1: Error for generic fail/error. */
	ERROR_FAIL = -1,

	/*! -2: Error for process aborted by user/external. */
	ERROR_ABORTED = -2,

	/*! -3: Error for invalid read only operations. */
	ERROR_READ_ONLY = -3,

	/*! -4: Error for out of range parameters. */
	ERROR_OUT_OF_RANGE = -4,

	/*! -5: Error for invalid argument passed to an function. */
	ERROR_INVALID_ARGUMENT = -5,

	/*! -6: Error for timeout occurred. */
	ERROR_TIMEOUT = -6,

	/*! -7: Error for not initialized modules. */
	ERROR_NOT_INITIALIZED = -7,

	/*! -8: Error for not supported. */
	ERROR_NOT_SUPPORTED = -8,

	/*! -9: Error for yet not implemented functions. */
	ERROR_NOT_IMPLEMENTED = -9,


	/**********************************************************************************************
	 ********** S2PI Layer Status *****************************************************************
	 *********************************************************************************************/

	/*! 51: SPI is disabled and pins are used in GPIO mode. */
	STATUS_S2PI_GPIO_MODE = 51,

	/*! -51: Error occurred on the Rx line. */
	ERROR_S2PI_RX_ERROR = -51,

	/*! -52: Error occurred on the Tx line. */
	ERROR_S2PI_TX_ERROR = -52,

	/*! -53: Called a function at a wrong driver state. */
	ERROR_S2PI_INVALID_STATE = -53,

	/*! -54: The specified baud rate is not valid. */
	ERROR_S2PI_INVALID_BAUDRATE = -54,

	/*! -55: The specified slave identifier is not valid. */
	ERROR_S2PI_INVALID_SLAVE = -55,


	/**********************************************************************************************
	 ********** NVM / Flash  Layer Status *********************************************************
	 *********************************************************************************************/

	/*! -98: Flash Error: The version of the settings in the flash memory is not compatible. */
	ERROR_NVM_INVALID_FILE_VERSION = -98,

	/*! -99: Flash Error: The memory is out of range. */
	ERROR_NVM_OUT_OF_RANGE = -99,


	/**********************************************************************************************
	 ********** AFBR-S50 Specific Status **********************************************************
	 *********************************************************************************************/

	/*! 104: AFBR-S50 Status: All (internal) raw data buffers are currently in use.
	 *  The measurement was not executed due to lack of available raw data buffers.
	 *  Please call #Argus_EvaluateData to free the buffers. */
	STATUS_ARGUS_BUFFER_BUSY = 104,

	/*! 105: AFBR-S50 Status: The measurement was not executed/started due to output power
	 *  limitations. */
	STATUS_ARGUS_POWERLIMIT = 105,

	/*! 106: AFBR-S50 Status: The PLL was not locked when the measurement was
	 *  started. The measurement frequency and this the range might be off. */
	STATUS_ARGUS_PLL_NOT_LOCKED = 106,

	/*! 108: AFBR-S50 Status: No object was detected within the field-of-view
	 *  and measurement range of the device. */
	STATUS_ARGUS_NO_OBJECT = 108,

	/*! 109: AFBR-S50 Status: The readout algorithm for the EEPROM has detected a bit
	 *  error which has been corrected. However, if more than a single bit error
	 *  has occurred, the corrected value is invalid! This cannot be distinguished
	 *  from the valid case. Thus, if the error starts to occur, the sensor
	 *  should be replaced soon! */
	STATUS_ARGUS_EEPROM_BIT_ERROR = 109,

	/*! 110: AFBR-S50 Status: Inconsistent EEPROM readout data. No calibration
	 *  trimming values are applied. The calibration remains invalid. */
	STATUS_ARGUS_INVALID_EEPROM = 110,

	/*! -101: AFBR-S50 Error: No device connected. Initial SPI tests failed. */
	ERROR_ARGUS_NOT_CONNECTED = -101,

	/*! -102: AFBR-S50 Error: Inconsistent configuration parameters. */
	ERROR_ARGUS_INVALID_CFG = -102,


	/*! -105: AFBR-S50 Error: Invalid measurement mode configuration parameter. */
	ERROR_ARGUS_INVALID_MODE = -105,

	/*! -107: AFBR-S50 Error: The APD bias voltage is reinitializing due to a dropout.
	 *  The current measurement data set is invalid! */
	ERROR_ARGUS_BIAS_VOLTAGE_REINIT = -107,


	/*! -109: AFBR-S50 Error: The EEPROM readout has failed. The failure is detected
	 *  by three distinct read attempts, each resulting in invalid data.
	 *  Note: this state differs from that #STATUS_ARGUS_EEPROM_BIT_ERROR
	 *  such that it is usually temporarily and due to harsh ambient conditions. */
	ERROR_ARGUS_EEPROM_FAILURE = -109,

	/*! -110: AFBR-S50 Error: The measurement signals of all active pixels are invalid
	 *  and thus the 1D range is also invalid and stalled.
	 *  This means the range value is not updated and kept at the previous valid value. */
	ERROR_ARGUS_STALLED = -110,

	/*! -111: AFBR-S50 Error: The background light is too bright. */
	ERROR_ARGUS_BGL_EXCEEDANCE = -111,

	/*! -112: AFBR-S50 Error: The crosstalk vector amplitude is too high. */
	ERROR_ARGUS_XTALK_AMPLITUDE_EXCEEDANCE = -112,

	/*! -113: AFBR-S50 Error: Laser malfunction! Laser Safety may not be given! */
	ERROR_ARGUS_LASER_FAILURE = -113,

	/*! -114: AFBR-S50 Error: Register data integrity is lost (e.g. due to unexpected
	 *  power-on-reset cycle or invalid write cycle of SPI. System tries to
	 *  reset the values. */
	ERROR_ARGUS_DATA_INTEGRITY_LOST = -114,

	/*! -115: AFBR-S50 Error: The range offsets calibration failed! */
	ERROR_ARGUS_RANGE_OFFSET_CALIBRATION_FAILED = -115,

	/*! -191: AFBR-S50 Error: The device is currently busy and cannot execute the
	 *  requested command. */
	ERROR_ARGUS_BUSY = -191,


	/*! -199: AFBR-S50 Error: Unknown module number. */
	ERROR_ARGUS_UNKNOWN_MODULE = -199,

	/*! -198: AFBR-S50 Error: Unknown chip version number. */
	ERROR_ARGUS_UNKNOWN_CHIP = -198,

	/*! -197: AFBR-S50 Error: Unknown laser type number. */
	ERROR_ARGUS_UNKNOWN_LASER = -197,



	/*! 193: AFBR-S50 Status (internal): The device is currently busy with updating the
	 *  configuration (i.e. with writing register values). */
	STATUS_ARGUS_BUSY_CFG_UPDATE = 193,

	/*! 194: AFBR-S50 Status (internal): The device is currently busy with updating the
	 *  calibration data (i.e. writing to register values). */
	STATUS_ARGUS_BUSY_CAL_UPDATE = 194,

	/*! 195: AFBR-S50 Status (internal): The device is currently executing a calibration
	 * 	sequence. */
	STATUS_ARGUS_BUSY_CAL_SEQ = 195,

	/*! 196: AFBR-S50 Status (internal): The device is currently executing a measurement
	 *  cycle. */
	STATUS_ARGUS_BUSY_MEAS = 196,


	/*! 100: AFBR-S50 Status (internal): The ASIC is initializing a new measurement, i.e.
	 *  a register value is written that starts an integration cycle on the ASIC. */
	STATUS_ARGUS_STARTING = 100,

	/*! 103: AFBR-S50 Status (internal): The ASIC is performing an integration cycle. */
	STATUS_ARGUS_ACTIVE = 103,



};

/*! @} */
#endif /* ARGUS_STATUS_H */
