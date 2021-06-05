/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides generic functionality belonging to all
 * 				devices from the AFBR-S50 product family.
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

#ifndef ARGUS_API_H
#define ARGUS_API_H

#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup	argusapi AFBR-S50 API
 *
 * @brief		The main module of the API from the AFBR-S50 SDK.
 *
 * @details		General API for the AFBR-S50 time-of-flight sensor device family.
 *
 * @addtogroup 	argusapi
 * @{
 *****************************************************************************/

#include "argus_def.h"
#include "argus_res.h"
#include "argus_pba.h"
#include "argus_dfm.h"
#include "argus_snm.h"
#include "argus_xtalk.h"

/*! The data structure for the API representing a AFBR-S50 device instance. */
typedef void argus_hnd_t;

/*! The S2PI slave identifier. */
typedef int32_t s2pi_slave_t;

/*!***************************************************************************
 * @brief 	Initializes the API modules and the device with default parameters.
 *
 * @details The function that needs to be called once after power up to
 * 			initialize the modules state (i.e. the corresponding handle) and the
 * 			dedicated Time-of-Flight device. In order to obtain a handle,
 * 			reference the #Argus_CreateHandle method.
 *
 * 			Prior to calling the function, the required peripherals (i.e. S2PI,
 * 			GPIO w/ IRQ and Timers) must be initialized and ready to use.
 *
 * 			The function executes the following tasks:
 * 			- Initialization of the internal state represented by the handle
 * 			  object.
 * 			- Setup the device such that an safe configuration is present in
 * 			  the registers.
 * 			- Initialize sub modules such as calibration or measurement modules.
 * 			.
 *
 * 			The modules configuration is initialized with reasonable default values.
 *
 * @param	hnd The API handle; contains all internal states and data.
 *
 * @param	spi_slave The SPI hardware slave, i.e. the specified CS and IRQ
 * 						lines. This is actually just a number that is passed
 * 						to the SPI interface to distinct for multiple SPI slave
 * 						devices. Note that the slave must be not equal to 0,
 * 						since is reserved for error handling.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_Init(argus_hnd_t *hnd, s2pi_slave_t spi_slave);

/*!***************************************************************************
 * @brief 	Reinitializes the API modules and the device with default parameters.
 *
 * @details The function reinitializes the device with default configuration.
 * 			Can be used as reset sequence for the device. See #Argus_Init for
 * 			more information on the initialization.
 *
 * 			Note that the #Argus_Init function must be called first! Otherwise,
 * 			the function will return an error if it is called for an yet
 * 			uninitialized device/handle.
 *
 * @param	hnd The API handle; contains all internal states and data.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_Reinit(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief 	Deinitializes the API modules and the device.
 *
 * @details The function deinitializes the device and clear all internal states.
 * 			Can be used to cleanup before releaseing the memory. The device
 * 			can not be used any more and must be initialized again prior to next
 * 			usage.
 *
 * 			Note that the #Argus_Init function must be called first! Otherwise,
 * 			the function will return an error if it is called for an yet
 * 			uninitialized device/handle.
 *
 * @param	hnd The API handle; contains all internal states and data.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_Deinit(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief 	Creates a new device data handle object to store all internal states.
 *
 * @details The function must be called to obtain a new device handle object.
 * 			The handle is basically an abstract object in memory that contains
 * 			all the internal states and settings of the API module. The handle
 * 			is passed to all the API methods in order to address the specified
 * 			device. This allows to use the API with more than a single measurement
 * 			device.
 *
 * 			The handler is created by calling the memory allocation method from
 * 			the standard library: @code void * malloc(size_t size) @endcode
 * 			In order to implement an individual memory allocation method,
 * 			define and implement the following weakly binded method and return
 * 			a pointer to the newly allocated memory. *
 * 			@code void * Argus_Malloc (size_t size) @endcode
 * 			Also see the #Argus_DestroyHandle method for the corresponding
 * 			deallocation of the allocated memory.
 *
 * @return 	Returns a pointer to the newly allocated device handler object.
 * 			Returns a null pointer if the allocation failed!
 *****************************************************************************/
argus_hnd_t *Argus_CreateHandle(void);

/*!***************************************************************************
 * @brief 	Destroys a given device data handle object.
 *
 * @details The function can be called to free the previously created device
 * 			data handle object in order to save memory when the device is not
 * 			used any more.
 *
 * 			Please refer to the #Argus_CreateHandle method for the corresponding
 * 			allocation of the memory.
 *
 * 			The handler is destroyed by freeing the corresponding memory with the
 * 			method from the standard library, @code void free(void * ptr) @endcode.
 * 			In order to implement an individual memory deallocation method, define
 * 			and implement the following weakly binded method and free the memory
 * 			object passed to the method by a pointer.
 *
 * 			@code void Argus_Free (void * ptr) @endcode
 *
 * @param	hnd The device handle object to be deallocated.
 *****************************************************************************/
void Argus_DestroyHandle(argus_hnd_t *hnd);

/*!**************************************************************************
 * Generic API
 ****************************************************************************/

/*!***************************************************************************
 * @brief	Gets the version number of the current API library.
 *
 * @details	The version is compiled of a major (a), minor (b) and bugfix (c)
 * 			number: a.b.c.
 *
 * 			The values are encoded into a 32-bit value:
 *
 * 			 - [ 31 .. 24 ] - Major Version Number
 * 			 - [ 23 .. 16 ] - Minor Version Number
 * 			 - [ 15 ..  0 ] - Bugfix Version Number
 * 			 .
 *
 * 			To obtain the parts from the returned uin32_t value:
 *
 *			@code
 *			uint32_t value = Argus_GetAPIVersion();
 *			uint8_t a = (value >> 24) & 0xFFU;
 *			uint8_t b = (value >> 16) & 0xFFU;
 *			uint8_t c = value & 0xFFFFU;
 * 			@endcode
 *
 * @return 	Returns the current version number.
 *****************************************************************************/
uint32_t Argus_GetAPIVersion(void);

/*!***************************************************************************
 * @brief	Gets the build number of the current API library.
 *
 * @return 	Returns the current build number as a C-string.
 *****************************************************************************/
char const *Argus_GetBuildNumber(void);

/*!***************************************************************************
 * @brief	Gets the version/variant of the module.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return 	Returns the current module number.
 *****************************************************************************/
argus_module_version_t Argus_GetModuleVersion(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief	Gets the version number of the chip.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return 	Returns the current version number.
 *****************************************************************************/
argus_chip_version_t Argus_GetChipVersion(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief	Gets the type number of the device laser.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return 	Returns the current device laser type number.
 *****************************************************************************/
argus_laser_type_t Argus_GetLaserType(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief	Gets the unique identification number of the chip.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return 	Returns the unique identification number.
 *****************************************************************************/
uint32_t Argus_GetChipID(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief	Gets the SPI hardware slave identifier.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return	The SPI hardware slave identifier.
 *****************************************************************************/
s2pi_slave_t Argus_GetSPISlave(argus_hnd_t *hnd);

/*! @} */

/*!**************************************************************************
 * Measurement/Device Operation
 ****************************************************************************
 * @addtogroup 	argusmeas
 * @{
 ****************************************************************************/

/*!***************************************************************************
 * @brief	Starts the timer based measurement cycle asynchronously.
 *
 * @details This function starts a timer based measurement cycle asynchronously.
 * 			in the background. A periodic timer interrupt triggers the measurement
 * 			frames on the ASIC and the data readout afterwards. When the frame is
 * 			finished, a callback (which is passed as a parameter to the function)
 * 			is invoked in order to inform the main thread to call the \link
 * 			#Argus_EvaluateData data evaluation method\endlink. This call is
 * 			mandatory to release the data buffer for the next measurement cycle
 * 			and it must not be invoked from the callback since it is within an
 * 			interrupt service routine. Rather a flag should inform the main thread
 * 			to invoke the evaluation as soon as possible in order to not introduce
 * 			any unwanted delays to the next measurement frame.
 *			The next measurement frame will be started as soon as the pre-
 *			conditions are meet. These are:
 *			 1. timer flag set (i.e. a certain time has passed since the last
 *			 	measurement in order to fulfill eye-safety),
 *			 2. device idle (i.e. no measurement currently ongoing) and
 *			 3. data buffer ready (i.e. the previous data has been evaluated).
 *			Usually, the device idle and data buffer ready conditions are met
 *			before the timer tick occurs and thus the timer dictates the frame
 *			rate.
 *
 * 			The callback function pointer will be invoked when the measurement
 * 			frame has finished successfully or whenever an error, that cannot
 * 			be handled internally, occurs.
 *
 * 			The periodic timer interrupts are used to check the measurement status
 *			for timeouts. An error is invoked when a measurement cycle have not
 *			finished within the specified time.
 *
 *			Use #Argus_StopMeasurementTimer to stop the measurements.
 *
 * @note	In order to use this function, the periodic interrupt timer module
 * 			(see @ref argus_timer) must be implemented!
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	cb  Callback function that will be invoked when the measurement
 * 				  is completed. Its parameters are the \link #status_t status
 * 				  \endlink and a pointer to the \link #argus_results_t results
 * 				  \endlink structure. If an error occurred, the status differs
 * 				  from #STATUS_OK and the second parameter is null.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_StartMeasurementTimer(argus_hnd_t *hnd, argus_callback_t cb);

/*!***************************************************************************
 * @brief	Stops the timer based measurement cycle.
 *
 * @details This function stops the ongoing timer based measurement cycles
 * 			that have been started using the #Argus_StartMeasurementTimer
 * 			function.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_StopMeasurementTimer(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief	Triggers a single measurement frame asynchronously.
 *
 * @details This function immediately triggers a single measurement frame
 * 			asynchronously if all the pre-conditions are met. Otherwise it returns
 * 			with a corresponding status.
 *			When the frame is finished, a callback (which is passed as a parameter
 *			to the function) is invoked in order to inform the main thread to
 *			call the \link #Argus_EvaluateData data evaluation method\endlink.
 *			This call is mandatory to release the data buffer for the next
 *			measurement and it must not be invoked from the callback since it is
 *			within an interrupt service routine. Rather a flag should inform
 *			the main thread to invoke the evaluation.
 *			The pre-conditions for starting a measurement frame are:
 *			 1. timer flag set (i.e. a certain time has passed since the last
 *			 	measurement in order to fulfill eye-safety),
 *			 2. device idle (i.e. no measurement currently ongoing) and
 *			 3. data buffer ready (i.e. the previous data has been evaluated).
 *
 * 			The callback function pointer will be invoked when the measurement
 * 			frame has finished successfully or whenever an error, that cannot
 * 			be handled internally, occurs.
 *
 *			The successful finishing of the measurement frame is not checked
 *			for timeouts! Instead, the user can call the #Argus_GetStatus()
 *			function on a regular function to do so.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	cb	Callback function that will be invoked when the measurement
 * 				is completed. Its parameters are the \link #status_t status
 * 				\endlink and a pointer to the \link #argus_results_t results
 * 				\endlink structure. If an error occurred, the status differs
 * 				from #STATUS_OK and the second parameter is null.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_TriggerMeasurement(argus_hnd_t *hnd, argus_callback_t cb);

/*!***************************************************************************
 * @brief 	Stops the currently ongoing measurements and SPI activity immediately.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_Abort(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief 	Checks the state of the device/driver.
 *
 * @details	Returns the current module state:
 *
 *			Status:
 *			- Idle/OK: Device and SPI interface are idle (== #STATUS_IDLE).
 * 			- Busy: Device or SPI interface are busy (== #STATUS_BUSY).
 * 			- Initializing: The modules and devices are currently initializing
 * 			                (== #STATUS_INITIALIZING).
 * 			.
 *
 * 			Error:
 * 			- Not Initialized: The modules (or any submodule) has not been
 * 							   initialized yet (== #ERROR_NOT_INITIALIZED).
 * 			- Not Connected: No device has been connected (or connection errors
 * 							 have occured) (== #ERROR_ARGUS_NOT_CONNECTED).
 * 			- Timeout: A previous frame measurement has not finished within a
 * 					   specified time (== #ERROR_TIMEOUT).
 *			.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetStatus(argus_hnd_t *hnd);

/*!*****************************************************************************
 * @brief 	Tests the connection to the device by sending a ping message.
 *
 * @details	A ping is transfered to the device in order to check the device and
 * 			SPI connection status. Returns #STATUS_OK on success and
 * 			#ERROR_ARGUS_NOT_CONNECTED elsewise.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 ******************************************************************************/
status_t Argus_Ping(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief 	Evaluate useful information from the raw measurement data.
 *
 * @details	This function is called with a pointer to the raw results obtained
 * 			from the measurement cycle. It evaluates this data and creates
 * 			useful information from it. Furthermore, calibration is applied to
 * 			the data. Finally, the results are used in order to adapt the device
 * 			configuration to the ambient conditions in order to achieve optimal
 * 			device performance.\n
 * 			Therefore, it consists of the following sub-functions:
 * 			- Apply pre-calibration: Applies calibration steps before evaluating
 * 			  the data, i.e. calculations that are to the integration results
 * 			  directly.
 * 			- Evaluate data: Calculates measurement parameters such as range,
 * 			  amplitude or ambient light intensity, depending on the configurations.
 * 			- Apply post-calibration: Applies calibrations after evaluation of
 * 			  measurement data, i.e. calibrations applied to the calculated
 * 			  values such as range.
 * 			- Dynamic Configuration Adaption: checks if the configuration needs
 *			  to be adjusted before the next measurement cycle in order to
 *			  achieve optimum performance. Note that the configuration might not
 *			  applied directly but before the next measurement starts. This is
 *			  due to the fact that the device could be busy measuring already
 *			  the next frame and thus no SPI activity is allowed.
 *			.
 *			However, if the device is idle, the configuration will be written
 *			immediately.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	res A pointer to the results structure that will be populated
 * 				  with evaluated data.
 * @param	raw The pointer to the raw data that has been obtained by the
 * 				  measurement finished callback.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_EvaluateData(argus_hnd_t *hnd, argus_results_t *res, void *raw);

/*!***************************************************************************
 * @brief	Executes a crosstalk calibration measurement.
 *
 * @details This function immediately triggers a crosstalk vector calibration
 * 			measurement sequence. The ordinary measurement activity is suspended
 * 			while the calibration is ongoing.
 *
 * 			In order to perform a crosstalk calibration, the reflection of the
 * 			transmitted signal must be kept from the receiver side, by either
 * 			covering the TX completely (or RX respectively) or by setting up
 * 			an absorbing target at far distance.
 *
 * 			After calibration has finished successfully, the obtained data is
 * 			applied immediately and can be read from the API using the
 * 			#Argus_GetCalibrationCrosstalkVectorTable function.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ExecuteXtalkCalibrationSequence(argus_hnd_t *hnd, argus_mode_t mode);


/*!***************************************************************************
 * @brief	Executes a relative range offset calibration measurement.
 *
 * @details This function immediately triggers a relative range offset calibration
 * 			measurement sequence. The ordinary measurement activity is suspended
 * 			while the calibration is ongoing.
 *
 * 			In order to perform a relative range offset calibration, a flat
 * 			calibration target must be setup perpendicular to the sensors
 * 			field-of-view.
 *
 * 			\code
 *                           AFBR-S50 ToF Sensor
 * 			          #|
 * 			          #|                                         |
 * 			          #|-----+                                   |
 * 			          #| Rx  |                                   |
 * 			Reference #|----++                                   | Calibration
 * 			    Plane #| Tx |                                    | Target
 * 			          #|----+                                    |
 * 			          #|                                         |
 * 			          #| <------- targetRange -----------------> |
 * 			\endcode
 *
 *			There are two options to run the offset calibration: relative and
 *			absolute.
 *			- Relative (#Argus_ExecuteRelativeRangeOffsetCalibrationSequence):
 *			  when the absolute distance is not essential or the distance to
 *			  the calibration target is not known, the relative method can be
 *			  used to compensate the relative pixel range offset w.r.t. the
 *			  average range. The absolute or global range offset is not changed.
 *			- Absolute (#Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence):
 *			  when the absolute distance is essential and the distance to the
 *			  calibration target is known, the absolute method can be used to
 *			  calibrate the absolute measured distance. Additionally, the
 *			  relative pixel offset w.r.t. the average range is also compensated.
 *			.
 *
 * 			After calibration has finished successfully, the obtained data is
 * 			applied immediately and can be read from the API using the
 * 			#Argus_GetCalibrationPixelRangeOffsets or
 * 			#Argus_GetCalibrationGlobalRangeOffset function.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ExecuteRelativeRangeOffsetCalibrationSequence(argus_hnd_t *hnd,
		argus_mode_t mode);

/*!***************************************************************************
 * @brief	Executes an absolute range offset calibration measurement.
 *
 * @details This function immediately triggers an absolute range offset calibration
 * 			measurement sequence. The ordinary measurement activity is suspended
 * 			while the calibration is ongoing.
 *
 * 			In order to perform a relative range offset calibration, a flat
 * 			calibration target must be setup perpendicular to the sensors
 * 			field-of-view.
 *
 * 			\code
 *                           AFBR-S50 ToF Sensor
 * 			          #|
 * 			          #|                                         |
 * 			          #|-----+                                   |
 * 			          #| Rx  |                                   |
 * 			Reference #|----++                                   | Calibration
 * 			    Plane #| Tx |                                    | Target
 * 			          #|----+                                    |
 * 			          #|                                         |
 * 			          #| <------- targetRange -----------------> |
 * 			\endcode
 *
 *			There are two options to run the offset calibration: relative and
 *			absolute.
 *			- Relative (#Argus_ExecuteRelativeRangeOffsetCalibrationSequence):
 *			  when the absolute distance is not essential or the distance to
 *			  the calibration target is not known, the relative method can be
 *			  used to compensate the relative pixel range offset w.r.t. the
 *			  average range. The absolute or global range offset is not changed.
 *			- Absolute (#Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence):
 *			  when the absolute distance is essential and the distance to the
 *			  calibration target is known, the absolute method can be used to
 *			  calibrate the absolute measured distance. Additionally, the
 *			  relative pixel offset w.r.t. the average range is also compensated.
 *			.
 *
 * 			After calibration has finished successfully, the obtained data is
 * 			applied immediately and can be read from the API using the
 * 			#Argus_GetCalibrationPixelRangeOffsets or
 * 			#Argus_GetCalibrationGlobalRangeOffset function.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	targetRange The absolute range between the reference plane and the
 * 						calibration target in meter an Q9.22 format.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence(argus_hnd_t *hnd,
		argus_mode_t mode,
		q9_22_t targetRange);

/*! @} */

/*!**************************************************************************
 * Configuration API
 ****************************************************************************
 * @addtogroup 	arguscfg
 * @{
 ****************************************************************************/

/*!***************************************************************************
 * @brief 	Sets the measurement mode to a specified device.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The new measurement mode.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationMeasurementMode(argus_hnd_t *hnd,
		argus_mode_t value);

/*!***************************************************************************
 * @brief 	Gets the measurement mode from a specified device.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The current measurement mode.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationMeasurementMode(argus_hnd_t *hnd,
		argus_mode_t *value);

/*!***************************************************************************
 * @brief 	Sets the frame time to a specified device.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The measurement frame time in microseconds.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationFrameTime(argus_hnd_t *hnd, uint32_t value);

/*!***************************************************************************
 * @brief	Gets the frame time from a specified device.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The current frame time in microseconds.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationFrameTime(argus_hnd_t *hnd, uint32_t *value);

/*!***************************************************************************
 * @brief 	Sets the smart power save enabled flag to a specified device.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new smart power save enabled flag.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationSmartPowerSaveEnabled(argus_hnd_t *hnd,
		argus_mode_t mode,
		bool value);

/*!***************************************************************************
 * @brief 	Gets the smart power save enabled flag from a specified device.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current smart power save enabled flag.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationSmartPowerSaveEnabled(argus_hnd_t *hnd,
		argus_mode_t mode,
		bool *value);

/*!***************************************************************************
 * @brief 	Sets the Dual Frequency Mode (DFM) to a specified device.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new DFM mode value.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationDFMMode(argus_hnd_t *hnd,
				       argus_mode_t mode,
				       argus_dfm_mode_t value);


/*!***************************************************************************
 * @brief 	Gets the Dual Frequency Mode (DFM) from a specified device.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current DFM mode value.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationDFMMode(argus_hnd_t *hnd,
				       argus_mode_t mode,
				       argus_dfm_mode_t *value);

/*!***************************************************************************
 * @brief 	Sets the Shot Noise Monitor (SNM) mode to a specified device.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new SNM mode value.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationShotNoiseMonitorMode(argus_hnd_t *hnd,
		argus_mode_t mode,
		argus_snm_mode_t value);

/*!***************************************************************************
 * @brief 	Gets the Shot Noise Montor (SNM) mode from a specified device.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current SNM mode value.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationShotNoiseMonitorMode(argus_hnd_t *hnd,
		argus_mode_t mode,
		argus_snm_mode_t *value);

/*!***************************************************************************
 * @brief 	Sets the full DCA module configuration to a specified device.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new DCA configuration set.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationDynamicAdaption(argus_hnd_t *hnd,
		argus_mode_t mode,
		argus_cfg_dca_t const *value);

/*!***************************************************************************
 * @brief 	Gets the # from a specified device.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current DCA configuration set value.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationDynamicAdaption(argus_hnd_t *hnd,
		argus_mode_t mode,
		argus_cfg_dca_t *value);
/*!***************************************************************************
 * @brief 	Sets the pixel binning configuration parameters to a specified device.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new pixel binning configuration parameters.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationPixelBinning(argus_hnd_t *hnd,
		argus_mode_t mode,
		argus_cfg_pba_t const *value);

/*!***************************************************************************
 * @brief 	Gets the pixel binning configuration parameters from a specified device.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current pixel binning configuration parameters.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationPixelBinning(argus_hnd_t *hnd,
		argus_mode_t mode,
		argus_cfg_pba_t *value);

/*!***************************************************************************
 * @brief 	Gets the current unambiguous range in mm.
 * @param	hnd The API handle; contains all internal states and data.
 * @param	range_mm The returned range in mm.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationUnambiguousRange(argus_hnd_t *hnd,
		uint32_t *range_mm);

/*! @} */

/*!**************************************************************************
 * Calibration API
 ****************************************************************************
 * @addtogroup 	arguscal
 * @{
 ****************************************************************************/

/*!***************************************************************************
 * @brief 	Sets the global range offset value to a specified device.
 *
 * @details	The global range offset is subtracted from the raw range values.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new global range offset in meter and Q9.22 format.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationGlobalRangeOffset(argus_hnd_t *hnd,
		argus_mode_t mode,
		q9_22_t value);

/*!***************************************************************************
 * @brief 	Gets the global range offset value from a specified device.
 *
 * @details	The global range offset is subtracted from the raw range values.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current global range offset in meter and Q9.22 format.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationGlobalRangeOffset(argus_hnd_t *hnd,
		argus_mode_t mode,
		q9_22_t *value);

/*!***************************************************************************
 * @brief 	Sets the relative pixel offset table to a specified device.
 *
 * @details The relative pixel offset values are subtracted from the raw range
 * 			values for each individual pixel. Note that a global range offset
 * 			is applied additionally. The relative pixel offset values are meant
 * 			to be with respect to the average range of all pixels, i.e. the
 * 			average of all relative offsets should be 0!
 *
 * 			The crosstalk vector table is a two dimensional array of type
 * 			#q0_15_t.
 *
 * 			The dimensions are:
 * 			 - size(0) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 * 			 - size(1) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *			 .
 *
 * 			Its recommended to use the built-in pixel offset calibration
 * 			sequence (see #Argus_ExecuteRelativeRangeOffsetCalibrationSequence)
 * 			to determine the offset table for the current device.
 *
 * 			If a constant offset table for all device needs to be incorporated
 * 			into the sources, the #Argus_GetExternalPixelRangeOffsets_Callback
 * 			should be used.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new relative range offset in meter and Q0.15 format.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationPixelRangeOffsets(argus_hnd_t *hnd, argus_mode_t mode,
		q0_15_t value[ARGUS_PIXELS_X][ARGUS_PIXELS_Y]);


/*!***************************************************************************
 * @brief 	Gets the relative pixel offset table from a specified device.
 *
 * @details The relative pixel offset values are subtracted from the raw range
 * 			values for each individual pixel. Note that a global range offset
 * 			is applied additionally. The relative pixel offset values are meant
 * 			to be with respect to the average range of all pixels, i.e. the
 * 			average of all relative offsets should be 0!
 *
 * 			The crosstalk vector table is a two dimensional array of type
 * 			#q0_15_t.
 *
 * 			The dimensions are:
 * 			 - size(0) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 * 			 - size(1) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *			 .
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current relative range offset in meter and Q0.15 format.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationPixelRangeOffsets(argus_hnd_t *hnd, argus_mode_t mode,
		q0_15_t value[ARGUS_PIXELS_X][ARGUS_PIXELS_Y]);


/*!***************************************************************************
 * @brief 	Gets the relative pixel offset table from a specified device.
 *
 * @details The relative pixel offset values are subtracted from the raw range
 * 			values for each individual pixel. Note that a global range offset
 * 			is applied additionally. The relative pixel offset values are meant
 * 			to be with respect to the average range of all pixels, i.e. the
 * 			average of all relative offsets should be 0!
 *
 * 			The crosstalk vector table is a two dimensional array of type
 * 			#q0_15_t.
 *
 * 			The dimensions are:
 * 			 - size(0) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 * 			 - size(1) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *
 * 			The total offset table consists of the custom pixel offset values
 * 			(set via #Argus_SetCalibrationPixelRangeOffsets) and the internal,
 * 			factory calibrated device specific offset values.
 * 			This is informational only!
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current total relative range offset in meter and Q0.15 format.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationTotalPixelRangeOffsets(argus_hnd_t *hnd, argus_mode_t mode,
		q0_15_t value[ARGUS_PIXELS_X][ARGUS_PIXELS_Y]);


/*!***************************************************************************
 * @brief 	Resets the relative pixel offset values for the specified device to
 * 			the factory calibrated default values.
 *
 * @details The relative pixel offset values are subtracted from the raw range
 * 			values for each individual pixel. Note that a global range offset
 * 			is applied additionally.
 *
 * 			The factory defaults are device specific values.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ResetCalibrationPixelRangeOffsets(argus_hnd_t *hnd, argus_mode_t mode);

/*!***************************************************************************
 * @brief 	A callback that returns the external pixel range offsets.
 *
 * @details The function needs to be implemented by the host application in
 * 			order to set the external pixel range offsets values upon system
 * 			initialization. If not defined in user code, the default
 * 			implementation will return an all zero offset table, assuming there
 * 			is no (additional) external pixel range offset values.
 *
 * 			If defined in user code, the function must fill all offset values
 * 			in the provided \par offsets parameter with external range offset
 * 			values.
 * 			The values can be obtained by the calibration routine.
 *
 *			Example usage:
 *
 *			@code
 *			status_t Argus_GetExternalPixelRangeOffsets_Callback(q0_15_t offsets[ARGUS_PIXELS_X][ARGUS_PIXELS_Y],
 *																 argus_mode_t mode)
 *			{
 *				(void) mode; // Ignore mode; use same values for all modes.
 *				memset(offsets, 0, sizeof(q0_15_t) * ARGUS_PIXELS);
 *
 *				// Set offset values in meter and Q0.15 format.
 *				offsets[0][0].dS = -16384;		offsets[0][0].dC = -32768;
 *				offsets[0][1].dS = -32768;		offsets[0][1].dC = 0;
 *				offsets[0][2].dS = 16384;		offsets[0][2].dC = -16384;
 *				// etc.
 *			}
 *			@endcode
 *
 * @param	offsets The pixel range offsets in meter and Q0.15 format; to be
 * 					filled with data.
 * @param	mode Determines the current measurement mode; can be ignored if
 * 				 only a single measurement mode is utilized.
 *****************************************************************************/
void Argus_GetExternalPixelRangeOffsets_Callback(q0_15_t offsets[ARGUS_PIXELS_X][ARGUS_PIXELS_Y],
		argus_mode_t mode);

/*!***************************************************************************
 * @brief 	Sets the sample count for the range offset calibration sequence.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The new range offset calibration sequence sample count.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationRangeOffsetSequenceSampleCount(argus_hnd_t *hnd, uint16_t value);

/*!***************************************************************************
 * @brief 	Gets the sample count for the range offset calibration sequence.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The current range offset calibration sequence sample count.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationRangeOffsetSequenceSampleCount(argus_hnd_t *hnd, uint16_t *value);

/*!***************************************************************************
 * @brief 	Sets the pixel-to-pixel crosstalk compensation parameters to a specified device.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new pixel-to-pixel crosstalk compensation parameters.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationCrosstalkPixel2Pixel(argus_hnd_t *hnd,
		argus_mode_t mode,
		argus_cal_p2pxtalk_t const *value);

/*!***************************************************************************
 * @brief 	Gets the pixel-to-pixel crosstalk compensation parameters from a specified device.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current pixel-to-pixel crosstalk compensation parameters.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationCrosstalkPixel2Pixel(argus_hnd_t *hnd,
		argus_mode_t mode,
		argus_cal_p2pxtalk_t *value);


/*!***************************************************************************
 * @brief 	Sets the custom crosstalk vector table to a specified device.
 *
 * @details The crosstalk vectors are subtracted from the raw sampling data
 * 			in the data evaluation phase.
 *
 * 			The crosstalk vector table is a three dimensional array  of type
 * 			#xtalk_t.
 *
 * 			The dimensions are:
 * 			 - size(0) = #ARGUS_DFM_FRAME_COUNT (Dual-frequency mode A- or B-frame)
 * 			 - size(1) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 * 			 - size(2) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *			 .
 *
 * 			Its recommended to use the built-in crosstalk calibration sequence
 * 			(see #Argus_ExecuteXtalkCalibrationSequence) to determine the
 * 			crosstalk vector table.
 *
 * 			If a constant table for all device needs to be incorporated into
 * 			the sources, the #Argus_GetExternalCrosstalkVectorTable_Callback
 * 			should be used.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The new crosstalk vector table.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationCrosstalkVectorTable(argus_hnd_t *hnd,
		argus_mode_t mode,
		xtalk_t value[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y]);

/*!***************************************************************************
 * @brief 	Gets the custom crosstalk vector table from a specified device.
 *
 * @details The crosstalk vectors are subtracted from the raw sampling data
 * 			in the data evaluation phase.
 *
 * 			The crosstalk vector table is a three dimensional array  of type
 * 			#xtalk_t.
 *
 * 			The dimensions are:
 * 			 - size(0) = #ARGUS_DFM_FRAME_COUNT (Dual-frequency mode A- or B-frame)
 * 			 - size(1) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 * 			 - size(2) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *			 .
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current crosstalk vector table.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationCrosstalkVectorTable(argus_hnd_t *hnd,
		argus_mode_t mode,
		xtalk_t value[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y]);

/*!***************************************************************************
 * @brief 	Gets the factory calibrated default crosstalk vector table for the
 * 			specified device.
 *
 * @details The crosstalk vectors are subtracted from the raw sampling data
 * 			in the data evaluation phase.
 *
 * 			The crosstalk vector table is a three dimensional array  of type
 * 			#xtalk_t.
 *
 * 			The dimensions are:
 * 			 - size(0) = #ARGUS_DFM_FRAME_COUNT (Dual-frequency mode A- or B-frame)
 * 			 - size(1) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 * 			 - size(2) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *			 .
 *
 * 			The total vector table consists of the custom crosstalk vector
 * 			table (set via #Argus_SetCalibrationCrosstalkVectorTable) and
 * 			an internal, factory calibrated device specific vector table.
 * 			This is informational only!
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @param	value The current total crosstalk vector table.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationTotalCrosstalkVectorTable(argus_hnd_t *hnd,
		argus_mode_t mode,
		xtalk_t value[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y]);

/*!***************************************************************************
 * @brief 	Resets the crosstalk vector table for the specified device to the
 * 			factory calibrated default values.
 *
 * @details The crosstalk vectors are subtracted from the raw sampling data
 * 			in the data evaluation phase.
 * *
 * 			The factory defaults are device specific calibrated values.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	mode The targeted measurement mode.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ResetCalibrationCrosstalkVectorTable(argus_hnd_t *hnd,
		argus_mode_t mode);

/*!***************************************************************************
 * @brief 	Sets the sample count for the crosstalk calibration sequence.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The new crosstalk calibration sequence sample count.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationCrosstalkSequenceSampleCount(argus_hnd_t *hnd,
		uint16_t value);

/*!***************************************************************************
 * @brief 	Gets the sample count for the crosstalk calibration sequence.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The current crosstalk calibration sequence sample count.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationCrosstalkSequenceSampleCount(argus_hnd_t *hnd,
		uint16_t *value);

/*!***************************************************************************
 * @brief 	Sets the max. amplitude threshold for the crosstalk calibration sequence.
 *
 * @details	The maximum amplitude threshold defines a maximum crosstalk vector
 * 			amplitude before causing an error message. If the crosstalk is
 * 			too high, there is usually an issue with the measurement setup, i.e.
 * 			there is still a measurement signal detected.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The new crosstalk calibration sequence maximum amplitude
 * 					threshold value in UQ12.4 format.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold(argus_hnd_t *hnd,
		uq12_4_t value);

/*!***************************************************************************
 * @brief 	Gets the max. amplitude threshold for the crosstalk calibration sequence.
 *
 * @details	The maximum amplitude threshold defines a maximum crosstalk vector
 * 			amplitude before causing an error message. If the crosstalk is
 * 			too high, there is usually an issue with the measurement setup, i.e.
 * 			there is still a measurement signal detected.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The current max. amplitude threshold value in UQ12.4 format.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationCrosstalkSequenceAmplitudeThreshold(argus_hnd_t *hnd,
		uq12_4_t *value);

/*!***************************************************************************
 * @brief 	Sets the sample count for the substrate voltage calibration sequence.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The new substrate voltage calibration sequence sample count.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationVsubSequenceSampleCount(argus_hnd_t *hnd,
		uint16_t value);

/*!***************************************************************************
 * @brief 	Gets the sample count for the substrate voltage calibration sequence.
 *
 * @param	hnd The API handle; contains all internal states and data.
 * @param	value The current substrate voltage calibration sequence sample count.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationVsubSequenceSampleCount(argus_hnd_t *hnd,
		uint16_t *value);

/*!***************************************************************************
 * @brief 	A callback that returns the external crosstalk vector table.
 *
 * @details The function needs to be implemented by the host application in
 * 			order to set the external crosstalk vector table upon system
 * 			initialization. If not defined in user code, the default
 * 			implementation will return an all zero vector table, assuming there
 * 			is no (additional) external crosstalk.
 *
 * 			If defined in user code, the function must fill all vector values
 * 			in the provided \par xtalk parameter with external crosstalk values.
 * 			The values can be obtained by the calibration routine.
 *
 *			Example usage:
 *
 *			@code
 *			status_t Argus_GetExternalCrosstalkVectorTable_Callback(xtalk_t xtalk[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y],
 *																	argus_mode_t mode)
 *			{
 *				(void) mode; // Ignore mode; use same values for all modes.
 *				memset(&xtalk, 0, sizeof(xtalk));
 *
 *				// Set crosstalk vectors in Q11.4 format.
 *				// Note on dual-frequency frame index: 0 = A-Frame; 1 = B-Frame
 *				xtalk[0][0][0].dS = -9;		xtalk[0][0][0].dC = -11;
 *				xtalk[0][0][1].dS = -13;	xtalk[0][0][1].dC = -16;
 *				xtalk[0][0][2].dS = 6;		xtalk[0][0][2].dC = -18;
 *				// etc.
 *			}
 *			@endcode
 *
 * @param	xtalk The crosstalk vector array; to be filled with data.
 * @param	mode Determines the current measurement mode; can be ignored if
 * 				   only a single measurement mode is utilized.
 *****************************************************************************/
void Argus_GetExternalCrosstalkVectorTable_Callback(xtalk_t
		xtalk[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y],
		argus_mode_t mode);


#ifdef __cplusplus
}
#endif

/*! @} */
#endif /* ARGUS_API_H */
