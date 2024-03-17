/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides generic functionality belonging to all
 *              devices from the AFBR-S50 product family.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
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
 * @defgroup    argus_api AFBR-S50 API
 * @ingroup     argus
 *
 * @brief       The main module of the API from the AFBR-S50 SDK.
 *
 * @details     General API for the AFBR-S50 time-of-flight sensor device family.\n
 *              See the \ref getting_started Guide for a detailed description
 *              on how to use the module/API.
 *
 * @addtogroup  argus_api
 * @{
 *****************************************************************************/

#include "argus_def.h"
#include "argus_res.h"
#include "argus_pba.h"
#include "argus_dfm.h"
#include "argus_snm.h"
#include "argus_xtalk.h"
#include "argus_offset.h"

/*! The S2PI slave identifier. */
typedef int32_t s2pi_slave_t;

/*!***************************************************************************
 * @brief   Initializes the device with default measurement mode.
 *
 * @details The function that needs to be called once after power up to
 *          initialize the modules state (i.e. the corresponding handle) and the
 *          dedicated Time-of-Flight device. In order to obtain a handle,
 *          reference the #Argus_CreateHandle method.
 *
 *          Prior to calling the function, the required peripherals (i.e. S2PI,
 *          GPIO w/ IRQ and Timers) must be initialized and ready to use.
 *
 *          The function executes the following tasks:
 *          - Initialization of the internal state represented by the handle
 *            object.
 *          - Setup the device such that an safe configuration is present in
 *            the registers.
 *          - Initialize sub modules such as calibration or measurement modules.
 *          .
 *
 *          The modules configuration is initialized with reasonable default
 *          values. Note that the default measurement mode depends on the
 *          given device.
 *
 *          Also refer to #Argus_InitMode, which uses an specified measurement
 *          mode instead of the dedicated default measurement mode.
 *
 * @param   hnd The API handle; contains all internal states and data.
 *
 * @param   spi_slave The SPI hardware slave, i.e. the specified CS and IRQ
 *                    lines. This is actually just a number that is passed
 *                    to the SPI interface to distinct for multiple SPI slave
 *                    devices. Note that the slave must be not equal to 0,
 *                    since is reserved for error handling.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_Init(argus_hnd_t *hnd, s2pi_slave_t spi_slave);

/*!***************************************************************************
 * @brief   Initializes the device with specified measurement mode.
 *
 * @details The function that needs to be called once after power up to
 *          initialize the modules state (i.e. the corresponding handle) and the
 *          dedicated Time-of-Flight device. In order to obtain a handle,
 *          reference the #Argus_CreateHandle method.
 *
 *          Prior to calling the function, the required peripherals (i.e. S2PI,
 *          GPIO w/ IRQ and Timers) must be initialized and ready to use.
 *
 *          The function executes the following tasks:
 *          - Initialization of the internal state represented by the handle
 *            object.
 *          - Setup the device such that an safe configuration is present in
 *            the registers.
 *          - Initialize sub modules such as calibration or measurement modules.
 *          .
 *
 *          The modules configuration is initialized with reasonable default values.
 *
 *          Also refer to #Argus_Init, which uses the dedicated default measurement
 *          mode instead of an user specified measurement mode.
 *
 * @param   hnd The API handle; contains all internal states and data.
 *
 * @param   spi_slave The SPI hardware slave, i.e. the specified CS and IRQ
 *                    lines. This is actually just a number that is passed
 *                    to the SPI interface to distinct for multiple SPI slave
 *                    devices. Note that the slave must be not equal to 0,
 *                    since is reserved for error handling.
 *
 * @param   mode The specified measurement mode to be initialized.
 *               Pass 0 as special value to select default measurement mode
 *               (see #Argus_Init).
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_InitMode(argus_hnd_t *hnd, s2pi_slave_t spi_slave, argus_mode_t mode);

/*!***************************************************************************
 * @brief   Reinitializes the device with the current measurement mode.
 *
 * @details The function reinitializes the device with the currently active
 *          measurement mode.
 *
 *          This can be used as a soft reset for the device and API.
 *          See #Argus_Init for more information on the initialization.
 *
 *          Note that the #Argus_Init or #Argus_InitMode function must be called
 *          first! Otherwise, the function will return an error if it is called
 *          for an yet uninitialized device/handle.
 *
 *          Also refer to #Argus_ReinitMode, which uses a specified measurement
 *          mode instead of the currently active measurement mode.
 *
 * @note    If a full re-initialization is not desired, refer to the
 *          #Argus_RestoreDeviceState function that will only re-write the
 *          register map to the device to restore its state after an power
 *          cycle.
 *
 * @param   hnd The API handle; contains all internal states and data.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_Reinit(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Reinitializes the device with a specified measurement mode.
 *
 * @details The function reinitializes the device with a specified (/p mode)
 *          measurement mode.
 *
 *          This can be used as a soft reset for the device and API.
 *          See #Argus_InitMode for more information on the initialization.
 *
 *          Note that the #Argus_Init or #Argus_InitMode function must be called
 *          first! Otherwise, the function will return an error if it is called
 *          for an yet uninitialized device/handle.
 *
 *          Also refer to #Argus_Reinit, which re-uses the currently active
 *          measurement mode instead of an user specified measurement mode.
 *
 * @note    If a full re-initialization is not desired, refer to the
 *          #Argus_RestoreDeviceState function that will only re-write the
 *          register map to the device to restore its state after an power
 *          cycle.
 *
 * @param   hnd The API handle; contains all internal states and data.
 *
 * @param   mode The specified measurement mode to be initialized.
 *               Pass 0 as special value to select the current measurement mode
 *               (see #Argus_Init).
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ReinitMode(argus_hnd_t *hnd, argus_mode_t mode);

/*!***************************************************************************
 * @brief   Deinitializes the API modules and the device.
 *
 * @details The function deinitializes the device and clear all internal states.
 *          Can be used to cleanup before releasing the memory. The device
 *          can not be used any more and must be initialized again prior to next
 *          usage.
 *
 *          Note that the #Argus_Init function must be called first! Otherwise,
 *          the function will return an error if it is called for an yet
 *          uninitialized device/handle.
 *
 * @param   hnd The API handle; contains all internal states and data.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_Deinit(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Creates a new device data handle object to store all internal states.
 *
 * @details The function must be called to obtain a new device handle object.
 *          The handle is basically an abstract object in memory that contains
 *          all the internal states and settings of the API module. The handle
 *          is passed to all the API methods in order to address the specified
 *          device. This allows to use the API with more than a single measurement
 *          device.
 *
 *          The handler is created by calling the memory allocation method from
 *          the standard library: @code void * malloc(size_t size) @endcode
 *          In order to implement an individual memory allocation method,
 *          define and implement the following weakly binded method and return
 *          a pointer to the newly allocated memory. *
 *
 *          @code void * Argus_Malloc (size_t size) @endcode
 *
 *          Also see the #Argus_DestroyHandle method for the corresponding
 *          deallocation of the allocated memory.
 *
 * @note    Although the method is using memory allocated on the heap, it
 *          is eventually no dynamic memory allocation, since the block of
 *          memory is kept all the time and no memory blocks are dynamically
 *          freed and re-allocated. If the usage of heap must be avoided, one
 *          can always implement its own version of the `Argus_Malloc` function
 *          to create the memory elsewhere.
 *
 * @return  Returns a pointer to the newly allocated device handler object.
 *          Returns a null pointer if the allocation failed!
 *****************************************************************************/
argus_hnd_t *Argus_CreateHandle(void);

/*!***************************************************************************
 * @brief   Destroys a given device data handle object.
 *
 * @details The function can be called to free the previously created device
 *          data handle object in order to save memory when the device is not
 *          used any more.
 *
 *          Note that the handle must be deinitialized before it can be
 *          destroyed. The function returns #ERROR_FAIL if the handle is not
 *          yet deinitialized.
 *
 *          Please refer to the #Argus_CreateHandle method for the corresponding
 *          allocation of the memory.
 *
 *          The handler is destroyed by freeing the corresponding memory with the
 *          method from the standard library, @code void free(void * ptr) @endcode.
 *          In order to implement an individual memory deallocation method, define
 *          and implement the following weakly binded method and free the memory
 *          object passed to the method by a pointer.
 *
 *          @code void Argus_Free (void * ptr) @endcode
 *
 *          Also see the #Argus_CreateHandle method for the corresponding
 *          allocation of the required memory.
 *
 * @param   hnd The device handle object to be deallocated.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_DestroyHandle(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Restores the device state with a re-write of all register values.
 *
 * @details The function invalidates and restores the device state by executing
 *          a re-write of the full register map.
 *
 *          The purpose of this function is to recover from known external
 *          events like power cycles, for example due to sleep / wake-up
 *          functionality. This can be implemented by cutting off the external
 *          power supply of the device (e.g. via a MOSFET switch controlled by
 *          a GPIB pin). By calling this function, the expected state of the
 *          API is written to the device without the need to fully re-initialize
 *          the device. Thus, the API can resume where it has stopped as if
 *          there has never been a power cycle.
 *
 *          The internal state machines like the dynamic configuration adaption
 *          (DCA) algorithm will not be reseted. The API/sensor will immediately
 *          resume at the last state that was optimized for the given
 *          environmental conditions.
 *
 *          The use case of sleep / wake-up can be implemented as follows:
 *
 *          1. In case of ongoing measurements, stop the measurements via
 *             the #Argus_StopMeasurementTimer function (if started by the
 *             #Argus_StartMeasurementTimer function).
 *
 *          2. Shut down the device by removing the 5V power supply, e.g.
 *             via a GPIO pin that switches a MOSFET circuit.
 *
 *          3. After the desired sleep period, power the device by switching
 *             the 5V power supply on again. Wait until the power-on-reset
 *             (POR) is finished (approx. 1 ms) or just repeat step 4 until
 *             it succeeds.
 *
 *          4. Call the #Argus_RestoreDeviceState function to trigger the
 *             restoration of the device state in the API. Note that the
 *             function will return an error code if it fails. One can repeat
 *             the execution of that function a few times until it succeeds.
 *
 *          6. Continue with measurements via #Argus_StartMeasurementTimer
 *             of #Argus_TriggerMeasurement functions as desired.
 *
 * @note    If a complete re-initialization (= soft-reset) is desired, see
 *          the #Argus_Reinit functionality.
 *
 * @note    Changing a configuration or calibration parameter will always
 *          invalidate the device state as well as the state machine of the
 *          dynamic configuration adaption (DCA) algorithm. In that case, the
 *          device/API needs a few measurements to adopt to the present
 *          environmental conditions before the first valid measurement result
 *          can be obtained. This is almost similar to re-initializing the
 *          device (see #Argus_Reinit) which would also re-read the EEPROM.
 *          On the other hand, the #Argus_RestoreDeviceState does not reset
 *          or re-initialize anything. It just makes sure that the device
 *          register map (which has changed to its reset values after the
 *          power cycle) is what the API expects upon the next measurement.
 *
 * @param   hnd The device handle object to be invalidated.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_RestoreDeviceState(argus_hnd_t *hnd);

/*!**************************************************************************
 * Generic API
 ****************************************************************************/

/*!***************************************************************************
 * @brief   Gets the version number of the current API library.
 *
 * @details The version is compiled of a major (a), minor (b) and bugfix (c)
 *          number: a.b.c.
 *
 *          The values are encoded into a 32-bit value:
 *
 *           - [ 31 .. 24 ] - Major Version Number
 *           - [ 23 .. 16 ] - Minor Version Number
 *           - [ 15 ..  0 ] - Bugfix Version Number
 *           .
 *
 *          To obtain the parts from the returned uin32_t value:
 *
 *          @code
 *          uint32_t value = Argus_GetAPIVersion();
 *          uint8_t a = (value >> 24) & 0xFFU;
 *          uint8_t b = (value >> 16) & 0xFFU;
 *          uint8_t c = value & 0xFFFFU;
 *          @endcode
 *
 * @return  Returns the current version number.
 *****************************************************************************/
uint32_t Argus_GetAPIVersion(void);

/*!***************************************************************************
 * @brief   Gets the build number of the current API library.
 *
 * @return  Returns the current build number as a C-string.
 *****************************************************************************/
char const *Argus_GetBuildNumber(void);

/*!***************************************************************************
 * @brief   Gets the version/variant of the module.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the current module number.
 *****************************************************************************/
argus_module_version_t Argus_GetModuleVersion(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Gets the name string of the module.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the current module name.
 *****************************************************************************/
char const *Argus_GetModuleName(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Gets the version number of the chip.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the current version number.
 *****************************************************************************/
argus_chip_version_t Argus_GetChipVersion(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Gets the type number of the device laser.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the current device laser type number.
 *****************************************************************************/
argus_laser_type_t Argus_GetLaserType(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Gets the unique identification number of the chip.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the unique identification number.
 *****************************************************************************/
uint32_t Argus_GetChipID(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Gets the SPI hardware slave identifier.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  The SPI hardware slave identifier.
 *****************************************************************************/
s2pi_slave_t Argus_GetSPISlave(argus_hnd_t *hnd);

/*! @} */

/*!**************************************************************************
 * Measurement/Device Operation
 ****************************************************************************
 * @addtogroup  argus_meas
 * @{
 ****************************************************************************/

/*!***************************************************************************
 * @brief   Starts the timer based measurement cycle asynchronously.
 *
 * @details This function starts a timer based measurement cycle asynchronously
 *          in the background. A periodic timer interrupt triggers the measurement
 *          frames on the ASIC and the data readout afterwards.
 *
 *          When the measurement has finished, a callback (which is passed as
 *          a parameter to the function) is invoked in order to inform the
 *          main thread to call the \link #Argus_EvaluateData data evaluation
 *          method\endlink. This call is mandatory to release the data buffer
 *          for the next measurement and it must not be invoked directly from
 *          the callback since it is currently within an interrupt service
 *          routine. Rather a flag should inform the main thread or task
 *          scheduler to invoke the evaluation as soon as possible in order
 *          to not introduce any unwanted delays to the next measurement frame.
 *
 *          The next measurement frame will be started as soon as the pre-
 *          conditions are meet. These are:
 *           1. timer flag set (i.e. a certain time has passed since the last
 *              measurement in order to fulfill eye-safety),
 *           2. device idle (i.e. no measurement currently ongoing) and
 *           3. data buffer ready (i.e. the previous data has been evaluated).
 *
 *          Usually, the device idle and data buffer ready conditions are met
 *          before the timer tick occurs and thus the timer dictates the frame
 *          rate.
 *
 *          The callback function pointer will be invoked when the measurement
 *          frame has finished successfully or whenever an error, that cannot
 *          be handled internally, occurs.
 *
 *          The periodic timer interrupts are used to check the measurement status
 *          for timeouts. An error is invoked when a measurement cycle have not
 *          finished within the specified time.
 *
 *          Use #Argus_StopMeasurementTimer to stop the measurements.
 *
 * @note    In order to use this function, the periodic interrupt timer module
 *          (see @ref argus_timer) must be implemented!
 *
 * @param   hnd The API handle; contains all internal states and data.
 *
 * @param   cb  Callback function that will be invoked when the measurement
 *              is completed. Its parameters are the \link #status_t status
 *              \endlink of the finished measurement cycle and the pointer to
 *              the calling \link #argus_hnd_t API handle\endlink, i.e. the
 *              /p hnd value. The latter must be passed to the
 *              #Argus_EvaluateData function.
 *              If an error occurred, the status differs from #STATUS_OK.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_StartMeasurementTimer(argus_hnd_t *hnd,
				     argus_measurement_ready_callback_t cb);

/*!***************************************************************************
 * @brief   Stops the timer based measurement cycle.
 *
 * @details This function stops the ongoing timer based measurement cycles
 *          that have been started using the #Argus_StartMeasurementTimer
 *          function.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_StopMeasurementTimer(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Triggers a single measurement frame asynchronously.
 *
 * @details This function immediately triggers a single measurement frame
 *          asynchronously if all the pre-conditions are met. Otherwise it
 *          returns with a corresponding status (e.g. #STATUS_BUSY or
 *          #STATUS_ARGUS_POWERLIMIT).
 *
 *          When the measurement has finished, a callback (which is passed as
 *          a parameter to the function) is invoked in order to inform the
 *          main thread to call the \link #Argus_EvaluateData data evaluation
 *          method\endlink. This call is mandatory to release the data buffer
 *          for the next measurement and it must not be invoked directly from
 *          the callback since it is currently within an interrupt service
 *          routine. Rather a flag should inform the main thread or task
 *          scheduler to invoke the evaluation task.
 *
 *          The pre-conditions for starting a measurement frame are:
 *           1. timer flag set (i.e. a certain time has passed since the last
 *              measurement in order to fulfill eye-safety),
 *           2. device idle (i.e. no measurement currently ongoing) and
 *           3. data buffer ready (i.e. the previous data has been evaluated).
 *
 *          The callback function pointer will be invoked when the measurement
 *          frame has finished successfully or whenever an error, that cannot
 *          be handled internally, occurs.
 *
 *          The successful finishing of the measurement frame is not checked
 *          for timeouts! Instead, the user can call the #Argus_GetStatus()
 *          function on a regular function to do so.
 *
 * @note    Despite this function triggers a new measurement cycle upon its
 *          invocation, the frame time parameter is still active for this
 *          measurement mode. Basically, the first pre-condition mentioned
 *          above is controlled via the frame time parameter. This means
 *          that measurements cannot be triggered faster than the frame
 *          timer parameters specifies. The maximum integration time (i.e.
 *          exposure time) is also determined by the frame time such that
 *          new measurements are finished with the specified frame time and
 *          the device is ready to trigger a new measurement after the
 *          frame time has elapse.
 *          See #Argus_SetConfigurationFrameTime function for more information
 *          on the frame time.
 *
 * @param   hnd The API handle; contains all internal states and data.
 *
 * @param   cb  Callback function that will be invoked when the measurement
 *              is completed. Its parameters are the \link #status_t status
 *              \endlink of the finished measurement cycle and the pointer to
 *              the calling \link #argus_hnd_t API handle\endlink, i.e. the
 *              /p hnd value. The latter must be passed to the
 *              #Argus_EvaluateData function.
 *              If an error occurred, the status differs from #STATUS_OK.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_TriggerMeasurement(argus_hnd_t *hnd,
				  argus_measurement_ready_callback_t cb);

/*!***************************************************************************
 * @brief   Determines whether a data evaluation is pending.
 *
 * @details If the function returns true, a raw buffer is required to be
 *          evaluated to the #Argus_EvaluateData function. The raw data buffer
 *          is filled with raw data from the measurement tasks which need to
 *          be evaluated and the buffer must be freed in order to restart a
 *          new measurement task.
 *
 *          Note that no configuration parameters can be update until all raw
 *          buffers are evaluated.
 *
 * @note    See also the #Argus_GetStatus function to obtain the current device
 *          status and error code if any.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  True if any raw buffer is filled with data that must be evaluated.
 *****************************************************************************/
bool Argus_IsDataEvaluationPending(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Determines if the device if active with timer based measurements.
 * @details If the function returns true, the device is active with timer
 *          scheduled measurements that have been started via the
 *          #Argus_StartMeasurementTimer.
 *
 *          Note that the active state is independent of the busy state that
 *          is set when the device is actually busy. The active state is also
 *          true if the device is currently idle but waits for the next timer
 *          event to trigger a new measurement cycle.
 *
 * @note    See also the #Argus_GetStatus function to obtain the current device
 *          status and error code if any.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  True if the device is operating in timer triggered measurement mode.
 *****************************************************************************/
bool Argus_IsTimerMeasurementActive(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Stops the currently ongoing measurements and SPI activity immediately.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_Abort(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Checks the state of the device/driver.
 *
 * @details Returns the current module status or error if any.
 *
 *          See the following for a list of errors:
 *
 *          Status:
 *          - Idle/OK: Device and SPI interface are idle (== #STATUS_IDLE).
 *          - Busy: Device or SPI interface are busy (== #STATUS_BUSY).
 *          - Initializing: The modules and devices are currently initializing
 *                          (== #STATUS_INITIALIZING).
 *          .
 *
 *          Error:
 *          - Not Initialized: The modules (or any submodule) has not been
 *                             initialized yet (== #ERROR_NOT_INITIALIZED).
 *          - Not Connected: No device has been connected (or connection errors
 *                           have occurred) (== #ERROR_ARGUS_NOT_CONNECTED).
 *          - Timeout: A previous frame measurement has not finished within a
 *                     specified time (== #ERROR_TIMEOUT).
 *          .
 *
 * @note    Note that this function returns the actual busy state. This means
 *          that it will return #STATUS_IDLE during the pause between two
 *          consecutive measurement frames. If the device is active with timer
 *          based measurements (i.e. started via the #Argus_StartMeasurementTimer
 *          function), the return state switches from idle to busy and back
 *          periodically. Use the #Argus_IsTimerMeasurementActive function in
 *          order to determine if the device is active with timer based
 *          measurements.
 *
 * @note    Note also that the device might reject configuration parameter
 *          update despite the status is #STATUS_IDLE. This is due to the fact
 *          that the internal raw data buffers are still busy and require to
 *          be freed by passing them to the #Argus_EvaluateData function. Use
 *          the #Argus_IsDataEvaluationPending function to see whether any of
 *          the raw data buffers is busy or the configuration can be changed.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetStatus(argus_hnd_t *hnd);

/*!*****************************************************************************
 * @brief   Tests the connection to the device by sending a ping message.
 *
 * @details A ping is transferred to the device in order to check the device and
 *          SPI connection status. Returns #STATUS_OK on success and
 *          #ERROR_ARGUS_NOT_CONNECTED else-wise.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 ******************************************************************************/
status_t Argus_Ping(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Evaluates measurement data from the raw sensor readout data.
 *
 * @details This function must be called after each completion of a measurement
 *          cycle. The completion of a measurement cycle is communicated by the
 *          API via the invocation of the measurement data ready callback. The
 *          callback is installed in the API when new measurements are started
 *          either via the #Argus_TriggerMeasurement or via the
 *          #Argus_StartMeasurementTimer functions.
 *
 *          This function evaluates measurement values like distances, amplitudes
 *          states and auxiliary values like temperature or voltage values from
 *          the raw sensor readout data obtained from the device during the
 *          measurement cycle. A pointer to a #argus_results_t data structure
 *          must be passed where all the evaluated values will be written to.
 *          The structure must persist during the whole execution of the
 *          #Argus_EvaluateData function.
 *
 *          In addition to the evaluation of measurement data, the function
 *          feeds back the obtained information to the device in order to
 *          optimize its performance with respect to the ambient conditions,
 *          utilizing the so called Dynamic Configuration Adaption (DCA)
 *          feature.
 *
 *          Furthermore, several calibration algorithm are applied to the data.
 *
 *          If the function is called without any data ready to be evaluated
 *          from the measurement module, the error code #ERROR_ARGUS_BUFFER_EMPTY
 *          is returned and not data is written to the passed #argus_results_t
 *          data structure.
 *
 * @note    The call to this function is mandatory for each finished measurement
 *          cycle, i.e. for each call to the measurement data ready callback.
 *          If the function is not called, the data is not evaluated and the
 *          internal raw data buffers are not freed. In that case, they can not
 *          be reused for the next measurement and the API can not start new
 *          measurements.
 *          There are up to two internal buffers available, the to callback
 *          is called twice before the API must wait for the data evaluation
 *          to finish.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   res A pointer to the results structure that will be populated
 *                with evaluated data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_EvaluateData(argus_hnd_t *hnd, argus_results_t *res);

/*!***************************************************************************
 * @brief   Evaluates measurement data from the raw sensor readout data.
 *
 * @details This function enhances the #Argus_EvaluateData by adding additional
 *          debug data into a specified debug data structure (\p dbg). If the
 *          \p dbg is null, the function is eqivalent to the #Argus_EvaluateData
 *          function. This, see #Argus_EvaluateData for reference.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   res A pointer to the results structure that will be populated
 *                with evaluated data.
 * @param   dbg An optional pointer (can be null) to the debug data structure.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_EvaluateDataDebug(argus_hnd_t *hnd, argus_results_t *res,
				 argus_results_debug_t *dbg);

/*!***************************************************************************
 * @brief   Executes a crosstalk calibration measurement.
 *
 * @details This function immediately triggers a crosstalk vector calibration
 *          measurement sequence. The ordinary measurement activity is suspended
 *          while the calibration is ongoing.
 *
 *          In order to perform a crosstalk calibration, the reflection of the
 *          transmitted signal must be kept from the receiver side, by either
 *          covering the TX completely (or RX respectively) or by setting up
 *          an absorbing target at far distance.
 *
 *          After calibration has finished successfully, the obtained data is
 *          applied immediately and can be read from the API using the
 *          #Argus_GetCalibrationCrosstalkVectorTable function.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ExecuteXtalkCalibrationSequence(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Executes a relative range offset calibration measurement.
 *
 * @details This function immediately triggers a relative range offset calibration
 *          measurement sequence. The ordinary measurement activity is suspended
 *          while the calibration is ongoing.
 *
 *          In order to perform a relative range offset calibration, a flat
 *          calibration target must be setup perpendicular to the sensors
 *          field-of-view.
 *
 *          \code
 *                           AFBR-S50 ToF Sensor
 *                    #|
 *                    #|                                         |
 *                    #|-----+                                   |
 *                    #| RX  |                                   |
 *          Reference #|----++                                   | Calibration
 *              Plane #| TX |                                    | Target
 *                    #|----+                                    |
 *                    #|                                         |
 *                    #| <------- targetRange -----------------> |
 *          \endcode
 *
 *          There are two options to run the offset calibration: relative and
 *          absolute.
 *
 *          - Relative (#Argus_ExecuteRelativeRangeOffsetCalibrationSequence):
 *            when the absolute distance is not essential or the distance to
 *            the calibration target is not known, the relative method can be
 *            used to compensate the relative pixel range offset w.r.t. the
 *            average range. The absolute or global range offset is not changed.
 *          - Absolute (#Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence):
 *            when the absolute distance is essential and the distance to the
 *            calibration target is known, the absolute method can be used to
 *            calibrate the absolute measured distance. Additionally, the
 *            relative pixel offset w.r.t. the average range is also compensated.
 *          .
 *
 *          After calibration has finished successfully, the obtained data is
 *          applied immediately and can be read from the API using the
 *          #Argus_GetCalibrationPixelRangeOffsets or
 *          #Argus_GetCalibrationGlobalRangeOffsets function.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ExecuteRelativeRangeOffsetCalibrationSequence(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Executes an absolute range offset calibration measurement.
 *
 * @details This function immediately triggers an absolute range offset calibration
 *          measurement sequence. The ordinary measurement activity is suspended
 *          while the calibration is ongoing.
 *
 *          In order to perform a relative range offset calibration, a flat
 *          calibration target must be setup perpendicular to the sensors
 *          field-of-view.
 *
 *          \code
 *                           AFBR-S50 ToF Sensor
 *                    #|
 *                    #|                                         |
 *                    #|-----+                                   |
 *                    #| RX  |                                   |
 *          Reference #|----++                                   | Calibration
 *              Plane #| TX |                                    | Target
 *                    #|----+                                    |
 *                    #|                                         |
 *                    #| <------- targetRange -----------------> |
 *          \endcode
 *
 *          There are two options to run the offset calibration: relative and
 *          absolute.
 *
 *          - Relative (#Argus_ExecuteRelativeRangeOffsetCalibrationSequence):
 *            when the absolute distance is not essential or the distance to
 *            the calibration target is not known, the relative method can be
 *            used to compensate the relative pixel range offset w.r.t. the
 *            average range. The absolute or global range offset is not changed.
 *          - Absolute (#Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence):
 *            when the absolute distance is essential and the distance to the
 *            calibration target is known, the absolute method can be used to
 *            calibrate the absolute measured distance. Additionally, the
 *            relative pixel offset w.r.t. the average range is also compensated.
 *          .
 *
 *          After calibration has finished successfully, the obtained data is
 *          applied immediately and can be read from the API using the
 *          #Argus_GetCalibrationPixelRangeOffsets or
 *          #Argus_GetCalibrationGlobalRangeOffsets function.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   targetRange The absolute range between the reference plane and the
 *                      calibration target in meter an Q9.22 format.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence(argus_hnd_t *hnd,
		q9_22_t targetRange);

/*! @} */

/*!**************************************************************************
 * Configuration API
 ****************************************************************************
 * @addtogroup  argus_cfg
 * @{
 ****************************************************************************/

/*!***************************************************************************
 * @brief   Gets the default measurement mode for a specified module type.
 *
 * @param   module The specified module type.
 * @return  Returns the default measurement mode for the specified module type.
 *****************************************************************************/
argus_mode_t Argus_GetDefaultMeasurementMode(argus_module_version_t module);

/*!***************************************************************************
 * @brief   Sets the measurement mode to a specified device.
 *
 * @details This generates a new default configuration and calibration for the
 *          specified measurement mode and applies it to the device.
 *
 *          See #argus_mode_t for a list of all available measurement modes.
 *
 * @warning The function overwrites all made changes to the configuration or
 *          calibration parameters with the default values. So this function
 *          must be called before any other changes to the configuration or
 *          calibration parameters are made!
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   mode The new measurement mode.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetMeasurementMode(argus_hnd_t *hnd, argus_mode_t mode);

/*!***************************************************************************
 * @brief   Resets the measurement mode to a specified device.
 *
 * @details This generates a new default configuration and calibration for the
 *          current measurement mode and applies it to the device.
 *
 * @warning The function overwrites all made changes to the configuration or
 *          calibration parameters with the default values. So this function
 *          must be called before any other changes to the configuration or
 *          calibration parameters are made!
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ResetMeasurementMode(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Gets the measurement mode from a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   mode The current measurement mode.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetMeasurementMode(argus_hnd_t *hnd, argus_mode_t *mode);

/*!***************************************************************************
 * @brief   Sets the frame time to a specified device.
 *
 * @details The frame time determines the measurement rate of the device.
 *          Usually, this controller the periodicity of measurements to be
 *          triggered via the timer based measurement mode that can be started
 *          via the #Argus_StartMeasurementTimer function. But also the
 *          behavior of the #Argus_TriggerMeasurement function is influenced
 *          by the frame rate parameter.
 *
 *          The frame time parameter handles the maximum frame rate by limiting
 *          the trigger of a new measurement frame to the specified value.
 *          On the other hand, the accuracy of measurement results it also
 *          influenced since the frame time specifies the maximum integration
 *          depth (i.e. exposure time) along with the laser safety limitations.
 *          This means, the measurement speed can be increased by decreasing
 *          the frame time parameter and the accuracy can be improved by
 *          increasing the frame time parameter.
 *
 *          Note the additional factor will limit the maximum frame rate on the
 *          one hand and the accuracy on the other hand:
 *          - High CPU load (or slow CPU in general) will lead to delays due
 *            to long data evaluation task (#Argus_EvaluateData) or long user
 *            application code. Reduce CPU load or increase CPU power to
 *            increase maximum frame rate.
 *          - The dual frequency mode (DFM, see #Argus_SetConfigurationDFMMode)
 *            will additionally limit the maximum frame rate to approximately
 *            100 frames per second. Disable the DFM to increase maximum frame
 *            rates.
 *          - The smart power save (SPS, see
 *            #Argus_SetConfigurationSmartPowerSaveEnabled) mode will decrease
 *            the maximum possible frame rate slightly. Disable it to increase
 *            the maximum frame rate.
 *          - The dynamic configuration adaption with its specific power saving
 *            ratio parameter (see #Argus_SetConfigurationDynamicAdaption)
 *            will limit the maximum integration depth along with the laser
 *            safety limitations. Increase the power saving ratio to increase
 *            accuracy. Note that laser safety limitations might already limit
 *            the maximum integration depth such that the power saving ratio
 *            is ineffective.
 *          .
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The measurement frame time in microseconds.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationFrameTime(argus_hnd_t *hnd, uint32_t value);

/*!***************************************************************************
 * @brief   Gets the frame time from a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current frame time in microseconds.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationFrameTime(argus_hnd_t *hnd, uint32_t *value);

/*!***************************************************************************
 * @brief   Sets the smart power save enabled flag to a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new smart power save enabled flag.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationSmartPowerSaveEnabled(argus_hnd_t *hnd,
		bool value);

/*!***************************************************************************
 * @brief   Gets the smart power save enabled flag from a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current smart power save enabled flag.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationSmartPowerSaveEnabled(argus_hnd_t *hnd,
		bool *value);

/*!***************************************************************************
 * @brief   Sets the Dual Frequency Mode (DFM) to a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new DFM mode value.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationDFMMode(argus_hnd_t *hnd,
				       argus_dfm_mode_t value);


/*!***************************************************************************
 * @brief   Gets the Dual Frequency Mode (DFM) from a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current DFM mode value.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationDFMMode(argus_hnd_t *hnd,
				       argus_dfm_mode_t *value);

/*!***************************************************************************
 * @brief   Sets the Shot Noise Monitor (SNM) mode to a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new SNM mode value.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationShotNoiseMonitorMode(argus_hnd_t *hnd,
		argus_snm_mode_t value);

/*!***************************************************************************
 * @brief   Gets the Shot Noise Monitor (SNM) mode from a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current SNM mode value.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationShotNoiseMonitorMode(argus_hnd_t *hnd,
		argus_snm_mode_t *value);

/*!***************************************************************************
* @brief    Sets the Crosstalk Monitor (XTM) mode to a specified device.
*
* @param    hnd The API handle; contains all internal states and data.
* @param    value The new XTM mode value (true: enabled; false: disabled).
* @return   Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t Argus_SetConfigurationCrosstalkMonitorMode(argus_hnd_t *hnd,
		bool value);

/*!***************************************************************************
* @brief    Gets the Crosstalk Monitor (XTM) mode from a specified device.
*
* @param    hnd The API handle; contains all internal states and data.
* @param    value The current XTM mode value (true: enabled; false: disabled).
* @return   Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t Argus_GetConfigurationCrosstalkMonitorMode(argus_hnd_t *hnd,
		bool *value);

/*!***************************************************************************
 * @brief   Sets the full DCA module configuration to a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new DCA configuration set.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationDynamicAdaption(argus_hnd_t *hnd,
		argus_cfg_dca_t const *value);

/*!***************************************************************************
 * @brief   Gets the # from a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current DCA configuration set value.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationDynamicAdaption(argus_hnd_t *hnd,
		argus_cfg_dca_t *value);
/*!***************************************************************************
 * @brief   Sets the pixel binning configuration parameters to a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new pixel binning configuration parameters.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetConfigurationPixelBinning(argus_hnd_t *hnd,
		argus_cfg_pba_t const *value);

/*!***************************************************************************
 * @brief   Gets the pixel binning configuration parameters from a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current pixel binning configuration parameters.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationPixelBinning(argus_hnd_t *hnd,
		argus_cfg_pba_t *value);

/*!***************************************************************************
 * @brief   Gets the current unambiguous range in mm.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   range_mm The returned range in mm.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetConfigurationUnambiguousRange(argus_hnd_t *hnd,
		uint32_t *range_mm);

/*! @} */

/*!**************************************************************************
 * Calibration API
 ****************************************************************************
 * @addtogroup  argus_cal
 * @{
 ****************************************************************************/

/*!***************************************************************************
 * @brief   Sets the global range offset values to a specified device.
 *
 * @details The global range offsets are subtracted from the raw range values.
 *          There are two distinct values that are applied in low or high
 *          power stage setting respectively.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   offset_low The new global range offset for the low power stage in
 *                     meter and Q0.15 format.
 * @param   offset_high The new global range offset for the high power stage in
 *                      meter and Q0.15 format.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationGlobalRangeOffsets(argus_hnd_t *hnd,
		q0_15_t offset_low,
		q0_15_t offset_high);

/*!***************************************************************************
 * @brief   Gets the global range offset values from a specified device.
 *
 * @details The global range offsets are subtracted from the raw range values.
 *          There are two distinct values that are applied in low or high
 *          power stage setting respectively.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   offset_low The current range offset for the low power stage in
 *                     meter and Q0.15 format.
 * @param   offset_high The current global range offset for the high power stage
 *                      in meter and Q0.15 format.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationGlobalRangeOffsets(argus_hnd_t *hnd,
		q0_15_t *offset_low,
		q0_15_t *offset_high);

/*!***************************************************************************
 * @brief   Sets the relative pixel offset table to a specified device.
 *
 * @details The relative pixel offset values are subtracted from the raw range
 *          values for each individual pixel. Note that a global range offset
 *          is applied additionally. The relative pixel offset values are meant
 *          to be with respect to the average range of all pixels, i.e. the
 *          average of all relative offsets should be 0!
 *
 *          The crosstalk vector table is a two dimensional array of type
 *          #q0_15_t, wrapped within the #argus_cal_offset_table_t structure.
 *
 *          The dimensions are:
 *           - size(0) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 *           - size(1) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *           .
 *
 *          Its recommended to use the built-in pixel offset calibration
 *          sequence (see #Argus_ExecuteRelativeRangeOffsetCalibrationSequence)
 *          to determine the offset table for the current device.
 *
 *          If a constant offset table for all device needs to be incorporated
 *          into the sources, the #Argus_GetPixelRangeOffsets_Callback
 *          should be used.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new relative range offset in meter and Q0.15 format.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationPixelRangeOffsets(argus_hnd_t *hnd,
		argus_cal_offset_table_t const *value);


/*!***************************************************************************
 * @brief   Gets the relative pixel offset table from a specified device.
 *
 * @details The relative pixel offset values are subtracted from the raw range
 *          values for each individual pixel. Note that a global range offset
 *          is applied additionally. The relative pixel offset values are meant
 *          to be with respect to the average range of all pixels, i.e. the
 *          average of all relative offsets should be 0!
 *
 *          The crosstalk vector table is a two dimensional array of type
 *          #q0_15_t, wrapped within the #argus_cal_offset_table_t structure.
 *
 *          The dimensions are:
 *           - size(0) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 *           - size(1) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *           .
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current relative range offset in meter and Q0.15 format.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationPixelRangeOffsets(argus_hnd_t *hnd,
		argus_cal_offset_table_t *value);


/*!***************************************************************************
 * @brief   Resets the relative pixel offset values for the specified device to
 *          the factory calibrated default values.
 *
 * @details The relative pixel offset values are subtracted from the raw range
 *          values for each individual pixel. Note that a global range offset
 *          is applied additionally.
 *
 *          The factory defaults are device specific values.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ResetCalibrationPixelRangeOffsets(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   A callback that returns the external pixel range offsets.
 *
 * @details The function needs to be implemented by the host application in
 *          order to set the external pixel range offsets values upon system
 *          initialization. If not defined in user code, the default
 *          implementation will return an all zero offset table, assuming there
 *          is no (additional) external pixel range offset values.
 *
 *          If defined in user code, the function must fill all offset values
 *          in the provided \par offsets parameter with external range offset
 *          values.
 *          The values can be obtained by the calibration routine.
 *
 *          Example usage:
 *
 *          @code
 *          status_t Argus_GetPixelRangeOffsets_Callback(argus_cal_offset_table_t offsets)
 *          {
 *              memset(offsets, 0, sizeof(argus_cal_offset_t));
 *
 *              // Set offset values in meter and Q0.15 format.
 *              offsets.Table[0][0] = -3542;
 *              offsets.Table[0][1] = -4385;
 *              offsets.Table[0][2] = 2953;
 *              // etc.
 *          }
 *          @endcode
 *
 * @param   offsets The pixel range offsets in meter and Q0.15 format; to be
 *                  filled with data.
 * @param   mode The current measurement mode.
 *****************************************************************************/
void Argus_GetPixelRangeOffsets_Callback(argus_cal_offset_table_t *offsets,
		argus_mode_t const mode);

/*!***************************************************************************
 * @brief   Sets the sample time for the range offset calibration sequence.
 *
 * @details Gets the measurement sample acquisition time for executing the
 *          range offset calibration sequence and generate the offset data.\n
 *          Units: msec.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new range offset calibration sequence sample time.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationRangeOffsetSequenceSampleTime(argus_hnd_t *hnd, uint16_t value);

/*!***************************************************************************
 * @brief   Gets the sample time for the range offset calibration sequence.
 *
 * @details Gets the measurement sample acquisition time for executing the
 *          range offset calibration sequence and generate the offset data.\n
 *          Units: msec.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current range offset calibration sequence sample time.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationRangeOffsetSequenceSampleTime(argus_hnd_t *hnd, uint16_t *value);

/*!***************************************************************************
 * @brief   Sets the pixel-to-pixel crosstalk compensation parameters to a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new pixel-to-pixel crosstalk compensation parameters.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationCrosstalkPixel2Pixel(argus_hnd_t *hnd,
		argus_cal_p2pxtalk_t const *value);

/*!***************************************************************************
 * @brief   Gets the pixel-to-pixel crosstalk compensation parameters from a specified device.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current pixel-to-pixel crosstalk compensation parameters.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationCrosstalkPixel2Pixel(argus_hnd_t *hnd,
		argus_cal_p2pxtalk_t *value);


/*!***************************************************************************
 * @brief   Sets the custom crosstalk vector table to a specified device.
 *
 * @details The crosstalk vectors are subtracted from the raw sampling data
 *          in the data evaluation phase.
 *
 *          The crosstalk vector table is a three dimensional array  of type
 *          #xtalk_t. The #argus_cal_xtalk_table_t is the corresponding
 *          typedef for the required data.
 *
 *          The dimensions are:
 *           - size(0) = #ARGUS_DFM_FRAME_COUNT (Dual-frequency mode A- or B-frame)
 *           - size(1) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 *           - size(2) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *           .
 *
 *          Its recommended to use the built-in crosstalk calibration sequence
 *          (see #Argus_ExecuteXtalkCalibrationSequence) to determine the
 *          crosstalk vector table.
 *
 *          If a constant table for all device needs to be incorporated into
 *          the sources, the #Argus_GetCrosstalkVectorTable_Callback
 *          should be used.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new crosstalk vector table.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationCrosstalkVectorTable(argus_hnd_t *hnd,
		argus_cal_xtalk_table_t const *value);

/*!***************************************************************************
 * @brief   Gets the custom crosstalk vector table from a specified device.
 *
 * @details The crosstalk vectors are subtracted from the raw sampling data
 *          in the data evaluation phase.
 *
 *          The crosstalk vector table is a three dimensional array  of type
 *          #xtalk_t. The #argus_cal_xtalk_table_t is the corresponding
 *          typedef for the required data.
 *
 *          The dimensions are:
 *           - size(0) = #ARGUS_DFM_FRAME_COUNT (Dual-frequency mode A- or B-frame)
 *           - size(1) = #ARGUS_PIXELS_X (Pixel count in x-direction)
 *           - size(2) = #ARGUS_PIXELS_Y (Pixel count in y-direction)
 *           .
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current crosstalk vector table.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationCrosstalkVectorTable(argus_hnd_t *hnd,
		argus_cal_xtalk_table_t *value);

/*!***************************************************************************
 * @brief   Resets the crosstalk vector table for the specified device to the
 *          factory calibrated default values.
 *
 * @details The crosstalk vectors are subtracted from the raw sampling data
 *          in the data evaluation phase.
 * *
 *          The factory defaults are device specific calibrated values.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ResetCalibrationCrosstalkVectorTable(argus_hnd_t *hnd);

/*!***************************************************************************
 * @brief   Sets the sample time for the crosstalk calibration sequence.
 *
 * @details Sets the measurement sample acquisition time for executing the
 *          crosstalk calibration sequence and generate the crosstalk data.\n
 *          Units: msec.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new crosstalk calibration sequence sample time.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationCrosstalkSequenceSampleTime(argus_hnd_t *hnd,
		uint16_t value);

/*!***************************************************************************
 * @brief   Gets the sample time for the crosstalk calibration sequence.
 *
 * @details Gets the measurement sample acquisition time for executing the
 *          crosstalk calibration sequence and generate the crosstalk data.\n
 *          Units: msec.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current crosstalk calibration sequence sample time.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationCrosstalkSequenceSampleTime(argus_hnd_t *hnd,
		uint16_t *value);

/*!***************************************************************************
 * @brief   Sets the max. amplitude threshold for the crosstalk calibration sequence.
 *
 * @details The maximum amplitude threshold defines a maximum crosstalk vector
 *          amplitude before causing an error message. If the crosstalk is
 *          too high, there is usually an issue with the measurement setup, i.e.
 *          there is still a measurement signal detected.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The new crosstalk calibration sequence maximum amplitude
 *                  threshold value in UQ12.4 format.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_SetCalibrationCrosstalkSequenceAmplitudeThreshold(argus_hnd_t *hnd,
		uq12_4_t value);

/*!***************************************************************************
 * @brief   Gets the max. amplitude threshold for the crosstalk calibration sequence.
 *
 * @details The maximum amplitude threshold defines a maximum crosstalk vector
 *          amplitude before causing an error message. If the crosstalk is
 *          too high, there is usually an issue with the measurement setup, i.e.
 *          there is still a measurement signal detected.
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   value The current max. amplitude threshold value in UQ12.4 format.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationCrosstalkSequenceAmplitudeThreshold(argus_hnd_t *hnd,
		uq12_4_t *value);


/*!***************************************************************************
 * @brief   Clears all user calibration values from NVM for the specified device.
 *
 * @details The user calibration values are stored in the non-volatile memory
 *          (NVM) if corresponding \link #argus_nvm NVM hardware layer\endlink
 *          is implemented. This method clears the user calibration data from
 *          the non-volatile memory.
 *
 * @warning This does not reset the currently set calibration values to
 *          factory defaults!
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_ClearUserCalibration(argus_hnd_t *hnd);


/*!***************************************************************************
 * @brief   A callback that returns the external crosstalk vector table.
 *
 * @details The function needs to be implemented by the host application in
 *          order to set the external crosstalk vector table upon system
 *          initialization. If not defined in user code, the default
 *          implementation will return an all zero vector table, assuming there
 *          is no (additional) external crosstalk.
 *
 *          If defined in user code, the function must fill all vector values
 *          in the provided \par crosstalk parameter with external crosstalk
 *          values. The values can be obtained by the calibration routine.
 *
 *          Example usage:
 *
 *          @code
 *          status_t Argus_GetCrosstalkVectorTable_Callback(
 *                          argus_cal_xtalk_table_t * xtalk)
 *          {
 *              memset(xtalk, 0, sizeof(argus_cal_xtalk_table_t));
 *
 *              // Set crosstalk vectors in Q11.4 format.
 *              // Note on dual-frequency frame index: 0 = A-Frame; 1 = B-Frame
 *              xtalk.FrameA[0][0].dS = -9;     xtalk.FrameB[0][0].dC = -11;
 *              xtalk.FrameA[0][1].dS = -13;    xtalk.FrameB[0][1].dC = -16;
 *              xtalk.FrameA[0][2].dS = 6;      xtalk.FrameB[0][2].dC = -18;
 *              // etc.
 *          }
 *          @endcode
 *
 * @param   xtalk The crosstalk vector array; to be filled with data.
 * @param   mode The current measurement mode.
 *****************************************************************************/
void Argus_GetCrosstalkVectorTable_Callback(argus_cal_xtalk_table_t *xtalk,
		argus_mode_t const mode);


/*!***************************************************************************
 * @brief   Gets the currently calibrated Golden Pixel coordinates.
 *
 * @details The Golden Pixel is the pixel that is located at the center of the
 *          receiving light beam. Thus it it the one that receives the most
 *          signal and plays a central role in 1D measurement systems.
 *
 *          The function fills the provided \p x and \p y parameters with
 *          the Golden Pixel coordinates. Typical values are x = 5 and y = 1
 *          or 2. But the actual values depend on the specific sensor.
 *
 *          Please also note the utility functions provided in the \ref argus_map
 *          module to convert between pixel coordinates and channel numbers or
 *          shift pixel maps by a position offset (#ShiftSelectedPixels) or
 *          generate pixel masks centered around the Golden Pixel
 *          (#FillPixelMask).
 *
 * @param   hnd The API handle; contains all internal states and data.
 * @param   x The Golden Pixel x-coordinate.
 * @param   y The Golden Pixel y-coordinate.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_GetCalibrationGoldenPixel(argus_hnd_t const *hnd, uint8_t *x, uint8_t *y);

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_API_H */
