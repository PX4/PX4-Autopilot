/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides an interface for the required S2PI module.
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

#ifndef ARGUS_S2PI_H
#define ARGUS_S2PI_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_s2pi S2PI: Serial Peripheral Interface
 * @ingroup     argus_hal
 *
 * @brief       S2PI: SPI incl. GPIO Hardware Layer Module
 *
 * @details     The S2PI module consists of a standard SPI interface plus a
 *              single GPIO interrupt line. Furthermore, the SPI pins are
 *              accessible via GPIO control to allow a software emulation of
 *              additional protocols using the same pins.
 *
 *              **SPI interface:**
 *
 *              The SPI interface is based around a single functionality:
 *
 *              #S2PI_TransferFrame. This function transfers a specified number
 *              of bytes via the interfaces MOSI line and simultaneously reads
 *              the incoming data on the MOSI line. The read can also be skipped.
 *              The transfer happen asynchronously, e.g. via a DMA request. After
 *              finishing the transfer, the provided callback is invoked with
 *              the status of the transfer and the provided abstract parameter.
 *              Furthermore, the functions receives a slave parameter that can
 *              be used to connect multiple slaves, each with its individual
 *              chip select line.
 *
 *              The interface also provides functionality to change the SPI
 *              baud rate. An additional abort method is used to cancel the
 *              ongoing transfer.
 *
 *              **GPIO interface:**
 *
 *              The GPIO part of the S2PI interface has two distinct concerns:
 *
 *              First, the GPIO interface handles the measurement finished interrupt
 *              from the device. When the device invokes the interrupt, it pulls
 *              the interrupt line to low. Thus the interrupt must trigger when
 *              a transition from high to low occurs on the interrupt line.
 *
 *              The module simply invokes a callback when this interrupt occurs.
 *              The #S2PI_SetIrqCallback method is used to install the callback
 *              for a specified slave. Each slave will have its own interrupt
 *              line. An additional callback parameter can be set that would be
 *              passed to the callback function.
 *
 *              In addition to the interrupt, all SPI pins need to be accessible
 *              as GPIO pins through this interface. This is required to read
 *              the EEPROM memory on the device hat is connected to the SPI
 *              pins but requires a different protocol that is not compatible
 *              to any standard SPI interface. Therefore, the interface provides
 *              the possibility to switch to GPIO control mode that allows to
 *              emulate the EEPROM protocol via software bit banging.
 *
 *              Two methods are provided to switch forth and back between SPI
 *              and GPIO control. In GPIO mode, several functions are used to
 *              read and write the individual GPIO pins.
 *
 *              Note that the GPIO mode is only required to readout the EEPROM
 *              upon initialization of the device, i.e. during execution of the
 *              #Argus_Init or #Argus_Reinit methods. The GPIO mode is not used
 *              during measurements.
 *
 *
 * @addtogroup  argus_s2pi
 * @{
 *****************************************************************************/

#include "api/argus_def.h"

/*!***************************************************************************
 * @brief   S2PI layer callback function type for the SPI transfer completed event.
 *
 * @param   status The \link #status_t status\endlink of the completed
 *                   transfer (#STATUS_OK on success).
 *
 * @param   param The provided (optional, can be null) callback parameter.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
typedef status_t (*s2pi_callback_t)(status_t status, void *param);

/*!***************************************************************************
 * @brief   S2PI layer callback function type for the GPIO interrupt event.
 *
 * @param   param The provided (optional, can be null) callback parameter.
 *****************************************************************************/
typedef void (*s2pi_irq_callback_t)(void *param);

/*! The S2PI slave identifier. Basically an user defined enumerable type that
 *  can be used to identify the slave within the SPI module. */
typedef int32_t s2pi_slave_t;

/*! The enumeration of S2PI pins. */
typedef enum s2pi_pin_t {
	/*! The SPI clock pin. */
	S2PI_CLK,

	/*! The SPI chip select pin. */
	S2PI_CS,

	/*! The SPI MOSI pin. */
	S2PI_MOSI,

	/*! The SPI MISO pin. */
	S2PI_MISO,

	/*! The IRQ pin. */
	S2PI_IRQ

} s2pi_pin_t;


/*!***************************************************************************
 * @brief   Returns the status of the SPI module.
 *
 * @param   slave The specified S2PI slave. Note that the slave information is
 *                only required if multiple SPI instances are used in order to
 *                map to the correct SPI instance.
 *
 * @return  Returns the \link #status_t status\endlink:
 *           - #STATUS_IDLE: No SPI transfer or GPIO access is ongoing.
 *           - #STATUS_BUSY: An SPI transfer is in progress.
 *           - #STATUS_S2PI_GPIO_MODE: The module is in GPIO mode.
 *****************************************************************************/
status_t S2PI_GetStatus(s2pi_slave_t slave);

/*!***************************************************************************
 * @brief   Tries to grab the SPI interface mutex for the next transfer.
 *
 * @details This mutex prevents new asynchronous SPI requests to interfere
 *          with transfers already in progress for this interface.
 *
 *          Note that this is only required if multiple device are connected to
 *          a single SPI interface. If only operating a single device per SPI,
 *          the function can simply always return #STATUS_OK.
 *
 *          There must be a dedicated mutex object per SPI interface if
 *          multiple SPI interfaces are used.
 *
 *          The mutex will be released in the #S2PI_ReleaseMutex function.
 *          See #S2PI_ReleaseMutex for additional information.
 *
 *          Here is a simple example implementation for the multiple devices on
 *          a single SPI interface case. Note that the SpiMutexBlocked must be
 *          defined per SPI interface if multiple SPI interfaces are used.
 *
 *          @code
 *          static volatile bool SpiMutexBlocked = false;
 *          status_t S2PI_TryGetMutex(s2pi_slave_t slave)
 *          {
 *              (void) slave; // not used in this implementation as all
 *                            // SPI slaves are on the same SPI interface
 *
 *              status_t status = STATUS_BUSY;
 *              IRQ_LOCK();
 *              if (!SpiMutexBlocked)
 *              {
 *                  SpiMutexBlocked = true;
 *                  status = STATUS_OK;
 *              }
 *              IRQ_UNLOCK();
 *              return status;
 *          }
 *          void S2PI_ReleaseMutex(s2pi_slave_t slave)
 *          {
 *              (void) slave; // not used in this implementation
 *              SpiMutexBlocked = false;
 *          }
 *          @endcode
 *
 * @param   slave The specified S2PI slave. Note that the slave information is
 *                only required if multiple SPI instances are used in order to
 *                map to the correct SPI instance.
 *
 * @return  Returns the \link #status_t status\endlink:
 *           - #STATUS_OK: the SPI interface was successfully reserved for the caller
 *           - #STATUS_BUSY: another transfer is ongoing, the caller must not access the bus
 *****************************************************************************/
status_t S2PI_TryGetMutex(s2pi_slave_t slave);

/*!***************************************************************************
 * @brief   Releases the SPI interface mutex.
 *
 * @details Once the mutex is captured, only a single thread (the one that
 *          captured it) will call this release function, so there is no
 *          need for any test or thread safe barriers. Also there is no
 *          side effect of calling this function when the Mutex is not
 *          taken so this function can be really simple and doesn't need
 *          to return anything.
 *
 *          See #S2PI_TryGetMutex on more information and an example
 *          implementation for the single SPI interface case.
 *
 * @param   slave The specified S2PI slave. Note that the slave information is
 *                only required if multiple SPI instances are used in order to
 *                map to the correct SPI instance.
 *****************************************************************************/
void S2PI_ReleaseMutex(s2pi_slave_t slave);

/*!***************************************************************************
 * @brief   Transfers a single SPI frame asynchronously.
 *
 * @details Transfers a single SPI frame in asynchronous manner. The Tx data
 *          buffer is written to the device via the MOSI line.
 *          Optionally, the data on the MISO line is written to the provided
 *          Rx data buffer. If null, the read data is dismissed. Note that
 *          Rx and Tx buffer can be identical. I.e. the same buffer is used
 *          for writing and reading data. First, a byte is transmitted and
 *          the received byte overwrites the previously send value.
 *
 *          The transfer of a single frame requires to not toggle the chip
 *          select line to high in between the data frame. The maximum
 *          number of bytes transferred in a single SPI transfer is given by
 *          the data value register of the device, which is 396 data bytes
 *          plus a single address byte: 397 bytes.
 *
 *          An optional callback is invoked when the asynchronous transfer
 *          is finished. If the \p callback parameter is a null pointer,
 *          no callback is provided. Note that the provided buffer must not
 *          change while the transfer is ongoing.
 *
 *          Use the slave parameter to determine the corresponding slave via the
 *          given chip select line.
 *
 *          Usually, two distinct interrupts are required to handle the RX and
 *          TX ready events. The callback must be invoked from whichever
 *          interrupt comes after the SPI transfer has been finished. Note
 *          that new SPI transfers are invoked from within the callback function
 *          (i.e. from within the interrupt service routine of same priority).
 *
 * @param   slave The specified S2PI slave.
 * @param   txData The 8-bit values to write to the SPI bus MOSI line.
 * @param   rxData The 8-bit values received from the SPI bus MISO line
 *                   (pass a null pointer if the data don't need to be read).
 * @param   frameSize The number of 8-bit values to be sent/received.
 * @param   callback A callback function to be invoked when the transfer is
 *                     finished. Pass a null pointer if no callback is required.
 * @param   callbackData A pointer to a state that will be passed to the
 *                         callback. Pass a null pointer if not used.
 *
 * @return  Returns the \link #status_t status\endlink:
 *           - #STATUS_OK: Successfully invoked the transfer.
 *           - #ERROR_INVALID_ARGUMENT: An invalid parameter has been passed.
 *           - #ERROR_S2PI_INVALID_SLAVE: A wrong slave identifier is provided.
 *           - #STATUS_BUSY: An SPI transfer is already in progress. The
 *                           transfer was not started.
 *           - #STATUS_S2PI_GPIO_MODE: The module is in GPIO mode. The transfer
 *                                     was not started.
 *****************************************************************************/
status_t S2PI_TransferFrame(s2pi_slave_t slave,
			    uint8_t const *txData,
			    uint8_t *rxData,
			    size_t frameSize,
			    s2pi_callback_t callback,
			    void *callbackData);

/*!***************************************************************************
 * @brief   Terminates a currently ongoing asynchronous SPI transfer.
 *
 * @details When a callback is set for the current ongoing activity, it is
 *          invoked with the #ERROR_ABORTED error byte.
 *
 * @param   slave The specified S2PI slave. Note that the slave information is
 *                only required if multiple SPI instances are used in order to
 *                map to the correct SPI instance.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_Abort(s2pi_slave_t slave);

/*!***************************************************************************
 * @brief   Set a callback for the GPIO IRQ for a specified S2PI slave.
 *
 * @param   slave The specified S2PI slave.
 * @param   callback A callback function to be invoked when the specified
 *                     S2PI slave IRQ occurs. Pass a null pointer to disable
 *                     the callback.
 * @param   callbackData A pointer to a state that will be passed to the
 *                         callback. Pass a null pointer if not used.
 *
 * @return  Returns the \link #status_t status\endlink:
 *           - #STATUS_OK: Successfully installation of the callback.
 *           - #ERROR_S2PI_INVALID_SLAVE: A wrong slave identifier is provided.
 *****************************************************************************/
status_t S2PI_SetIrqCallback(s2pi_slave_t slave,
			     s2pi_irq_callback_t callback,
			     void *callbackData);

/*!***************************************************************************
 * @brief   Reads the current interrupt pending status of the IRQ pin.
 *
 * @details In order to keep a low priority for GPIO IRQs, the state of the
 *          IRQ pin must be read in order to reliable check for chip timeouts.
 *
 *          The execution of the interrupt service routine for the data-ready
 *          interrupt from the corresponding GPIO pin might be delayed due to
 *          priority issues. The delayed execution might disable the timeout
 *          for the eye-safety checker too late causing false error messages.
 *          In order to overcome the issue, the interrupt state of the IRQ
 *          GPIO input pin is read before raising a timeout error in order to
 *          check if the device has already finished and the IRQ is still
 *          pending to be executed!
 *
 *          Note: an easy implementation is to simply return the state of the
 *          IRQ line, i.e. 0 if there is a low input state and 1 if there is
 *          a high input state on the IRQ input pin. However, this
 *          implementation is not fully reliable since the GPIO interrupt
 *          (triggered on the falling edge) might be missed and the callback
 *          is never invoked while the IRQ line is correctly asserted to low
 *          state by the device. In that case, the API is waiting forever
 *          until the callback is invoked which might never happen. Therefore,
 *          it is better if the implementation checks the state of the IRQ
 *          pending status register or even combines both variations.

 * @param   slave The specified S2PI slave.
 *
 * @return  Returns 1U if the IRQ is NOT pending (pin is in high state) and
 *          0U if the IRQ is pending (pin is pulled to low state by the device).
 *****************************************************************************/
uint32_t S2PI_ReadIrqPin(s2pi_slave_t slave);

/*!***************************************************************************
 * @brief   Cycles the chip select line.
 *
 * @details In order to cancel the integration on the ASIC, a fast toggling
 *          of the chip select pin of the corresponding SPI slave is required.
 *          Therefore, this function toggles the CS from high to low and back.
 *          The SPI instance for the specified S2PI slave must be idle,
 *          otherwise the status #STATUS_BUSY is returned.
 *
 * @param   slave The specified S2PI slave.
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_CycleCsPin(s2pi_slave_t slave);

/*!*****************************************************************************
 * @brief   Captures the S2PI pins for GPIO usage.
 *
 * @details The SPI is disabled (module status: #STATUS_S2PI_GPIO_MODE) and the
 *          pins are configured for GPIO operation. The GPIO control must be
 *          release with the #S2PI_ReleaseGpioControl function in order to
 *          switch back to ordinary SPI functionality.
 *
 * @note    This function is only called during device initialization!
 *
 * @param   slave The specified S2PI slave. Note that the slave information is
 *                only required if multiple SPI instances are used in order to
 *                map to the correct SPI instance.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_CaptureGpioControl(s2pi_slave_t slave);

/*!*****************************************************************************
 * @brief   Releases the S2PI pins from GPIO usage and switches back to SPI mode.
 *
 * @details The GPIO pins are configured for SPI operation and the GPIO mode is
 *          left. Must be called if the pins are captured for GPIO operation via
 *          the #S2PI_CaptureGpioControl function.
 *
 * @note    This function is only called during device initialization!
 *
 * @param   slave The specified S2PI slave. Note that the slave information is
 *                only required if multiple SPI instances are used in order to
 *                map to the correct SPI instance.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_ReleaseGpioControl(s2pi_slave_t slave);

/*!*****************************************************************************
 * @brief   Writes the output for a specified SPI pin in GPIO mode.
 *
 * @details This function writes the value of an SPI pin if the SPI pins are
 *          captured for GPIO operation via the #S2PI_CaptureGpioControl previously.
 *
 * @note    Since some GPIO peripherals switch the GPIO pins very fast a delay
 *          must be added after each GBIO access (i.e. right before returning
 *          from the #S2PI_WriteGpioPin method) in order to decrease the baud
 *          rate of the software EEPROM protocol. Increase the delay if timing
 *          issues occur while reading the EERPOM. For example:
 *          Delay = 10 µsec => Baud Rate < 100 kHz
 *
 * @note    This function is only called during device initialization!
 *
 * @param   slave The specified S2PI slave.
 * @param   pin The specified S2PI pin.
 * @param   value The GPIO pin state to write (0 = low, 1 = high).
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_WriteGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t value);

/*!*****************************************************************************
 * @brief   Reads the input from a specified SPI pin in GPIO mode.
 *
 * @details This function reads the value of an SPI pin if the SPI pins are
 *          captured for GPIO operation via the #S2PI_CaptureGpioControl previously.
 *
 * @note    This function is only called during device initialization!
 *
 * @param   slave The specified S2PI slave.
 * @param   pin The specified S2PI pin.
 * @param   value The GPIO pin state to read (0 = low, GND level, 1 = high, VCC level).
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_ReadGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t *value);

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // ARGUS_S2PI_H
