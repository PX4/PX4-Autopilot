/*************************************************************************//**
 * @file
 * @brief   Tests for the AFBR-S50 API hardware abstraction layer.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef ARGUS_HAL_TEST_H
#define ARGUS_HAL_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_test HAL Self Test
 * @ingroup     argus
 *
 * @brief       A test module to verify implementation of the HAL.
 *
 * @details     A series of automated tests that can be executed on the target
 *              platform in order to verify the implementation of the HAL that
 *              are required by the API.
 *
 *              See #Argus_VerifyHALImplementation for a detailed documentation.
 *
 * @addtogroup  argus_test
 * @{
 *****************************************************************************/

#include "argus.h"

/*!***************************************************************************
 * @brief   Version number of the HAL Self Test.
 *
 * @details Changes:
 *          * v1.0:
 *              - Initial release.
 *          * v1.1:
 *              - Added additional print output.
 *              - Increased tolerance for timer test to 3%.
 *              - Fixed callback issue by disabling it after IRQ test.
 *          * v1.2:
 *              - Added PIT test cases.
 *          * v1.3:
 *              - Added test case for SPI maximum data transfer size.
 *              - Added tests for SPI transfers invoked from all IRQ callbacks.
 *              - Added verification of first PIT event occurrence.
 *              - Relaxed PIT pass conditions (0.1% -> 0.5%)
 *          * v1.4:
 *              - Adopted to new multi-device HAL interface of API v1.4.4 release.
 *              - Added verification of SPI callback invocation.
 *              - Updated GPIO interrupt test to verify if delayed interrupt
 *                pending states can be detected via #S2PI_ReadIrqPin.
 *
 *****************************************************************************/
#define HAL_TEST_VERSION "v1.4"

/*!***************************************************************************
 * @brief   Executes a series of tests in order to verify the HAL implementation.
 *
 * @details A series of automated tests are executed on the target platform in
 *          order to verify the implementation of the HAL that are required by
 *          the API.
 *
 *          Each test will write an error description via the print (i.e. UART)
 *          function that shows what went wrong. Also an corresponding status is
 *          returned in case no print functionality is available.
 *
 *          The following tests are executed:
 *
 *          **1) Timer Plausibility Test:**
 *
 *          Rudimentary tests of the lifetime counter (LTC) implementation.
 *          This verifies that the LTC is running by checking if the returned
 *          values of two consecutive calls to the #Timer_GetCounterValue
 *          function are ascending. An artificial delay using the NOP operation
 *          is induced such that the timer is not read to fast.
 *
 *          **2) Timer Wraparound Test:**
 *
 *          The LTC values must wrap from 999999 µs to 0 µs and increase the
 *          seconds counter accordingly. This test verifies the correct wrapping
 *          by consecutively calling the #Timer_GetCounterValue function until
 *          at least 2 wraparound events have been occurred.
 *
 *          **3) SPI Connection Test:**
 *
 *          This test verifies the basic functionality of the SPI interface.
 *          The test utilizes the devices laser pattern register, which can
 *          be freely programmed by any 128-bit pattern. Thus, it writes a byte
 *          sequence and reads back the written values on the consecutive SPI
 *          access.
 *
 *          **4) SPI Maximum Data Length Test**:
 *
 *          This test verifies the maximum data transfer length of the SPI
 *          interface. The test sends and receives up to 396 data bytes plus
 *          a single address byte over the SPI interface and verifies that no
 *          data get lost.
 *
 *          **5) SPI Interrupt Test:**
 *
 *          This test verifies the correct implementation of the device
 *          integration finished interrupt callback. Therefore it configures
 *          the device with a minimal setup to run a pseudo measurement that
 *          does not emit any laser light.
 *
 *          Note that this test does verify the GPIO interrupt that occurs
 *          whenever the device has finished the integration/measurement and
 *          new data is waiting to be read from the device. This does not test
 *          the interrupt that is triggered when the SPI transfer has finished.
 *
 *          The data ready interrupt implies two S2PI layer functions that
 *          are tested in this test: The #S2PI_SetIrqCallback function installs
 *          a callback function that is invoked whenever the IRQ occurs.
 *          The IRQ can be delayed due to higher priority task, e.g. from the
 *          user code. It is essential for the laser safety timeout algorithm
 *          to determine the device ready signal as fast as possible, another
 *          method is implemented to read if the IRQ is pending but the
 *          callback has not been reset yet. This is what the #S2PI_ReadIrqPin
 *          function is for.
 *
 *          **6) GPIO Mode Test:**
 *
 *          This test verifies the GPIO mode of the S2PI HAL module. This is
 *          done by leveraging the EEPROM readout sequence that accesses the
 *          devices EEPROM via a software protocol that depends on the GPIO
 *          mode.
 *
 *          This the requires several steps, most of them are already verified
 *          in previous tests:
 *
 *          - Basic device configuration and enable EEPROM.
 *          - Read EERPOM via GPIO mode and apply Hamming weight.
 *          - Repeat several times (to eliminate random readout issues).
 *          - Decode the EEPROM (using EEPROM_Decode in argus_cal_eeprom.c).
 *          - Check if Module Number and Chip ID is not 0.
 *
 *          **7) Timer Test for Lifetime Counter:**
 *
 *          The test verifies the lifetime counter timer HAL implementation by
 *          comparing the timings to the AFBR-S50 device as a reference.
 *          Therefore several measurement are executed on the device, each with
 *          a different averaging sample count. The elapsed time increases
 *          linearly with the number of averaging samples. In order to remove
 *          the time for software/setup, a linear regression fit is applied to
 *          the measurement results and only the slope is considered for the
 *          result. A delta of 102.4 microseconds per sample is expected.
 *          If the measured delta per sample is within an specified error range,
 *          the timer implementation is considered correct.
 *
 *          **8) Timer Test for Periodic Interrupt Timer (optional):**
 *
 *          The test verifies the correct implementation of the periodic
 *          interrupt timer (PIT). It sets different intervals and waits for
 *          a certain number of interrupts to happen. Each interrupt event
 *          is counted and the time between the first and the last interrupt
 *          is measured. Finally, the measured interval is compared to the
 *          expectations.
 *
 *          Note that this test is only executed if the PIT is actually
 *          implemented. Otherwise, the test is skipped.
 *
 *          **9) SPI Transfer from Interrupt Callback Test:**
 *
 *          The test verifies that the #S2PI_TransferFrame method of the
 *          S2PI layer can be invoked from a interrupt callback function too.
 *          Thus, it repeats the S2PI Connection Test but this time from
 *          different interrupt callback functions:
 *
 *          - SPI Callback: The first transfer is invoked from thread level,
 *            the second transfer is invoke from the SPI interrupt callback
 *            function.
 *
 *          - GPIO Callback: The device is setup to trigger an GPIO interrupt
 *            (see also the SPI Interrupt Test). The corresponding GPIO
 *            interrupt callback function will trigger the first transfer while
 *            the second one is triggered from the SPI callback function.
 *
 *          - PIT Callback (optional): This test is only executed optional if
 *            the PIT interface is implemented. The test sequence is the same
 *            as for the GPIO callback, but the first transfer is triggered
 *            from the PIT callback function.
 *
 * @note    See #HAL_TEST_VERSION for a version history and change log of
 *          the HAL self tests.
 *
 * @param   spi_slave The SPI hardware slave, i.e. the specified CS and IRQ
 *                    lines. This is actually just a number that is passed
 *                    to the SPI interface to distinct for multiple SPI slave
 *                    devices. Note that the slave must be not equal to 0,
 *                    since is reserved for error handling.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Argus_VerifyHALImplementation(s2pi_slave_t spi_slave);

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_HAL_TEST_H */
