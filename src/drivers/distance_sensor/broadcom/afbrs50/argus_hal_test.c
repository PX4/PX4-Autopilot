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

/*!***************************************************************************
 * @addtogroup  argus_test
 * @{
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "argus_hal_test.h"

#include "platform/argus_print.h"
#include "platform/argus_s2pi.h"
#include "platform/argus_timer.h"
#include "platform/argus_nvm.h"
#include "platform/argus_irq.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! An error log message via #print function. */
#define error_log(fmt, ...) print("ERROR: " fmt "\n", ##__VA_ARGS__)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t VerifyHALImplementation(s2pi_slave_t spi_slave);

static status_t TimerPlausibilityTest(void);
static status_t TimerWraparoundTest(void);
static status_t SpiConnectionTest(s2pi_slave_t slave);
static status_t SpiMaxLengthTest(s2pi_slave_t slave);
//static status_t SpiInterruptTest(s2pi_slave_t slave);
static status_t GpioInterruptTest(s2pi_slave_t slave);
static status_t GpioModeTest(s2pi_slave_t slave);
static status_t TimerTest(s2pi_slave_t slave);
static status_t PITTest(void);
static status_t SpiTransferFromInterruptTest(s2pi_slave_t slave);

static status_t CheckTimerCounterValues(uint32_t hct, uint32_t lct);
static status_t SPITransferSync(s2pi_slave_t slave, uint8_t *data, size_t size);
static status_t ConfigureDevice(s2pi_slave_t slave, int8_t rcoTrim);
static status_t TriggerMeasurement(s2pi_slave_t slave, uint16_t samples, s2pi_callback_t callback, void *callbackData);
static status_t ReadEEPROM(s2pi_slave_t slave, uint8_t *eeprom);
static status_t ReadRcoTrim(s2pi_slave_t slave, int8_t *RcoTrim);
static status_t RunMeasurement(s2pi_slave_t slave, uint16_t samples);
static status_t RunPITTest(uint32_t exp_dt_us, uint32_t n);

static void PIT_Callback(void *param);
static void GPIO_Callback(void *param);

/// @cond EXTERN
extern uint32_t EEPROM_ReadChipId(uint8_t const *eeprom);
extern uint8_t EEPROM_ReadModule(uint8_t const *eeprom);
extern status_t EEPROM_Read(s2pi_slave_t slave, uint8_t address, uint8_t *data);
extern uint8_t hamming_decode(uint8_t const *code, uint8_t *data);
/// @endcond

/******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t Argus_VerifyHALImplementation(s2pi_slave_t spi_slave)
{
	print("########################################################\n");
	print("#   Running HAL Verification Test - " HAL_TEST_VERSION "\n");
	print("########################################################\n");
	print("- SPI Slave: %d \n\n", spi_slave);

	const status_t status = VerifyHALImplementation(spi_slave);

	print("########################################################\n");

	if (status != STATUS_OK) {
		print("#   FAIL: HAL Verification Test finished with error %d!\n", status);

	} else {
		print("#   PASS: HAL Verification Test finished successfully!\n");
	}

	print("########################################################\n\n");

	return status;
}

/*!***************************************************************************
 * @brief   Executes a series of tests in order to verify the HAL implementation.
 *
 * @details See #Argus_VerifyHALImplementation for details.
 *
 * @param   spi_slave The SPI hardware slave.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
static status_t VerifyHALImplementation(s2pi_slave_t spi_slave)
{
	status_t status = STATUS_OK;

	print("1 > Timer Plausibility Test\n");
	status = TimerPlausibilityTest();

	if (status != STATUS_OK) { return status; }

	print("1 > PASS\n\n");

	print("2 > Timer Wraparound Test\n");
	status = TimerWraparoundTest();

	if (status != STATUS_OK) { return status; }

	print("2 > PASS\n\n");

	print("3 > SPI Connection Test\n");
	status = SpiConnectionTest(spi_slave);

	if (status != STATUS_OK) { return status; }

	print("3 > PASS\n\n");

	print("4 > SPI Maximum Data Length Test\n");
	status = SpiMaxLengthTest(spi_slave);

	if (status != STATUS_OK) { return status; }

	print("4 > PASS\n\n");

	print("5 > GPIO Interrupt Test\n");
	status = GpioInterruptTest(spi_slave);

	if (status != STATUS_OK) { return status; }

	print("5 > PASS\n\n");

	print("6 > GPIO Mode Test\n");
	status = GpioModeTest(spi_slave);

	if (status != STATUS_OK) { return status; }

	print("6 > PASS\n\n");

	print("7 > Lifetime Counter Timer (LTC) Test\n");
	status = TimerTest(spi_slave);

	if (status != STATUS_OK) { return status; }

	print("7 > PASS\n\n");

	print("8 > Periodic Interrupt Timer (PIT) Test\n");
	status = PITTest();

	if (status == ERROR_NOT_IMPLEMENTED) {
		print("8 > SKIPPED (PIT is not implemented)\n\n");

	} else {
		if (status != STATUS_OK) { return status; }

		print("8 > PASS\n\n");
	}

	print("9 > SPI Interrupt Test\n");
	status = SpiTransferFromInterruptTest(spi_slave);

	if (status != STATUS_OK) { return status; }

	print("9 > PASS\n\n");

	return status;
}

/*!***************************************************************************
 * @brief   Checks the validity of timer counter values.
 *
 * @details This verifies that the counter values returned from the
 *          #Timer_GetCounterValue function are valid. This means, the low
 *          counter value \p lct is within 0 and 999999 µs.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_FAIL on failure (check the error log for more information).
 *****************************************************************************/
static status_t CheckTimerCounterValues(uint32_t hct, uint32_t lct)
{
	if (lct > 999999) {
		error_log("Timer plausibility check:\n"
			  "The parameter \"lct\" of Timer_GetCounterValue() must always "
			  "be within 0 and 999999.\n"
			  "Current Values: hct = %d, lct = %d", hct, lct);
		return ERROR_FAIL;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Plausibility Test for Timer HAL Implementation.
 *
 * @details Rudimentary tests the lifetime counter (LTC) implementation.
 *          This verifies that the LTC is running by checking if the returned
 *          values of two consecutive calls to the #Timer_GetCounterValue
 *          function are ascending. An artificial delay using the NOP operation
 *          is induced such that the timer is not read to fast.
 *
 * @warning If using an ultra-fast processor with a rather low timer granularity,
 *          the test may fail! In this case, it could help to increase the delay
 *          by increasing the for-loop exit criteria.
 *
 * @warning This test does not test yet verify if the timing is correct at all!
 *          This it done in later test...
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_FAIL on failure (check the error log for more information).
 *****************************************************************************/
static status_t TimerPlausibilityTest(void)
{
	uint32_t hct0 = 0;
	uint32_t lct0 = 0;
	uint32_t hct1 = 0;
	uint32_t lct1 = 0;

	/* Get some start values */
	Timer_GetCounterValue(&hct0, &lct0);

	/* Check max value is not exceeded for LCT timer (us) */
	status_t status = CheckTimerCounterValues(hct0, lct0);

	if (status != STATUS_OK) { return status; }

	/* Adding a delay. Depending on MCU speed, this takes any time.
	 * However, the Timer should be able to solve this on any MCU. */
	for (volatile uint32_t i = 0; i < 100000; ++i) { __asm("nop"); }

	/* Get new timer value and verify some time has elapsed. */
	Timer_GetCounterValue(&hct1, &lct1);

	/* Check max value is not exceeded for LCT timer (us) */
	status = CheckTimerCounterValues(hct1, lct1);

	if (status != STATUS_OK) { return status; }

	/* Either the hct value must have been increased or the lct value if the hct
	 * value is still the same. */
	if (!((hct1 > hct0) || ((hct1 == hct0) && (lct1 > lct0)))) {
		error_log("Timer plausibility check: the elapsed time could not be "
			  "measured with the Timer_GetCounterValue() function; no time "
			  "has elapsed!\n"
			  "The delay was induced by the following code:\n"
			  "for (volatile uint32_t i = 0; i < 100000; ++i) __asm(\"nop\");\n",
			  "Current Values: hct0 = %d, lct0 = %d, hct1 = %d, lct1 = %d",
			  hct0, lct0, hct1, lct1);
		return ERROR_FAIL;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Wraparound Test for the Timer HAL Implementation.
 *
 * @details The LTC values must wrap from 999999 µs to 0 µs and increase the
 *          seconds counter accordingly. This test verifies the correct wrapping
 *          by consecutively calling the #Timer_GetCounterValue function until
 *          at least 2 wraparound events have been occurred.
 *
 * @note    This test requires the timer to basically run and return ascending
 *          values. Also, if the timer is too slow, this may take very long!
 *          Usually, the test takes 2 seconds, since 2 wraparound events are
 *          verified.
 *
 * @warning This test does not test yet verify if the timing is correct at all!
 *          This it done in later test...
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_FAIL on failure (check the error log for more information).
 *****************************************************************************/
static status_t TimerWraparoundTest(void)
{
	/* Test parameter configuration: *****************************************/
	const uint8_t n = 2;        // The number of wraparounds to test.
	/*************************************************************************/

	uint32_t hct0 = 0;
	uint32_t lct0 = 0;
	uint32_t hct1 = 0;
	uint32_t lct1 = 0;

	/* Get some start values. */
	Timer_GetCounterValue(&hct0, &lct0);

	/* Check max value is not exceeded for LCT timer (us) */
	status_t status = CheckTimerCounterValues(hct0, lct0);

	if (status != STATUS_OK) { return status; }

	/* Set end after 2 seconds, i.e. 2 wrap around events. */
	uint32_t hct2 = hct0 + n;
	uint32_t lct2 = lct0;

	/* Periodically read timer values. From previous tests we
	 * already know the timer value is increasing. */
	while (hct0 < hct2 || lct0 < lct2) {
		/* add counter a , which is increasing by +1, 1000000 or 1000,
		 * different MCU different times get stuck for hard code value */
		Timer_GetCounterValue(&hct1, &lct1);

		/* Check max value is not exceeded for LCT timer (us) */
		status = CheckTimerCounterValues(hct0, lct0);

		if (status != STATUS_OK) { return status; }

		/* Testing if calls to Timer_GetCounterValue are equal or increasing.
		 * Also testing if wraparound is correctly handled.
		 * Assumption here is that two sequential calls to the get functions are
		 * only a few µs apart! I.e. if hct wraps, the new lct must be smaller
		 * than previous one. */
		if (!(((hct1 == hct0 + 1) && (lct1 < lct0))
		      || ((hct1 == hct0) && (lct1 >= lct0)))) {
			error_log("Timer plausibility check: the wraparound of \"lct\" or "
				  "\"hct\" parameters of the Timer_GetCounterValue() "
				  "function was not handled correctly!\n"
				  "Current Values: hct0 = %d, lct0 = %d, hct1 = %d, lct1 = %d",
				  hct0, lct0, hct1, lct1);
			return ERROR_FAIL;
		}

		hct0 = hct1;
		lct0 = lct1;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   SPI interrupt callback function for the SPI transfer interrupt test.
 *
 * @details The interrupt callback is invoked from the S2PI module upon
 *          finishing the SPI transfer. The callback is used by the
 *          #SPITransferSync helper function to retrieve the status of the
 *          SPI transfer.
 *
 * @param   status The S2PI module status passed to the callback.
 * @param   param The abstract interrupt callback parameter.
 *
 * @return  Returns #STATUS_OK.
 *****************************************************************************/
static status_t SpiTransferInterruptCallback(status_t status, void *param)
{
	*((status_t *)param) = status;
	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Helper function for transfer data to SPI in blocking mode.
 *
 * @details Calls the #S2PI_TransferFrame function and waits until the transfer
 *          has been finished by checking the #S2PI_GetStatus return code to
 *          become #STATUS_IDLE (or #STATUS_OK).
 *
 * @warning The test utilizes already the timer HAL in order to implement a
 *          rudimentary timeout. However, at this time, only some basic
 *          plausibility checks are performed on the timer HAL. I.e. if there
 *          is an issue in the time HAL, e.g. too fast or too slow time
 *          counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 *          one also needs to verify the timer HAL, especially the
 *          #Timer_GetCounterValue function.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param   data The data array to be transferred.
 * @param   size The size of the data array to be transferred.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if the operation did not finished within a specified
 *            time (check also timer HAL implementation).
 *          - The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 *            return any negative status.
 *****************************************************************************/
static status_t SPITransferSync(s2pi_slave_t slave, uint8_t *data, size_t size)
{
	/* Test parameter configuration: *****************************************/
	const uint32_t timeout_ms = 100;    // The transfer timeout in ms.
	/*************************************************************************/

	/* The status will be changed in the SPI callback. */
	volatile status_t callbackStatus = STATUS_BUSY;

	status_t status = S2PI_TransferFrame(slave, data, data, size,
					     SpiTransferInterruptCallback,
					     (void *)&callbackStatus);

	if (status != STATUS_OK) {
		error_log("SPI transfer failed! The call to S2PI_TransferFrame "
			  "yielded error code: %d",
			  status);
		return status;
	}

	/* Wait until the transfer is finished using a timeout.
	 * Note: this already utilizes the timer HAL. So we might
	 * need to test the timer before the SPI connection test. */
	ltc_t start;
	Time_GetNow(&start);

	do {
		status = S2PI_GetStatus(slave);

		if (status < STATUS_OK) {
			error_log("SPI transfer failed! The call to S2PI_GetStatus "
				  "yielded error code: %d", status);
			S2PI_Abort(slave);
			return status;
		}

		if (Time_CheckTimeoutMSec(&start, timeout_ms)) {
			error_log("SPI transfer failed! The operation did not finished "
				  "within %d ms. This may also be caused by an invalid "
				  "timer implementation!", timeout_ms);
			return ERROR_TIMEOUT;
		}
	} while (status == STATUS_BUSY);

	if (callbackStatus != STATUS_OK) {
		error_log("Invocation of the SPI callback failed! The SPI transfer "
			  "callback yielded error code: %d", callbackStatus);
		return callbackStatus;
	}

	return status;
}

/*!***************************************************************************
 * @brief   SPI Connection Test for S2PI HAL Implementation.
 *
 * @details This test verifies the basic functionality of the SPI interface.
 *
 *          The test utilizes the devices laser pattern register, which can
 *          be freely programmed by any 128-bit pattern. Thus, it writes a byte
 *          sequence and reads back the written values on the consecutive SPI
 *          access.
 *
 *          Note: The test verifies the SPI interface transfer functionality
 *          in blocking mode and also verifies the interrupt callback.
 *          In order to wait for the transfer to finish, it reads the S2PI
 *          status in a loop. If the status does not change to #STATUS_IDLE,
 *          the test will fail with an #ERROR_TIMEOUT. Finally, the test will
 *          verify the SPI transfer callback status.
 *
 * @warning The test utilizes already the timer HAL in order to implement a
 *          rudimentary timeout. However, at this time, only some basic
 *          plausibility checks are performed on the timer HAL. I.e. if there
 *          is an issue in the time HAL, e.g. too fast or too slow time
 *          counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 *          one also needs to verify the timer HAL, especially the
 *          #Timer_GetCounterValue function.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if the operation did not finished within a specified
 *            time (check also timer HAL implementation).
 *          - #ERROR_FAIL if the device access failed and the read data did not
 *            match the expected values.
 *          - The S2PI layer error code if #S2PI_TransferFrame, #S2PI_GetStatus
 *            or the SPI callback yield any negative status.
 *****************************************************************************/
static status_t SpiConnectionTest(s2pi_slave_t slave)
{
	status_t status = STATUS_OK;
	uint8_t data[17U] = { 0 };

	/* Transfer a pattern to the register */
	data[0] = 0x04; // Laser Pattern Register Address

	for (uint8_t i = 1; i < 17U; ++i) { data[i] = i; }

	status = SPITransferSync(slave, data, 17U);

	if (status != STATUS_OK) {
		error_log("SPI connection test failed!");
		return status;
	}

	/* Clear the laser pattern and read back previous values. */
	data[0] = 0x04; // Laser Pattern Register Address

	for (uint8_t i = 1; i < 17U; ++i) { data[i] = 0; }

	status = SPITransferSync(slave, data, 17U);

	if (status != STATUS_OK) {
		error_log("SPI connection test failed!");
		return status;
	}

	/* Verify the read pattern. */
	for (uint8_t i = 1; i < 17U; ++i) {
		if (data[i] != i) {
			error_log("SPI connection test failed!\n"
				  "Verification of read data is invalid!\n"
				  "read_data[%d] = %d, but expected was %d",
				  i, data[i], i);
			return ERROR_FAIL;
		}
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Maximum SPI Data Size Test for S2PI HAL Implementation.
 *
 * @details This test verifies the maximum data transfer length of the SPI
 *          interface. The test sends and receives up to 396 data bytes plus
 *          a single address byte over the SPI interface and verifies that no
 *          data get lost.
 *
 *          The test utilizes the channel select register which is 3 bytes plus
 *          address. This register can be repeatedly written with any pattern
 *          using the DMA mode. The register is written 100 times in a row
 *          to verify that long data frames with up to 400 bytes can be
 *          transmitted.
 *
 *          Note that this test was motivated by an invalid implementation that
 *          used uint8_t type for the frame length in the #S2PI_TransferFrame
 *          function instead of an uint16_t value. This resulted in a maximum
 *          data length of 141 bytes (367 & 0xFF = 141) when reading the
 *          data value register.
 *
 * @warning The test utilizes already the timer HAL in order to implement a
 *          rudimentary timeout. However, at this time, only some basic
 *          plausibility checks are performed on the timer HAL. I.e. if there
 *          is an issue in the time HAL, e.g. too fast or too slow time
 *          counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 *          one also needs to verify the timer HAL, especially the
 *          #Timer_GetCounterValue function.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if the operation did not finished within a specified
 *            time (check also timer HAL implementation).
 *          - #ERROR_FAIL if the device access failed and the read data did not
 *            match the expected values.
 *          - The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 *            return any negative status.
 *****************************************************************************/
static status_t SpiMaxLengthTest(s2pi_slave_t slave)
{
	status_t status = STATUS_OK;
	uint8_t data[400U] = { 0 };

	/* Setup device (enable DMA mode). */
	data[0] = 0x10; data[1] = 0x12;
	status = SPITransferSync(slave, data, 2);

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	data[0] = 0x12; data[1] = 0x00; data[2] = 0x2B;
	status = SPITransferSync(slave, data, 3);

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	/* Transfer a pattern to the register */
	for (uint32_t i = 0; i < sizeof(data); i += 4) {
		data[i + 0] = 0x1E;             // Address
		data[i + 1] = (uint8_t)i;       // Random Data Byte 0
		data[i + 2] = (uint8_t)(i + 1); // Random Data Byte 1
		data[i + 3] = (uint8_t)(i * 2); // Random Data Byte 2
	}

	status = SPITransferSync(slave, data, sizeof(data));

	if (status != STATUS_OK) {
		error_log("SPI maximum data length test failed!");
		return status;
	}

	/* Repeat ... */
	for (uint32_t i = 0; i < sizeof(data); i += 4) {
		data[i + 0] = 0x1E;             // Address
		data[i + 1] = (uint8_t)i;       // Random Data Byte 0
		data[i + 2] = (uint8_t)(i + 1); // Random Data Byte 1
		data[i + 3] = (uint8_t)(i * 2); // Random Data Byte 2
	}

	status = SPITransferSync(slave, data, sizeof(data));

	if (status != STATUS_OK) {
		error_log("SPI maximum data length test failed!");
		return status;
	}

	/* Verify the read pattern; skip all address bytes. */
	for (uint32_t i = 0; i < sizeof(data); i += 4) {
		uint32_t j = (i + 4) % sizeof(data);

		if (data[j + 1] != (uint8_t)i
		    || data[j + 2] != (uint8_t)(i + 1)
		    || data[j + 3] != (uint8_t)(i * 2)) {
			error_log("SPI maximum data length test failed!\n"
				  "Verification of read data is invalid at byte %d!\n"
				  " - expected: 0x%02X%02X%02X\n"
				  " - actual:   0x%02X%02X%02X",
				  i, (uint8_t)i, (uint8_t)(i + 1), (uint8_t)(i * 2),
				  data[j + 1], data[j + 2], data[j + 3]);
			return ERROR_FAIL;
		}
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Configures the device with a bare minimum setup to run the tests.
 *
 * @details This function applies a number of configuration values to the
 *          device, such that a pseudo measurement w/o laser output can be
 *          performed.
 *
 *          A \p rcoTrim parameter can be passed to adjust the actual clock
 *          setup.
 *
 * @warning The test utilizes already the timer HAL in order to implement a
 *          rudimentary timeout. However, at this time, only some basic
 *          plausibility checks are performed on the timer HAL. I.e. if there
 *          is an issue in the time HAL, e.g. too fast or too slow time
 *          counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 *          one also needs to verify the timer HAL, especially the
 *          #Timer_GetCounterValue function.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param   rcoTrim The RCO Trimming value added to the nominal RCO register
 *                  value. Pass 0 if no fine tuning is required.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if the SPI operation did not finished within a
 *            specified time (check also timer HAL implementation).
 *          - The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 *            return any negative status.
 *****************************************************************************/
static status_t ConfigureDevice(s2pi_slave_t slave, int8_t rcoTrim)
{
	/* Setup Device and Trigger Measurement. */
	assert(rcoTrim >= -34 && rcoTrim < 0x3F - 34);
	const uint16_t v = (uint16_t)(0x0010U | (((uint16_t)(34 + rcoTrim) & 0x3F) << 6U));
	uint8_t d1[] = { 0x14U, (uint8_t)(v >> 8U), v & 0xFFU, 0x21U };
	status_t status = SPITransferSync(slave, d1, sizeof(d1));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d2[] = { 0x16, 0x7F, 0xFF, 0x7F, 0xE9 };
	status = SPITransferSync(slave, d2, sizeof(d2));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d3[] = { 0x18, 0x00, 0x00, 0x03 };
	status = SPITransferSync(slave, d3, sizeof(d3));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d4[] = { 0x10, 0x12 };
	status = SPITransferSync(slave, d4, sizeof(d4));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d5[] = { 0x12, 0x00, 0x2B };
	status = SPITransferSync(slave, d5, sizeof(d5));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d6[] = { 0x08, 0x04, 0x84, 0x10 };
	status = SPITransferSync(slave, d6, sizeof(d6));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d7[] = { 0x0A, 0xFE, 0x51, 0x0F, 0x05 };
	status = SPITransferSync(slave, d7, sizeof(d7));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d8[] = { 0x0C, 0x00, 0x00, 0x00 };
	status = SPITransferSync(slave, d8, sizeof(d8));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d9[] = { 0x1E, 0x00, 0x00, 0x00 };
	status = SPITransferSync(slave, d9, sizeof(d9));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d10[] = { 0x20, 0x01, 0xFF, 0xFF };
	status = SPITransferSync(slave, d10, sizeof(d10));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	uint8_t d11[] = { 0x22, 0xFF, 0xFF, 0x04 };
	status = SPITransferSync(slave, d11, sizeof(d11));

	if (status != STATUS_OK) {
		error_log("Device configuration failed!");
		return status;
	}

	return status;
}

/*!***************************************************************************
 * @brief   Triggers a measurement on the device with specified sample count.
 *
 * @details The function triggers a measurement cycle on the device. A
 *          \p sample count can be specified to setup individual number of
 *          digital averaging.
 *
 *          The measurement in triggered asynchronously without waiting
 *          for any event to finish.
 *
 * @warning The test utilizes already the timer HAL in order to implement a
 *          rudimentary timeout. However, at this time, only some basic
 *          plausibility checks are performed on the timer HAL. I.e. if there
 *          is an issue in the time HAL, e.g. too fast or too slow time
 *          counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 *          one also needs to verify the timer HAL, especially the
 *          #Timer_GetCounterValue function.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param   samples The specified number of averaging samples for the measurement.
 * @param   callback An optional SPI callback.
 * @param   callbackData The optional callback data parameter.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if the operation did not finished within a specified
 *            time (check also timer HAL implementation).
 *          - The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 *            return any negative status.
 *****************************************************************************/
static status_t TriggerMeasurement(s2pi_slave_t slave, uint16_t samples,
				   s2pi_callback_t callback, void *callbackData)
{
	// samples is zero based, i.e. writing 0 yields 1 sample
	samples = samples > 0 ? samples - 1 : samples;
	const uint16_t v = (uint16_t)(0x8000U | ((samples & 0x03FFU) << 5U));

	// data is static as the transfer is asynchronous and the buffer must persist.
	static uint8_t data[] = { 0x1CU, 0x00U, 0x00U };
	data[0] = 0x1CU;
	data[1] = (uint8_t)(v >> 8U);
	data[2] = v & 0xFFU;

	status_t status = S2PI_TransferFrame(slave, data, data, sizeof(data),
					     callback, callbackData);

	if (status != STATUS_OK) {
		error_log("SPI transfer failed to trigger measurements! "
			  "The call to S2PI_TransferFrame yielded error code: %d",
			  status);
		return status;
	}

	return status;
}

/*!***************************************************************************
 * @brief   Data structure for the GPIO interrupt test.
 *
 * @details Contains data that is required by the GPIO interrupt test.
 *****************************************************************************/
typedef struct gpio_data_t {
	/* The S2PI slave parameter passed to the S2PI HAL functions. */
	s2pi_slave_t Slave;

	/* The callback status. */
	volatile status_t Status;

	/* The GPIO timeout in milliseconds. */
	uint32_t Timeout_ms;

	/* A counter to determine how often the callback is invoked. */
	volatile uint32_t CallbackInvoked;

	/* The return value of the #S2PI_ReadIrqPin function. */
	volatile uint32_t ReadIrqPinValue;

} gpio_data_t;

/*!***************************************************************************
 * @brief   The IRQ callback dedicated to the #GpioInterruptTest.
 *
 * @details The callback is invoked by the API when the device GPIO IRQ is
 *          pending after a measurement has been executed and data is ready to
 *          be read from the device.
 *
 * @param   param The abstract pointer to the boolean value that determines if
 *                the callback is invoked.
 *****************************************************************************/
static void GPIO_Callback(void *param)
{
	if (param == NULL) {
		error_log("GPIO interrupt test failed: callback parameter \"param\" was NULL!");
		return;
	}

	gpio_data_t *data = (gpio_data_t *)param;
	data->CallbackInvoked = 1;
}

/*!***************************************************************************
 * @brief   The SPI transfer callback dedicated to the #GpioInterruptTest.
 *
 * @details The callback is invoked by the S2PI layer when the SPI transfer
 *          finished IRQ is invoked. The callback is used to simulate a
 *          deferred GPIO interrupt by locking the interrupts until the
 *          #S2PI_ReadIrqPin detects an GPIO interrupt pending state and
 *          returns 0.
 *
 * @param   status The status of the SPI transfer.
 * @param   param The abstract pointer to the boolean value that determines if
 *                the callback is invoked.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_FAIL if the \p param parameter is NULL.
 *          - #ERROR_TIMEOUT if the #S2PI_ReadIrqPin does not return 0 after
 *            a specified time (check also timer HAL implementation).
 *          - The S2PI layer error code that may be received from the S2PI
 *            module via the \p status parameter.
 *****************************************************************************/
static status_t GPIO_SPI_Callback(status_t status, void *param)
{
	IRQ_LOCK(); // prevents GPIO interrupt to preempt if set to higher priority.

	if (param == NULL) {
		IRQ_UNLOCK();
		error_log("GPIO interrupt test failed: callback parameter \"param\" was NULL!");
		return ERROR_FAIL;
	}

	gpio_data_t *data = (gpio_data_t *)param;

	if (status != STATUS_OK) {
		IRQ_UNLOCK();
		error_log("GPIO interrupt test failed: callback parameter \"status\" was %d!",
			  status);
		data->Status = status;
		return status;
	}

	/* The S2PI_ReadIrqPin must correctly return the GPIO IRQ state if the GPIO
	 * interrupt is pending but deferred due to any higher priority or critical
	 * sections. Therefore, the SPI callback with the #IRQ_LOCK/#IRQ_UNLOCK is
	 * used to delay the GPIO callback and test the #S2PI_ReadIrqPin function.
	 *
	 * The purpose is to simulate a delayed GPIO interrupt that can in the
	 * production code happen due to any higher priority interrupts (such as
	 * the SPI interrupt in this test). In those cases, the API relies on the
	 * #S2PI_ReadIrqPin method to obtain if the device has finished in time and
	 * the interrupt is already pending. Otherwise, it would fail with an
	 * timeout due to the deferred GPIO interrupt callback event. */

	ltc_t start;
	Time_GetNow(&start);
	data->ReadIrqPinValue = S2PI_ReadIrqPin(data->Slave);

	while (data->ReadIrqPinValue) {
		if (Time_CheckTimeoutMSec(&start, data->Timeout_ms)) {
			IRQ_UNLOCK();
			error_log("GPIO interrupt test failed! The IRQ pin did not assert "
				  "to low state when reading from the IRQ callback. "
				  "Elapsed %d ms.", data->Timeout_ms);
			data->Status = ERROR_TIMEOUT;
			return ERROR_TIMEOUT;
		}

		data->ReadIrqPinValue = S2PI_ReadIrqPin(data->Slave);
	}

	IRQ_UNLOCK();
	data->Status = STATUS_OK;
	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   SPI Interrupt Test for S2PI HAL Implementation.
 *
 * @details This test verifies the correct implementation of the device
 *          integration finished interrupt callback, a.k.a. the GPIO interrupt.
 *          Therefore it configures the device with a minimal setup to run a
 *          pseudo measurement that does not emit any laser light but triggers
 *          an GPIO interrupt once finished.
 *
 *          The data ready interrupt implies two S2PI layer functions that
 *          are tested in this test: The #S2PI_SetIrqCallback function installs
 *          a callback function that is invoked whenever the IRQ occurs and
 *          the #S2PI_ReadIrqPin function to obtain the pending interrupt state.
 *
 *          The IRQ can be delayed due to higher priority task, e.g. from the
 *          user code. It is essential for the laser safety timeout algorithm
 *          to determine the device ready signal as fast as possible. Thus a
 *          method is required to obtain if the IRQ is currently pending but
 *          the callback has not been invoked yet. This is what the
 *          #S2PI_ReadIrqPin function is for. Note that the #S2PI_ReadIrqPin
 *          must return 0 if not interrupt is pending and 1 else. Just like
 *          the IRQ pin is active low.
 *
 *          The test simulate a delayed GPIO interrupt by locking the interrupts
 *          until the #S2PI_ReadIrqPin detects an GPIO interrupt pending state
 *          and returns 0. This is done by the #GPIO_SPI_Callback function.
 *
 *          Note that this test does verify the GPIO interrupt that occurs
 *          whenever the device has finished the integration/measurement and
 *          new data is waiting to be read from the device. This does not test
 *          the interrupt that is triggered when the SPI transfer has finished.
 *
 * @warning The test assumes the device is in a fresh power on state and no
 *          additional reset is required. If the test fail, one may want to
 *          power cycle the device and try again.
 *
 * @warning The test locks the interrupts for a quite long period of time in
 *          order to simulate a delayed GPIO interrupt. This is not a good
 *          practice in production code. However, it is required to test the
 *          #S2PI_ReadIrqPin function. Please be aware of that when you run
 *          this test.
 *
 * @warning The test utilizes already the timer HAL in order to implement a
 *          rudimentary timeout. However, at this time, only some basic
 *          plausibility checks are performed on the timer HAL. I.e. if there
 *          is an issue in the time HAL, e.g. too fast or too slow time
 *          counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 *          one also needs to verify the timer HAL, especially the
 *          #Timer_GetCounterValue function.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if either the SPI operation did not finished
 *            or the IRQ was not detected within a specified time (check also
 *            timer HAL implementation).
 *          - #ERROR_FAIL if the IRQ pin readout failed and the no or invalid
 *            interrupt was detected.
 *          - The S2PI layer error code if #S2PI_TransferFrame, #S2PI_GetStatus
 *            or #S2PI_SetIrqCallback return any negative status.
 *****************************************************************************/
static status_t GpioInterruptTest(s2pi_slave_t slave)
{
	/* Test parameter configuration: *****************************************/
	const uint32_t timeout_ms = 300; // timeout for measurement, might be increased..
	/*************************************************************************/

	gpio_data_t data = { .Slave = slave,
			     .Status = ERROR_FAIL,
			     .Timeout_ms = timeout_ms,
			     .ReadIrqPinValue = 12345,
			     .CallbackInvoked = 0
			   };

	/* Install IRQ callback. */
	status_t status = S2PI_SetIrqCallback(slave, GPIO_Callback, &data);

	if (status != STATUS_OK) {
		error_log("GPIO interrupt test failed! The call to S2PI_SetIrqCallback "
			  "yielded error code: %d", status);
		return status;
	}

	/* Setup Device. */
	status = ConfigureDevice(slave, 0);

	if (status != STATUS_OK) {
		error_log("GPIO interrupt test failed!");
		return status;
	}

	/* Check if IRQ is not yet pending. */
	if (S2PI_ReadIrqPin(slave) == 0) {
		error_log("GPIO interrupt test failed! The S2PI_ReadIrqPin did "
			  "return 0 but no interrupt is pending since no "
			  "measurements are executed yet!");
		return ERROR_FAIL;
	};

	/* Trigger Measurement. */
	status = TriggerMeasurement(slave, 0, GPIO_SPI_Callback, &data);

	if (status != STATUS_OK) {
		error_log("GPIO interrupt test failed!");
		return status;
	}

	/* Wait for Interrupt using the callback method. */
	ltc_t start;
	Time_GetNow(&start);

	while (!data.CallbackInvoked) {
		if (Time_CheckTimeoutMSec(&start, timeout_ms)) {
			error_log("GPIO interrupt test failed! The IRQ callback was not "
				  "invoked within %d ms.", timeout_ms);
			return ERROR_TIMEOUT;
		}
	}

	/* Verify ... */
	if (data.Status != STATUS_OK) {
		error_log("GPIO interrupt test failed! The SPI IRQ callback yielded "
			  "an error status: %d (expected 0)", data.Status);
		return ERROR_FAIL;
	}

	if (data.ReadIrqPinValue != 0) {
		error_log("GPIO interrupt test failed! The IRQ pin returned "
			  "the wrong value: %d (expected 0)", data.ReadIrqPinValue);
		return ERROR_FAIL;
	}

	/* Remove callback. */
	status = S2PI_SetIrqCallback(slave, 0, 0);

	if (status != STATUS_OK) {
		error_log("GPIO interrupt test failed! The call to S2PI_SetIrqCallback "
			  "with null pointers yielded error code: %d", status);
		return status;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Reads the EEPROM byte-wise and applies Hamming weight.
 * @details The EEPROM bytes are consecutively read from the device via GPIO mode.
 *          The EEPROM_Read function is an internal API function that enables
 *          the GPIO mode from the S2PI module and reads the data via a software
 *          bit-banging protocol. Finally it disables the GPIO mode and returns
 *          to SPI mode.
 *
 *          The calls to S2PI HAL module is as follows:
 *          1. S2PI_CaptureGpioControl
 *          2. multiple calls to S2PI_WriteGpioPin and S2PI_ReadGpioPin
 *          3. S2PI_ReleaseGpioControl
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param   eeprom The 16 byte array to be filled with EEPROM data.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #STATUS_ARGUS_EEPROM_BIT_ERROR if the Hamming weight fails.
 *            interrupt was detected.
 *          - The S2PI layer error code if #S2PI_CaptureGpioControl,
 *            #S2PI_ReleaseGpioControl, #S2PI_WriteGpioPin or
 *            #S2PI_ReadGpioPin return any negative status.
 *****************************************************************************/
static status_t ReadEEPROM(s2pi_slave_t slave, uint8_t *eeprom)
{
	/* Enable EEPROM: */
	uint8_t d1[] = { 0x12, 0x00, 0x4B };
	status_t status = SPITransferSync(slave, d1, sizeof(d1));

	if (status != STATUS_OK) {
		error_log("EEPROM readout failed (enable EEPROM), "
			  "error code: %d", status);
		return status;
	}

	uint8_t data[16] = { 0 };

	/* Readout Data */
	for (uint8_t address = 0; address < 16; address++) {
		status = EEPROM_Read(slave, address, &data[address]);

		if (status != STATUS_OK) {
			error_log("EEPROM readout failed @ address 0x%02x, "
				  "error code: %d!", address, status);
			return status;
		}
	}

	/* Disable EEPROM: */
	uint8_t d2[] = { 0x12, 0x00, 0x2B };
	status = SPITransferSync(slave, d2, sizeof(d2));

	if (status != STATUS_OK) {
		error_log("EEPROM readout failed (enable EEPROM), "
			  "error code: %d", status);
		return status;
	}

	/* Apply Hamming Code */
	uint8_t err = hamming_decode(data, eeprom);

	if (err != 0) {
		error_log("EEPROM readout failed! Failed to decoding "
			  "Hamming weight (Hamming parity error: %d)!", err);
		return STATUS_ARGUS_EEPROM_BIT_ERROR;
	}

	/* Add remaining bit to the end. */
	eeprom[15] = data[15] & 0x80U;

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   GPIO Mode Test for S2PI HAL Implementation.
 *
 * @details This test verifies the GPIO mode of the S2PI HAL module. This is
 *          done by leveraging the EEPROM readout sequence that accesses the
 *          devices EEPROM via a software protocol that depends on the GPIO
 *          mode.
 *
 *          This the requires several steps, most of them are already verified
 *          in previous tests:
 *          - Basic device configuration and enable EEPROM.
 *          - Read EEPROM via GPIO mode and apply Hamming weight
 *          - Repeat several times (to eliminate random readout issues).
 *          - Decode the EEPROM (using EEPROM_Decode in argus_cal_eeprom.c)
 *          - Check if Module Number and Chip ID is not 0
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_FAIL if the GPIO test fails.
 *          - #STATUS_ARGUS_EEPROM_BIT_ERROR if the Hamming weight fails.
 *          - The S2PI layer error code if #S2PI_CaptureGpioControl,
 *            #S2PI_ReleaseGpioControl, #S2PI_WriteGpioPin or
 *            #S2PI_ReadGpioPin return any negative status.
 *****************************************************************************/
static status_t GpioModeTest(s2pi_slave_t slave)
{
	/* Read EEPROM 3 times and verify. */
	uint8_t eeprom1[16] = { 0 };
	uint8_t eeprom2[16] = { 0 };
	uint8_t eeprom3[16] = { 0 };

	status_t status = ReadEEPROM(slave, eeprom1);

	if (status != STATUS_OK) {
		error_log("GPIO mode test failed (1st attempt)!");
		return status;
	}

	status = ReadEEPROM(slave, eeprom2);

	if (status != STATUS_OK) {
		error_log("GPIO mode test failed (2nd attempt)!");
		return status;
	}

	status = ReadEEPROM(slave, eeprom3);

	if (status != STATUS_OK) {
		error_log("GPIO mode test failed (3rd attempt)!");
		return status;
	}

	/* Verify EEPROM data. */
	if ((memcmp(eeprom1, eeprom2, 16) != 0) ||
	    (memcmp(eeprom1, eeprom3, 16) != 0)) {
		error_log("GPIO Mode test failed (data comparison)!\n"
			  "The data from 3 distinct EEPROM readout does not match!");
		return ERROR_FAIL;
	}

	/* Check EEPROM data for reasonable chip and module number (i.e. not 0) */
	uint32_t chipID = EEPROM_ReadChipId(eeprom1);
	uint8_t module = EEPROM_ReadModule(eeprom1);

	if (chipID == 0 || module == 0) {
		error_log("GPIO Mode test failed (data verification)!\n"
			  "Invalid EEPROM data: Module = %d; Chip ID = %d!", module, chipID);
		return ERROR_FAIL;
	}

	print("EEPROM Readout succeeded!\n");
	print("- Module: %d\n", module);
	print("- Device ID: %d\n", chipID);

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Reads the RCO_TRIM value from the devices EEPROM.
 *
 * @details The function reads the devices EEPROM via GPIO mode and extracts
 *          the RCO_TRIM value from the EEPROM map.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param   rcotrim The read RCO_TRIM value will be returned via this pointer.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #STATUS_ARGUS_EEPROM_BIT_ERROR if the Hamming weight fails.
 *          - #ERROR_ARGUS_UNKNOWN_MODULE if the EEPROM module number is invalid.
 *          - The S2PI layer error code if #S2PI_CaptureGpioControl,
 *            #S2PI_ReleaseGpioControl, #S2PI_WriteGpioPin or
 *            #S2PI_ReadGpioPin return any negative status.
 *****************************************************************************/
static status_t ReadRcoTrim(s2pi_slave_t slave, int8_t *rcotrim)
{
	/* Read EEPROM */
	uint8_t eeprom[16] = { 0 };
	status_t status = ReadEEPROM(slave, eeprom);

	if (status != STATUS_OK) { return status; }

	uint8_t module = EEPROM_ReadModule(eeprom);

	if (module > 0 && module < 8) {
		/* Read RCO Trim Value from EEPROM Map 1/2/3: */
		*rcotrim = ((int8_t) eeprom[0]) >> 3;

	} else {
		/* Uncalibrated module; use all 0 data. */
		error_log("EEPROM Readout failed! Unknown module number: %d", module);
		return ERROR_ARGUS_UNKNOWN_MODULE;
	}

	return status;
}

/*!***************************************************************************
 * @brief   Callback function for the data ready interrupt.
 *
 * @details The function is called by the S2PI layer when the data ready
 *          interrupt is pending. The function sets the \p param to
 *          #STATUS_IDLE.
 *
 * @param   param The parameter passed to the #S2PI_SetIrqCallback function as
 *                an abstract pointer to an #status_t type.
 *****************************************************************************/
static void MeasurementCallback(void *param)
{
	*(status_t *) param = STATUS_IDLE;
}

/*!***************************************************************************
 * @brief   Triggers a measurement on the device and waits for the data ready
 *          interrupt.
 *
 * @details The function triggers a measurement cycle on the device and waits
 *          until the measurement has been finished. A \p sample count can be
 *          specified to setup individual number of digital averaging.
 *
 * @warning The test utilizes already the timer HAL in order to implement a
 *          rudimentary timeout. However, at this time, only some basic
 *          plausibility checks are performed on the timer HAL. I.e. if there
 *          is an issue in the time HAL, e.g. too fast or too slow time
 *          counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 *          one also needs to verify the timer HAL, especially the
 *          #Timer_GetCounterValue function.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 * @param   samples The specified number of averaging samples for the measurement.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if either the SPI operation did not finished
 *            or the IRQ was not detected within a specified time (check also
 *            timer HAL implementation).
 *          - The S2PI layer error code if #S2PI_TransferFrame, #S2PI_GetStatus
 *            or #S2PI_SetIrqCallback return any negative status.
 *****************************************************************************/
static status_t RunMeasurement(s2pi_slave_t slave, uint16_t samples)
{
	/* Test parameter configuration: *****************************************/
	const uint32_t timeout_ms = 300;    // The transfer timeout in ms.
	/*************************************************************************/

	volatile status_t callbackStatus = STATUS_BUSY;

	status_t status = S2PI_SetIrqCallback(slave, MeasurementCallback, (void *)&callbackStatus);

	if (status != STATUS_OK) {
		error_log("Failed to run a measurement!\n"
			  "Call to SetIrqCallback returned code: %d", status);
		return status;
	}

	status = TriggerMeasurement(slave, samples, 0, 0);

	if (status != STATUS_OK) {
		error_log("Failed to run a measurement!\n"
			  "Call to TransferFrame returned code: %d", status);
		return status;
	}

	/* Wait until the transfer is finished using a timeout. */

	ltc_t start;
	Time_GetNow(&start);

	while (callbackStatus == STATUS_BUSY) {
		if (Time_CheckTimeoutMSec(&start, timeout_ms)) {
			error_log("Failed to run a measurement!\n"
				  "Timeout occurred while waiting for the SPI interrupt (%d ms).",
				  timeout_ms);
			return ERROR_TIMEOUT;
		}
	}

	if (callbackStatus != STATUS_OK) {
		error_log("Failed to run a measurement!\n"
			  "The SPI callback yielded returned code: %d",
			  callbackStatus);
		return callbackStatus;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Test for Timer HAL Implementation by comparing timings to the device.
 *
 * @details The test verifies the timer HAL implementation by comparing the
 *          timings to the AFBR-S50 device as a reference.
 *          Therefore several measurement are executed on the device, each with
 *          a different averaging sample count. The elapsed time increases
 *          linearly with the number of averaging samples. In order to remove
 *          the time for software/setup, a linear regression fit is applied to
 *          the measurement results and only the slope is considered for the
 *          result. A delta of 102.4 microseconds per sample is expected.
 *          If the measured delta per sample is within an specified error range,
 *          the timer implementation is considered correct.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_FAIL if the timer test fails.
 *          - #STATUS_ARGUS_EEPROM_BIT_ERROR if the EEPROM Hamming weight fails.
 *          - #ERROR_ARGUS_UNKNOWN_MODULE if the EEPROM module number is invalid.
 *          - #ERROR_TIMEOUT if either the SPI operation did not finished
 *            or the IRQ was not detected within a specified time (check also
 *            timer HAL implementation).
 *          - The S2PI layer error code if #S2PI_TransferFrame, #S2PI_GetStatus,
 *            #S2PI_SetIrqCallback, #S2PI_CaptureGpioControl,
 *            #S2PI_ReleaseGpioControl, #S2PI_WriteGpioPin or #S2PI_ReadGpioPin
 *            return any negative status.
 *****************************************************************************/
static status_t TimerTest(s2pi_slave_t slave)
{
	/* Test parameter configuration: *****************************************/
	const int8_t n = 10;                // The number of measurements.
	const uint32_t ds = 100;            // The step size in averaging samples.
	const float exp_slope = 102.4f;         // Expected slope is 102.4 µs / phase / sample
	const float rel_slope_error = 3e-2f; // Relative slope tolerance is 3%.
	/*************************************************************************/

	/* Read RCOTrim value from EEPROM*/
	int8_t RcoTrim = 0;
	status_t status = ReadRcoTrim(slave, &RcoTrim);

	if (status != STATUS_OK) {
		error_log("Timer test failed!\n"
			  "EEPROM Read test returned code: %d", status);
		return status;
	}

	print("RCOTrim = %d\n", RcoTrim);

	/* Configure the device with calibrated RCO to 24MHz. */
	status = ConfigureDevice(slave, RcoTrim);

	if (status != STATUS_OK) {
		error_log("Timer test failed!\n"
			  "Configuration test returned code: %d", status);
		return status;
	}


	/* Run multiple measurements and calculate a linear regression.
	 * Note: this uses float types for simplicity. */
	float xsum = 0;
	float ysum = 0;
	float x2sum = 0;
	float xysum = 0;

	print("+-------+---------+------------+\n");
	print("| count | samples | elapsed us |\n");
	print("+-------+---------+------------+\n");

	for (uint8_t i = 1; i <= n; ++i) {
		ltc_t start;
		Time_GetNow(&start);

		uint32_t samples = ds * i;
		assert(samples < UINT16_MAX);

		status = RunMeasurement(slave, (uint16_t)samples);

		if (status != STATUS_OK) {
			error_log("Timer test failed!\n"
				  "Run measurement returned code: %d",
				  status);
			return status;
		}

		uint32_t elapsed_usec = Time_GetElapsedUSec(&start);

		xsum += (float) samples;
		ysum += (float) elapsed_usec;
		x2sum += (float) samples * (float) samples;
		xysum += (float) samples * (float) elapsed_usec;

		print("| %5d | %7d | %10d |\n", i, samples, elapsed_usec);
	}

	print("+-------+---------+------------+\n");


	const float slope = (n * xysum - xsum * ysum) / (n * x2sum - xsum * xsum);
	const float intercept = (ysum * x2sum - xsum * xysum) / (n * x2sum - xsum * xsum);
	print("Linear Regression: y(x) = %dE-7 sec * x + %dE-7 sec\n",
	      (int)(10 * slope), (int)(10 * intercept));

	/* Check the error of the slope. */
	const float max_slope = exp_slope * (1.f + rel_slope_error);
	const float min_slope = exp_slope * (1.f - rel_slope_error);

	if (slope > max_slope || slope < min_slope) {
		error_log("Time test failed!\n"
			  "The measured time slope does not match the expected value! "
			  "(actual: %dE-7, expected: %dE-7, min: %dE-7, max: %dE-7)\n",
			  (int)(10 * slope), (int)(10 * exp_slope),
			  (int)(10 * min_slope), (int)(10 * max_slope));
		return ERROR_FAIL;
	}

	return STATUS_OK;
}


/*!***************************************************************************
 * @brief   Data structure for the PIT test.
 *
 * @details Contains data that is required by the PIT timer test.
 *****************************************************************************/
typedef struct pit_data_t {
	/*! The number of PIT callback events. */
	volatile uint32_t n;

	/*! The time stamp of the first callback event. */
	ltc_t t_first;

	/*! The time stamp of the last callback event. */
	ltc_t t_last;

} pit_data_t;

/*!***************************************************************************
 * @brief   Callback function invoked by the PIT.
 *
 * @details The function that is invoked every time a specified interval elapses.
 *          An abstract parameter is passed to the function whenever it is called.
 *
 *          This implementation collects callback time stamps and counts the
 *          number of callback events using the abstract parameter.
 *
 * @param   param An abstract parameter to be passed to the callback. This is
 *                  also the identifier of the given interval.
 *****************************************************************************/
static void PIT_Callback(void *param)
{
	if (param == NULL) {
		error_log("PIT interrupt test failed: callback parameter \"param\" was NULL!");

	} else {
		pit_data_t *data = (pit_data_t *)param;

		if (data->n == 0) {
			Time_GetNow(&data->t_first);
			data->t_last = data->t_first;

		} else {
			Time_GetNow(&data->t_last);
		}

		data->n++;
	}
}

/*!***************************************************************************
 * @brief   Executes a PIT measurement and verifies the callback interval.
 *
 * @details The function configures the PIT with a given interval and waits
 *          several callback events to happen. In each callback event, the
 *          elapsed time is measured and the number of calls are counted.
 *          Finally, the average interrupt period is compared with the
 *          lifetime timer that has been already verified in a previous test
 *          (see #TimerTest). The time until the first interrupt event is also
 *          verified.
 *
 * @param   exp_dt_us The expected timer interval in microseconds.
 * @param   n The number of PIT events to await.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_FAIL if the measured interval does not match the
 *            expectations or the PIT was not disabled properly.
 *          - #ERROR_TIMEOUT if either the PIT events do not occur within the
 *            expected time.
 *          - The PIT layer error code if #Timer_SetInterval return any
 *            negative status.
 *****************************************************************************/
static status_t RunPITTest(uint32_t exp_dt_us, uint32_t n)
{
	/* Test parameter configuration: *****************************************/
	const float rel_dt_error = 5e-3f; // Relative timer interval tolerance: 0.5 %.
	const float abs_dt_error = 5.0f;  // Absolute timer interval tolerance: 5.0 us.
	/*************************************************************************/
	float dt = (float) exp_dt_us * rel_dt_error;

	if (dt < abs_dt_error) { dt = abs_dt_error; }

	const float max_dt = (float) exp_dt_us + dt;
	const float min_dt = (float) exp_dt_us - dt;

	if (dt < abs_dt_error * 3) { dt = abs_dt_error * 3; }

	const float t_first_max = (float) exp_dt_us + dt * 5; // use 5x tolerance for
	const float t_first_min = (float) exp_dt_us - dt * 5; // the first interval
	/*************************************************************************/

	print("Run PIT Test (w/ %d us interval):\n"
	      " - expected event count: %d\n"
	      " - expected interval: %d us, min: %d us, max: %d us\n"
	      " - expected first event: %d us, min: %d us, max: %d us\n",
	      exp_dt_us, n, exp_dt_us, (int)min_dt, (int)max_dt,
	      exp_dt_us, (int)t_first_min, (int)t_first_max);

	/* Setup the PIT callback with specified interval. */
	pit_data_t data = { 0 };
	status_t status = Timer_SetInterval(exp_dt_us, &data);

	if (status != STATUS_OK) {
		error_log("PIT test failed!\n"
			  "Timer_SetInterval returned status code: %d", status);
		return status;
	}

	/* Wait until n PIT callback have been happened. */
	const uint32_t timeout_us = (n + 1) * exp_dt_us;

	ltc_t start;
	Time_GetNow(&start);

	while (data.n < n) {
		if (Time_CheckTimeoutUSec(&start, timeout_us)) {
			const uint32_t elapsed_us = Time_GetElapsedUSec(&start);
			const uint32_t t_first_us = Time_DiffUSec(&start, &data.t_first);
			const uint32_t t_last_us = Time_DiffUSec(&start, &data.t_last);
			error_log("PIT test failed!\n"
				  "Waiting for the PIT interrupt events yielded a timeout.\n"
				  "Timeout: %d us; Elapsed: %d us (%d of %d events).\n"
				  "First event @ %d us, last event @ %d us",
				  timeout_us, elapsed_us, data.n, n, t_first_us, t_last_us);
			status = ERROR_TIMEOUT;
			break;
		}
	}

	if (status == STATUS_OK) {
		/* Disable the PIT timer callback. */
		status = Timer_SetInterval(0, &data);

		if (status != STATUS_OK) {
			error_log("PIT test failed!\n"
				  "Timer_SetInterval returned status code: %d", status);
		}
	}

	if (status == STATUS_OK) {
		/* Check if PIT callback is not invoked any more. */
		Time_DelayUSec(3 * exp_dt_us);

		if (data.n > n) {
			const uint32_t elapsed_us = Time_GetElapsedUSec(&start);
			error_log("PIT test failed!\n"
				  "Timer_SetInterval has been called again after it was disabled\n"
				  "(within %d us; %d of %d events in total).",
				  elapsed_us, data.n, n);
			status = ERROR_FAIL;
		}
	}

	/* Verify the measured average timer interval. */
	const float act_dt_us = Time_DiffUSec(&data.t_first, &data.t_last) / (float)(n - 1);
	const uint32_t t_first_us = Time_DiffUSec(&start, &data.t_first);
	const uint32_t t_last_us = Time_DiffUSec(&start, &data.t_last);

	print(" - actual event count: %d\n"
	      " - actual interval: %d us\n"
	      " - actual first event: %d us\n"
	      " - actual last event: %d us\n\n",
	      data.n, (int)act_dt_us, t_first_us, t_last_us);

	if (status == STATUS_OK && (t_first_us > t_first_max || t_first_us < t_first_min)) {
		error_log("PIT test failed!\n"
			  "The first timer event did not occur after the expected interval!");
		status = ERROR_FAIL;
	}

	if (status == STATUS_OK && (act_dt_us > max_dt || act_dt_us < min_dt)) {
		error_log("PIT test failed!\n"
			  "The measured timer interval does not match the expected value!");
		status = ERROR_FAIL;
	}

	print(" - test status: %d\n\n", status);

	return status;
}

/*!***************************************************************************
 * @brief   Test for PIT HAL Implementation by comparing timings to the device.
 *
 * @details The test verifies the timer HAL implementation by comparing the
 *          period between the interrupts with the lifetime timer values
 *          that has been already verified in a previous test (see #TimerTest).
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_NOT_IMPLEMENTED if the PIT functionality is not
 *            implemented and the test is skipped.
 *          - #ERROR_FAIL if the measured interval does not match the
 *            expectations or the PIT was not disabled properly.
 *          - #ERROR_TIMEOUT if either the PIT events do not occur within the
 *            expected time.
 *          - The PIT layer error code if #Timer_SetInterval or
 *            #Timer_SetCallback return any negative status.
 *****************************************************************************/
static status_t PITTest(void)
{
	status_t status = Timer_SetCallback(PIT_Callback);

	if (status == ERROR_NOT_IMPLEMENTED) { return status; }

	if (status != STATUS_OK) {
		error_log("PIT test failed!\n"
			  "Timer_SetCallback returned status code: %d", status);
		return status;
	}

	status = RunPITTest(200000, 5);

	if (status != STATUS_OK) { return status; }

	status = RunPITTest(10000, 10);

	if (status != STATUS_OK) { return status; }

	/* High Speed Test down to 1000 microseconds. If this fails, just print
	 * a message that very high frame rates might have issues. */
	status = RunPITTest(1000, 500);

	if (status != STATUS_OK) {
		print("WARNING: PIT test failed for 1000 us interval!\n"
		      "         This is only critical if high frame rates (up to 1000 fps)\n"
		      "         need to be achieved. Otherwise, the error can be safely ignored.\n");
		status = STATUS_IGNORE; // ignore
	}

	if (status == STATUS_OK) { // only run if previous test succeeded!
		/* High Speed Test down to 333 microseconds. If this fails, just print
		 * a message that very high frame rates might have issues. */
		status = RunPITTest(333, 500);

		if (status != STATUS_OK) {
			print("WARNING: PIT test failed for 333 us interval!\n"
			      "         This is only critical if very high frame rates (up to 3000 fps)\n"
			      "         need to be achieved. Otherwise, the error can be safely ignored.\n");
			status = STATUS_IGNORE; // ignore
		}
	}

	status = Timer_SetCallback(0);

	if (status != STATUS_OK) {
		error_log("PIT test failed!\n"
			  "Timer_SetCallback to 0 returned status code: %d", status);
		return status;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   Data structure for the S2PI transfer from interrupt tests.
 *
 * @details Contains data that is required by the S2PI transfer from interrupt
 *          test. The data structure is passed to the corresponding interrupt
 *          callback functions.
 *****************************************************************************/
typedef struct spi_irq_data_t {
	/*! The status of the interrupt callback function. */
	volatile status_t Status;

	/*! The S2PI slave parameter passed to the S2PI HAL functions. */
	s2pi_slave_t Slave;

	/*! The data buffer to be transferred from/to the device for testing purposes. */
	uint8_t Data[17U];

	/*! Set to true when all SPI transfers are finished. */
	volatile bool Finished;

	/*! Set to true when the second SPI transfers is started.
	    The second transfer is used to read-back the previously set values. */
	volatile bool ReadBack;

} spi_irq_data_t;


/*!***************************************************************************
 * @brief   SPI interrupt callback function for the SPI transfer from IRQ test.
 *
 * @details The interrupt callback is invoked from the S2PI module upon
 *          finishing the SPI transfer. The callback is used by the
 *          #SpiTransferFromSpiInterrupt test to trigger the second SPI transfer
 *          from the interrupt callback context.
 *
 * @note    The callback also utilizes the #print functionality. This requires
 *          a correct implementation of the corresponding function such that it
 *          can be invoked from the given interrupt context. This usually
 *          requires the underlying send (e.g. UART or USB send functions) to
 *          have higher priority that this interrupt in order to finished the
 *          print statement asynchronously.
 *
 * @param   status The S2PI module status passed to the callback.
 * @param   param The abstract interrupt callback parameter.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_INVALID_ARGUMENT if the \p param is NULL.
 *          - The S2PI layer error code if any is passed to the callback function.
 *          - The S2PI layer error code if #S2PI_TransferFrame return any.
 *****************************************************************************/
static status_t SpiTransferFromSpiInterruptCallback(status_t status, void *param)
{
	if (param == NULL) {
		error_log("SPI transfer from SPI interrupt test failed\n"
			  "callback parameter \"param\" was NULL!");
		return ERROR_INVALID_ARGUMENT;
	}

	spi_irq_data_t *data = (spi_irq_data_t *) param;

	if (status != STATUS_OK) {
		error_log("SPI transfer from SPI interrupt test failed:\n"
			  "callback received error! Error code: %d", status);
		data->Status = status;
		return status;
	}

	if (!data->ReadBack) {
		print("Invoking SPI transfer from SPI interrupt callback...\n");

		/* Clear the laser pattern and read back previous values. */
		data->Data[0] = 0x04; // Laser Pattern Register Address

		for (uint8_t i = 1; i < 17U; ++i) { data->Data[i] = 0; }

		status = S2PI_TransferFrame(data->Slave, data->Data, data->Data, 17U,
					    SpiTransferFromSpiInterruptCallback, param);

		if (status != STATUS_OK) {
			error_log("SPI transfer from SPI interrupt test failed:\n"
				  "Calling S2PI_TransferFrame from SPI interrupt "
				  "returned error code: %d", status);
			data->Status = status;
			return status;
		}

		data->ReadBack = true;

	} else {
		data->Finished = true;
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   SPI transfer from SPI interrupt callback test.
 *
 * @details This test verifies the interrupt functionality of the SPI interface.
 *          The test verifies that an SPI transfer can be triggered from the SPI
 *          interrupt service routine context.
 *
 *          The test basically repeats the #SpiConnectionTest but this time it
 *          invokes the second SPI transfer from the SPI callback function.
 *          A very common error is that the callback is invoked while the SPI
 *          module is still busy which does not allow to invoke another SPI
 *          transfer from the callback.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if the test did not finish within a specified time.
 *          - #ERROR_FAIL if the device access failed and the read data did not
 *            match the expected values.
 *          - The S2PI layer error code if #S2PI_TransferFrame or the SPI
 *            callback yield in any non-OK status.
 *****************************************************************************/
static status_t SpiTransferFromSpiInterrupt(s2pi_slave_t slave)
{
	/* Test parameter configuration: *****************************************/
	const uint32_t timeout_us = 100000; // timeout for SPI transfers to finish
	/*************************************************************************/

	status_t status = STATUS_OK;
	spi_irq_data_t data = { .Slave = slave };

	print("Invoking SPI transfer from task level...\n");

	/* Transfer a pattern to the register */
	data.Data[0] = 0x04; // Laser Pattern Register Address

	for (uint8_t i = 1; i < 17U; ++i) { data.Data[i] = i; }

	status = S2PI_TransferFrame(slave, data.Data, data.Data, 17U,
				    SpiTransferFromSpiInterruptCallback, &data);

	if (status != STATUS_OK) {
		error_log("SPI transfer from SPI interrupt test failed:\n"
			  "Failed to transfer a data frame! Error code: %d", status);
		return status;
	}

	/* Wait until transfers has finished. */
	ltc_t start;
	Time_GetNow(&start);

	while (!data.Finished && (data.Status == STATUS_OK)) {
		if (Time_CheckTimeoutUSec(&start, timeout_us)) {
			const uint32_t elapsed_us = Time_GetElapsedUSec(&start);
			error_log("SPI transfer from SPI interrupt test failed:\n"
				  "Waiting for the transfers to be finished yielded a timeout.\n"
				  "Timeout: %d us; Elapsed: %d us (%d of %d events).",
				  timeout_us, elapsed_us);
			status = ERROR_TIMEOUT;
			break;
		}
	}

	if (data.Status != STATUS_OK) {
		error_log("SPI transfer from SPI interrupt test failed:\n"
			  "Waiting for the transfers to be finished yielded a error code: %d",
			  data.Status);
		return data.Status;
	}

	print("Verify read data...\n");

	/* Verify the read pattern. */
	for (uint8_t i = 1; i < 17U; ++i) {
		if (data.Data[i] != i) {
			error_log("SPI transfer from SPI interrupt test failed:\n"
				  "Verification of read data is invalid!\n"
				  "read_data[%d] = %d, but expected was %d",
				  i, data.Data[i], i);
			return ERROR_FAIL;
		}
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   GPIO interrupt callback function for the SPI transfer from IRQ test.
 *
 * @details The interrupt callback is invoked from the S2PI module upon
 *          receiving an GPIO interrupt from the devices IRQ pin. The callback
 *          is used by the #SpiTransferFromGpioInterrupt test to trigger the
 *          first SPI transfer from the interrupt callback context.
 *
 * @note    The callback also utilizes the #print functionality. This requires
 *          a correct implementation of the corresponding function such that it
 *          can be invoked from the given interrupt context. This usually
 *          requires the underlying send (e.g. UART or USB send functions) to
 *          have higher priority that this interrupt in order to finished the
 *          print statement asynchronously.
 *
 * @param   param The abstract interrupt callback parameter.
 *****************************************************************************/
static void SpiTransferFromGpioInterruptCallback(void *param)
{
	if (param == NULL) {
		error_log("SPI transfer from GPIO interrupt test failed:\n"
			  "callback parameter \"param\" was NULL!");
		return;
	}

	print("Invoking SPI transfer from GPIO interrupt callback...\n");

	/* Clear the laser pattern and read back previous values. */
	spi_irq_data_t *data = (spi_irq_data_t *) param;
	data->Data[0] = 0x04; // Laser Pattern Register Address

	for (uint8_t i = 1; i < 17U; ++i) { data->Data[i] = i; }

	status_t status = S2PI_TransferFrame(data->Slave, data->Data, data->Data, 17U,
					     SpiTransferFromSpiInterruptCallback, param);

	if (status != STATUS_OK) {
		error_log("SPI transfer from GPIO interrupt test failed:\n"
			  "Calling S2PI_TransferFrame from GPIO interrupt "
			  "returned error code: %d", status);
		data->Status = status;
		return;
	}
}

/*!***************************************************************************
 * @brief   SPI transfer from GPIO interrupt callback test.
 *
 * @details This test verifies the interrupt functionality of the SPI interface.
 *          The test verifies that an SPI transfer can be triggered from the
 *          GPIO interrupt service routine context.
 *
 *          The test basically repeats the #SpiTransferFromSpiInterrupt but
 *          this time it invokes the first SPI transfer from the GPIO callback
 *          function. In order to trigger a GPIO interrupt, the device is
 *          configured and a measurement is started (see #GpioInterruptTest).
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if the test did not finish within a specified time.
 *          - #ERROR_FAIL if the device access failed and the read data did not
 *            match the expected values.
 *          - The S2PI layer error code if #S2PI_TransferFrame or the GPIO or
 *            SPI callback yield in any non-OK status.
 *****************************************************************************/
static status_t SpiTransferFromGpioInterrupt(s2pi_slave_t slave)
{
	/* Test parameter configuration: *****************************************/
	const uint32_t timeout_ms = 300; // timeout for measurement, might be increased..
	/*************************************************************************/

	spi_irq_data_t data = { .Slave = slave };

	/* Install IRQ callback. */
	status_t status = S2PI_SetIrqCallback(slave, SpiTransferFromGpioInterruptCallback, &data);

	if (status != STATUS_OK) {
		error_log("SPI transfer from GPIO interrupt test failed:\n"
			  "The call to S2PI_SetIrqCallback returned error code: %d", status);
		return status;
	}

	/* Setup Device for invoking GPIO interrupt. */
	status = ConfigureDevice(slave, 0);

	if (status != STATUS_OK) {
		error_log("SPI transfer from GPIO interrupt test failed.");
		return status;
	}

	/* Trigger Measurement and invoke GPIO interrupt. */
	status = TriggerMeasurement(slave, 0, 0, 0);

	if (status != STATUS_OK) {
		error_log("GPIO interrupt test failed!");
		return status;
	}

	ltc_t start;
	Time_GetNow(&start);

	/* Wait for Interrupt using the callback method. */
	while (!data.Finished) {
		if (Time_CheckTimeoutMSec(&start, timeout_ms)) {
			error_log("SPI transfer from GPIO interrupt test failed:\n"
				  "The IRQ callback was not invoked within %d ms.",
				  timeout_ms);
			return ERROR_TIMEOUT;
		}
	}

	if (data.Status != STATUS_OK) {
		error_log("SPI transfer from GPIO interrupt test failed:\n"
			  "Waiting for the transfers to be finished yielded a error code: %d",
			  data.Status);
		return data.Status;
	}

	print("Verify read data...\n");

	/* Verify the read pattern. */
	for (uint8_t i = 1; i < 17U; ++i) {
		if (data.Data[i] != i) {
			error_log("SPI transfer from GPIO interrupt test failed:\n"
				  "Verification of read data is invalid!\n"
				  "read_data[%d] = %d, but expected was %d",
				  i, data.Data[i], i);
			return ERROR_FAIL;
		}
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   PIT interrupt callback function for the SPI transfer from IRQ test.
 *
 * @details The interrupt callback is invoked from the PIT module upon periodic
 *          timeout event. The callback is used by the
 *          #SpiTransferFromPitInterrupt test to trigger the first SPI transfer
 *          from the interrupt callback context.
 *
 * @note    The callback also utilizes the #print functionality. This requires
 *          a correct implementation of the corresponding function such that it
 *          can be invoked from the given interrupt context. This usually
 *          requires the underlying send (e.g. UART or USB send functions) to
 *          have higher priority that this interrupt in order to finished the
 *          print statement asynchronously.
 *
 * @param   param The abstract interrupt callback parameter.
 *****************************************************************************/
static void SpiTransferFromPitInterruptCallback(void *param)
{
	status_t status = Timer_SetInterval(0, param); // disable timer

	if (status != STATUS_OK) {
		error_log("SPI transfer from PIT interrupt test failed:\n"
			  "Timer_SetCallback to 0 returned status code: %d",
			  status);

		if (param != NULL) { ((spi_irq_data_t *)param)->Status = status; }

		return;
	}


	if (param == NULL) {
		error_log("SPI transfer from PIT interrupt test failed:\n"
			  "callback parameter \"param\" was NULL!");
		return;
	}

	print("Invoking SPI transfer from PIT interrupt callback...\n");

	/* Clear the laser pattern and read back previous values. */
	spi_irq_data_t *data = (spi_irq_data_t *) param;
	data->Data[0] = 0x04; // Laser Pattern Register Address

	for (uint8_t i = 1; i < 17U; ++i) { data->Data[i] = i; }

	status = S2PI_TransferFrame(data->Slave, data->Data, data->Data, 17U,
				    SpiTransferFromSpiInterruptCallback, param);

	if (status != STATUS_OK) {
		error_log("SPI transfer from PIT interrupt test failed:\n"
			  "Calling S2PI_TransferFrame from GPIO interrupt "
			  "returned error code: %d", status);
		data->Status = status;
		return;
	}
}

/*!***************************************************************************
 * @brief   SPI transfer from PIT interrupt callback test.
 *
 * @details This test verifies the interrupt functionality of the SPI interface.
 *          The test verifies that an SPI transfer can be triggered from the
 *          PIT interrupt service routine context.
 *
 *          The test basically repeats the #SpiTransferFromSpiInterrupt but
 *          this time it invokes the first SPI transfer from the PIT callback
 *          function. In order to trigger a PIT interrupt, the timer is
 *          configured with a small interval and immediately disabled upon the
 *          first event.
 *
 *          Note that this test is only executed if the PIT module is actually
 *          implemented.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_NOT_IMPLEMENTED if the PIT functionality is not
 *            implemented and the test is skipped.
 *          - #ERROR_TIMEOUT if the test did not finish within a specified time.
 *          - #ERROR_FAIL if the device access failed and the read data did not
 *            match the expected values.
 *          - The S2PI layer error code if #S2PI_TransferFrame or the SPI
 *            callback yield in any non-OK status.
 *          - The PIT layer error code if #Timer_SetCallback or the PIT
 *            callback yield in any non-OK status.
 *****************************************************************************/
static status_t SpiTransferFromPitInterrupt(s2pi_slave_t slave)
{
	/* Test parameter configuration: *****************************************/
	const uint32_t timeout_ms = 100;    // timeout for test.
	const uint32_t interval_us = 1000;  // PIT interval for the first event.
	/*************************************************************************/

	spi_irq_data_t data = { .Slave = slave };

	status_t status = Timer_SetCallback(SpiTransferFromPitInterruptCallback);

	if (status == ERROR_NOT_IMPLEMENTED) { return status; }

	if (status != STATUS_OK) {
		error_log("SPI transfer from PIT interrupt test failed:\n"
			  "Timer_SetCallback returned status code: %d", status);
		return status;
	}

	/* Setup the PIT callback with specified interval. */
	status = Timer_SetInterval(interval_us, &data);

	if (status != STATUS_OK) {
		error_log("SPI transfer from PIT interrupt test failed:\n"
			  "Timer_SetInterval returned status code: %d", status);
		return status;
	}

	ltc_t start;
	Time_GetNow(&start);

	/* Wait for test to be finished. */
	while (!data.Finished) {
		if (Time_CheckTimeoutMSec(&start, timeout_ms)) {
			error_log("SPI transfer from PIT interrupt test failed:\n"
				  "The IRQ callback was not invoked within %d ms.",
				  timeout_ms);
			return ERROR_TIMEOUT;
		}
	}

	if (data.Status != STATUS_OK) {
		error_log("SPI transfer from PIT interrupt test failed:\n"
			  "Waiting for the transfers to be finished yielded a error code: %d",
			  data.Status);
		return data.Status;
	}

	status = Timer_SetCallback(0);

	if (status != STATUS_OK) {
		error_log("SPI transfer from PIT interrupt test failed:\n"
			  "Timer_SetCallback to 0 returned status code: %d", status);
		return status;
	}

	print("Verify read data...\n");

	/* Verify the read pattern. */
	for (uint8_t i = 1; i < 17U; ++i) {
		if (data.Data[i] != i) {
			error_log("SPI transfer from PIT interrupt test failed:\n"
				  "Verification of read data is invalid!\n"
				  "read_data[%d] = %d, but expected was %d",
				  i, data.Data[i], i);
			return ERROR_FAIL;
		}
	}

	return STATUS_OK;
}

/*!***************************************************************************
 * @brief   SPI Transfer from Interrupt Test for S2PI HAL Implementation.
 *
 * @details This test verifies the interrupt functionality of the SPI interface.
 *          The test verifies that an SPI transfer can be triggered from the
 *          interrupt service routine context. I.e. the #S2PI_TransferFrame
 *          function is called from the following interrupts:
 *          - SPI interrupt
 *          - GPIO interrupt
 *          - PIT interrupt (optional, if PIT is implemented)
 *
 * @warning The test utilizes already the timer HAL in order to implement a
 *          rudimentary timeout. However, at this time, only some basic
 *          plausibility checks are performed on the timer HAL. I.e. if there
 *          is an issue in the time HAL, e.g. too fast or too slow time
 *          counting, the test may fail with an #ERROR_TIMEOUT. In this case,
 *          one also needs to verify the timer HAL, especially the
 *          #Timer_GetCounterValue function.
 *
 * @param   slave The S2PI slave parameter passed to the S2PI HAL functions.
 *
 * @return  Returns the \link #status_t status\endlink:
 *          - #STATUS_OK on success.
 *          - #ERROR_TIMEOUT if the operation did not finished within a specified
 *            time (check also timer HAL implementation).
 *          - #ERROR_FAIL if the device access failed and the read data did not
 *            match the expected values.
 *          - The S2PI layer error code if #S2PI_TransferFrame or #S2PI_GetStatus
 *            return any negative status.
 *****************************************************************************/
static status_t SpiTransferFromInterruptTest(s2pi_slave_t slave)
{
	status_t status = STATUS_OK;

	print(" .1 >> SPI Transfer from SPI Interrupt Test\n");
	status = SpiTransferFromSpiInterrupt(slave);

	if (status != STATUS_OK) { return status; }

	print(" .1 >> PASS\n\n");

	print(" .2 >> SPI Transfer from GPIO Interrupt Test\n");
	status = SpiTransferFromGpioInterrupt(slave);

	if (status != STATUS_OK) { return status; }

	print(" .2 >> PASS\n\n");

	print(" .3 >> SPI Transfer from PIT Interrupt Test\n");
	status = SpiTransferFromPitInterrupt(slave);

	if (status == ERROR_NOT_IMPLEMENTED) {
		print(" .3 >> SKIPPED (PIT is not implemented)\n\n");

	} else {
		if (status != STATUS_OK) { return status; }

		print(" .3 >> PASS\n\n");
	}

	return STATUS_OK;
}

/*! @} */
