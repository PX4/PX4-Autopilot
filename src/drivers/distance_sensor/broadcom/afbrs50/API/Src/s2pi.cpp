

#include "irq.h"
#include "s2pi.h"

#include <stdio.h>

#include <board_config.h>

#include <nuttx/spi/spi.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/workqueue.h>

#include <lib/perf/perf_counter.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

/*! A structure to hold all internal data required by the S2PI module. */
typedef struct {
	/*! Determines the current driver status. */
	volatile status_t Status;

	/*! Determines the current S2PI slave. */
	volatile s2pi_slave_t Slave;

	/*! A callback function to be called after transfer/run mode is completed. */
	s2pi_callback_t Callback;

	/*! A parameter to be passed to the callback function. */
	void *CallbackData;

	/*! A callback function to be called after external interrupt is triggered. */
	s2pi_irq_callback_t IrqCallback;

	/*! A parameter to be passed to the interrupt callback function. */
	void *IrqCallbackData;

	struct spi_dev_s *spidev;
	uint8_t *spi_tx_data;
	uint8_t *spi_rx_data;
	size_t spi_frame_size;

	/*! The mapping of the GPIO blocks and pins for this device. */
	const uint32_t GPIOs[ S2PI_IRQ + 1 ];
}
s2pi_handle_t;

s2pi_handle_t s2pi_ = { .GPIOs = { [ S2PI_CLK ]  = BROADCOM_AFBR_S50_S2PI_CLK,
				   [ S2PI_CS ]   = BROADCOM_AFBR_S50_S2PI_CS,
				   [ S2PI_MOSI ] = BROADCOM_AFBR_S50_S2PI_MOSI,
				   [ S2PI_MISO ] = BROADCOM_AFBR_S50_S2PI_MISO,
				   [ S2PI_IRQ ]  = BROADCOM_AFBR_S50_S2PI_IRQ
				 }
		      };

static perf_counter_t irq_perf = NULL;

class AFBRS50_SPI :  public px4::ScheduledWorkItem
{
public:
	AFBRS50_SPI();
	void schedule_now();
	void schedule_clear();

private:

	void Run() override;

};

AFBRS50_SPI::AFBRS50_SPI():
	// NOTE: we use SPI0 WQ since it is the 2nd highest priority thread (behind rate_ctrl).
	// TODO: we should fix how SPI comms work. Async SPI comms is
	// undesirable. We should use SPI TX DMA complete callback
	// instead of relying on a high priority thread.
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::SPI0)
{
	// Anything to do?
}

void AFBRS50_SPI::Run()
{
	px4_arch_gpiowrite(s2pi_.GPIOs[S2PI_CS], 0);
	SPI_EXCHANGE(s2pi_.spidev, s2pi_.spi_tx_data, s2pi_.spi_rx_data, s2pi_.spi_frame_size);
	px4_arch_gpiowrite(s2pi_.GPIOs[S2PI_CS], 1);

	//// WARNING!
	// After the last SPI TX we have ~60us to execute the below
	// callback otherwise the IRQ will fire and we're screwed.
	// The proper way to solve this problem is to either fix
	// the API or to configure SPI TX DMA callback complete
	// to execute the below callback immediately.


	// If we are pre-empted here and the IRQ fires before the
	// callback has been invoked -- we're screwed.

	IRQ_LOCK();
	s2pi_.Status = STATUS_IDLE;

	if (s2pi_.Callback != 0) {
		s2pi_callback_t callback = s2pi_.Callback;
		s2pi_.Callback = 0;
		callback(STATUS_OK, s2pi_.CallbackData);
	}

	IRQ_UNLOCK();
}

void AFBRS50_SPI::schedule_now()
{
	ScheduleNow();
}

void AFBRS50_SPI::schedule_clear()
{
	ScheduleClear();
}

static AFBRS50_SPI *_spi_iface = nullptr;

/*!***************************************************************************
* @brief Initialize the S2PI module.
* @details Setup the board as a S2PI master, this also sets up up the S2PI
* pins.
* The SPI interface is initialized with the corresponding default
* SPI slave (i.e. CS and IRQ lines) and the default baud rate.
*
* @param defaultSlave The default SPI slave to be addressed right after
* module initialization.
* @param baudRate_Bps The default SPI baud rate in bauds-per-second.
*
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t S2PI_Init(s2pi_slave_t defaultSlave, uint32_t baudRate_Bps)
{
	(void)defaultSlave;

	px4_arch_configgpio(BROADCOM_AFBR_S50_S2PI_CS);

	s2pi_.spidev = px4_spibus_initialize(BROADCOM_AFBR_S50_S2PI_SPI_BUS);

	// Falling edge callback
	auto callback = [](int irq, void *context, void *arg) -> int {
		if (s2pi_.IrqCallback != 0)
		{
			perf_begin(irq_perf);
			s2pi_.IrqCallback(s2pi_.IrqCallbackData);
			perf_end(irq_perf);
		}

		return 0;
	};
	// NOTE: we enable the interrupt event here but do not configure the GPIO.
	// We configure the GPIO and enable the interrupt after the device mode
	// has been configured. This prevents erroneous interrupts from occuring.
	px4_arch_gpiosetevent(BROADCOM_AFBR_S50_S2PI_IRQ, false, true, false, callback, NULL);

	irq_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": irq callback");

	_spi_iface = new AFBRS50_SPI();

	return S2PI_SetBaudRate(baudRate_Bps);
}

/*!***************************************************************************
* @brief Returns the status of the SPI module.
*
* @return Returns the \link #status_t status\endlink:
* - #STATUS_IDLE: No SPI transfer or GPIO access is ongoing.
* - #STATUS_BUSY: An SPI transfer is in progress.
* - #STATUS_S2PI_GPIO_MODE: The module is in GPIO mode.
*****************************************************************************/
status_t S2PI_GetStatus(s2pi_slave_t slave)
{
	(void)slave;

	return s2pi_.Status;
}

status_t S2PI_TryGetMutex(s2pi_slave_t slave)
{
	(void) slave;
	return STATUS_OK;
}

void S2PI_ReleaseMutex(s2pi_slave_t slave)
{
	(void) slave;
}

/*!***************************************************************************
* @brief Sets the SPI baud rate in bps.
* @param baudRate_Bps The default SPI baud rate in bauds-per-second.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
* - #STATUS_OK on success
* - #ERROR_S2PI_INVALID_BAUD_RATE on invalid baud rate value.
*****************************************************************************/
status_t S2PI_SetBaudRate(uint32_t baudRate_Bps)
{
	SPI_SETMODE(s2pi_.spidev, SPIDEV_MODE3);
	SPI_SETBITS(s2pi_.spidev, 8);
	SPI_SETFREQUENCY(s2pi_.spidev, baudRate_Bps);
	return STATUS_OK;
}

/*!*****************************************************************************
* @brief Captures the S2PI pins for GPIO usage.
* @details The SPI is disabled (module status: #STATUS_S2PI_GPIO_MODE) and the
* pins are configured for GPIO operation. The GPIO control must be
* release with the #S2PI_ReleaseGpioControl function in order to
* switch back to ordinary SPI functionality.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t S2PI_CaptureGpioControl(s2pi_slave_t slave)
{
	(void)slave;

	/* Check if something is ongoing. */
	IRQ_LOCK();
	status_t status = s2pi_.Status;

	if (status != STATUS_IDLE) {
		IRQ_UNLOCK();
		return status;
	}

	s2pi_.Status = STATUS_S2PI_GPIO_MODE;

	// GPIO mode (output push pull)
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_SET(s2pi_.GPIOs[S2PI_CLK]));
	px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(s2pi_.GPIOs[S2PI_MISO]));
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_SET(s2pi_.GPIOs[S2PI_MOSI]));

	IRQ_UNLOCK();

	return STATUS_OK;
}

/*!*****************************************************************************
* @brief Releases the S2PI pins from GPIO usage and switches back to SPI mode.
* @details The GPIO pins are configured for SPI operation and the GPIO mode is
* left. Must be called if the pins are captured for GPIO operation via
* the #S2PI_CaptureGpioControl function.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t S2PI_ReleaseGpioControl(s2pi_slave_t slave)
{
	(void)slave;

	/* Check if something is ongoing. */
	IRQ_LOCK();
	status_t status = s2pi_.Status;

	if (status != STATUS_S2PI_GPIO_MODE) {
		IRQ_UNLOCK();
		return status;
	}

	s2pi_.Status = STATUS_IDLE;

	// SPI alternate
	stm32_configgpio(s2pi_.GPIOs[S2PI_CLK]);
	stm32_configgpio(s2pi_.GPIOs[S2PI_MISO]);
	stm32_configgpio(s2pi_.GPIOs[S2PI_MOSI]);

	// probably not necessary
	stm32_spibus_initialize(BROADCOM_AFBR_S50_S2PI_SPI_BUS);

	IRQ_UNLOCK();

	return STATUS_OK;
}

/*!*****************************************************************************
* @brief Writes the output for a specified SPI pin in GPIO mode.
* @details This function writes the value of an SPI pin if the SPI pins are
* captured for GPIO operation via the #S2PI_CaptureGpioControl previously.
* @param slave The specified S2PI slave.
* @param pin The specified S2PI pin.
* @param value The GPIO pin state to write (0 = low, 1 = high).
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t S2PI_WriteGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t value)
{
	(void)slave;

	/* Check if pin is valid. */
	if (pin > S2PI_IRQ || value > 1) {
		return ERROR_INVALID_ARGUMENT;
	}

	/* Check if in GPIO mode. */
	if (s2pi_.Status != STATUS_S2PI_GPIO_MODE) {
		return ERROR_S2PI_INVALID_STATE;
	}

	px4_arch_gpiowrite(s2pi_.GPIOs[pin], value);

	return STATUS_OK;
}

/*!*****************************************************************************
* @brief Reads the input from a specified SPI pin in GPIO mode.
* @details This function reads the value of an SPI pin if the SPI pins are
* captured for GPIO operation via the #S2PI_CaptureGpioControl previously.
* @param slave The specified S2PI slave.
* @param pin The specified S2PI pin.
* @param value The GPIO pin state to read (0 = low, 1 = high).
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t S2PI_ReadGpioPin(s2pi_slave_t slave, s2pi_pin_t pin, uint32_t *value)
{
	(void)slave;

	/* Check if pin is valid. */
	if (pin > S2PI_IRQ || !value) {
		return ERROR_INVALID_ARGUMENT;
	}

	/* Check if in GPIO mode. */
	if (s2pi_.Status != STATUS_S2PI_GPIO_MODE) {
		return ERROR_S2PI_INVALID_STATE;
	}

	*value = px4_arch_gpioread(s2pi_.GPIOs[pin]);

	return STATUS_OK;
}

/*!***************************************************************************
* @brief Cycles the chip select line.
* @details In order to cancel the integration on the ASIC, a fast toggling
* of the chip select pin of the corresponding SPI slave is required.
* Therefore, this function toggles the CS from high to low and back.
* The SPI instance for the specified S2PI slave must be idle,
* otherwise the status #STATUS_BUSY is returned.
* @param slave The specified S2PI slave.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t S2PI_CycleCsPin(s2pi_slave_t slave)
{
	(void)slave;

	/* Check the driver status. */
	IRQ_LOCK();
	status_t status = s2pi_.Status;

	if (status != STATUS_IDLE) {
		IRQ_UNLOCK();
		return status;
	}

	s2pi_.Status = STATUS_BUSY;

	px4_arch_gpiowrite(s2pi_.GPIOs[S2PI_CS], 0);
	px4_arch_gpiowrite(s2pi_.GPIOs[S2PI_CS], 1);

	s2pi_.Status = STATUS_IDLE;

	IRQ_UNLOCK();

	return STATUS_OK;
}

/*!***************************************************************************
* @brief Transfers a single SPI frame asynchronously.
* @details Transfers a single SPI frame in asynchronous manner. The Tx data
* buffer is written to the device via the MOSI line.
* Optionally the data on the MISO line is written to the provided
* Rx data buffer. If null, the read data is dismissed.
* The transfer of a single frame requires to not toggle the chip
* select line to high in between the data frame.
* An optional callback is invoked when the asynchronous transfer
* is finished. Note that the provided buffer must not change while
* the transfer is ongoing. Use the slave parameter to determine
* the corresponding slave via the given chip select line.
*
* @param slave The specified S2PI slave.
* @param txData The 8-bit values to write to the SPI bus MOSI line.
* @param rxData The 8-bit values received from the SPI bus MISO line
* (pass a null pointer if the data don't need to be read).
* @param frameSize The number of 8-bit values to be sent/received.
* @param callback A callback function to be invoked when the transfer is
* finished. Pass a null pointer if no callback is required.
* @param callbackData A pointer to a state that will be passed to the
* callback. Pass a null pointer if not used.
*
* @return Returns the \link #status_t status\endlink:
* - #STATUS_OK: Successfully invoked the transfer.
* - #ERROR_INVALID_ARGUMENT: An invalid parameter has been passed.
* - #ERROR_S2PI_INVALID_SLAVE: A wrong slave identifier is provided.
* - #STATUS_BUSY: An SPI transfer is already in progress. The
* transfer was not started.
* - #STATUS_S2PI_GPIO_MODE: The module is in GPIO mode. The transfer
* was not started.
*****************************************************************************/

status_t S2PI_TransferFrame(s2pi_slave_t spi_slave, uint8_t const *txData, uint8_t *rxData, size_t frameSize,
			    s2pi_callback_t callback, void *callbackData)
{
	/* Verify arguments. */
	if (!txData || frameSize == 0 || frameSize >= 0x10000) {
		return ERROR_INVALID_ARGUMENT;
	}

	/* Check the spi slave.*/
	// if (spi_slave != S2PI_S2) {
	// 	return ERROR_S2PI_INVALID_SLAVE;
	// }

	/* Check the driver status, lock if idle. */
	IRQ_LOCK();
	status_t status = s2pi_.Status;

	if (status != STATUS_IDLE) {
		IRQ_UNLOCK();
		return status;
	}

	s2pi_.Status = STATUS_BUSY;

	/* Set the callback information */
	s2pi_.Callback = callback;
	s2pi_.CallbackData = callbackData;

	s2pi_.spi_tx_data = (uint8_t *)txData;
	s2pi_.spi_rx_data = rxData;
	s2pi_.spi_frame_size = frameSize;

	_spi_iface->schedule_now();

	IRQ_UNLOCK();

	return STATUS_OK;
}

/*!***************************************************************************
* @brief Terminates a currently ongoing asynchronous SPI transfer.
* @details When a callback is set for the current ongoing activity, it is
* invoked with the #ERROR_ABORTED error byte.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t S2PI_Abort(s2pi_slave_t slave)
{
	(void)slave;

	status_t status = s2pi_.Status;

	/* Check if something is ongoing. */
	if (status == STATUS_IDLE) {
		return STATUS_OK;
	}

	/* Abort SPI transfer. */
	if (status == STATUS_BUSY) {
		_spi_iface->schedule_clear();
	}

	return STATUS_OK;
}

/*!***************************************************************************
* @brief Set a callback for the GPIO IRQ for a specified S2PI slave.
*
* @param slave The specified S2PI slave.
* @param callback A callback function to be invoked when the specified
* S2PI slave IRQ occurs. Pass a null pointer to disable
* the callback.
* @param callbackData A pointer to a state that will be passed to the
* callback. Pass a null pointer if not used.
*
* @return Returns the \link #status_t status\endlink:
* - #STATUS_OK: Successfully installation of the callback.
* - #ERROR_S2PI_INVALID_SLAVE: A wrong slave identifier is provided.
*****************************************************************************/
status_t S2PI_SetIrqCallback(s2pi_slave_t slave, s2pi_irq_callback_t callback, void *callbackData)
{
	(void)slave;

	s2pi_.IrqCallback = callback;
	s2pi_.IrqCallbackData = callbackData;

	return STATUS_OK;
}

/*!***************************************************************************
* @brief Reads the current status of the IRQ pin.
* @details In order to keep a low priority for GPIO IRQs, the state of the
* IRQ pin must be read in order to reliable check for chip timeouts.
*
* The execution of the interrupt service routine for the data-ready
* interrupt from the corresponding GPIO pin might be delayed due to
* priority issues. The delayed execution might disable the timeout
* for the eye-safety checker too late causing false error messages.
* In order to overcome the issue, the state of the IRQ GPIO input
* pin is read before raising a timeout error in order to check if
* the device has already finished but the IRQ is still pending to be
* executed!
* @param slave The specified S2PI slave.
* @return Returns 1U if the IRQ pin is high (IRQ not pending) and 0U if the
* devices pulls the pin to low state (IRQ pending).
*****************************************************************************/
uint32_t S2PI_ReadIrqPin(s2pi_slave_t slave)
{
	(void)slave;

	return px4_arch_gpioread(s2pi_.GPIOs[S2PI_IRQ]);
}
