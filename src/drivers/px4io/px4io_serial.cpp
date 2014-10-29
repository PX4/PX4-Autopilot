/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

 /**
  * @file px4io_serial.cpp
  *
  * Serial interface for PX4IO
  */

/* XXX trim includes */
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>

/* XXX might be able to prune these */
#include <nuttx/arch.h>
#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <debug.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <board_config.h>

#include <systemlib/perf_counter.h>

#include <modules/px4iofirmware/protocol.h>

#ifdef PX4IO_SERIAL_BASE

device::Device	*PX4IO_serial_interface();

/* serial register accessors */
#define REG(_x)		(*(volatile uint32_t *)(PX4IO_SERIAL_BASE + _x))
#define rSR		REG(STM32_USART_SR_OFFSET)
#define rDR		REG(STM32_USART_DR_OFFSET)
#define rBRR		REG(STM32_USART_BRR_OFFSET)
#define rCR1		REG(STM32_USART_CR1_OFFSET)
#define rCR2		REG(STM32_USART_CR2_OFFSET)
#define rCR3		REG(STM32_USART_CR3_OFFSET)
#define rGTPR		REG(STM32_USART_GTPR_OFFSET)

class PX4IO_serial : public device::Device
{
public:
	PX4IO_serial();
	virtual ~PX4IO_serial();

	virtual int	init();
	virtual int	read(unsigned offset, void *data, unsigned count = 1);
	virtual int	write(unsigned address, void *data, unsigned count = 1);
	virtual int	ioctl(unsigned operation, unsigned &arg);

private:
	/*
	 * XXX tune this value
	 *
	 * At 1.5Mbps each register takes 13.3µs, and we always transfer a full packet.
	 * Packet overhead is 26µs for the four-byte header.
	 *
	 * 32 registers = 451µs
	 *
	 * Maybe we can just send smaller packets (e.g. 8 regs) and loop for larger (less common)
	 * transfers? Could cause issues with any regs expecting to be written atomically...
	 */
	static IOPacket		_dma_buffer;		// XXX static to ensure DMA-able memory

	DMA_HANDLE		_tx_dma;
	DMA_HANDLE		_rx_dma;

	/** saved DMA status */
	static const unsigned	_dma_status_inactive = 0x80000000;	// low bits overlap DMA_STATUS_* values
	static const unsigned	_dma_status_waiting  = 0x00000000;
	volatile unsigned	_rx_dma_status;

	/** bus-ownership lock */
	sem_t			_bus_semaphore;

	/** client-waiting lock/signal */
	sem_t			_completion_semaphore;

	/**
	 * Start the transaction with IO and wait for it to complete.
	 */
	int			_wait_complete();

	/**
	 * DMA completion handler.
	 */
	static void		_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
	void			_do_rx_dma_callback(unsigned status);

	/**
	 * Serial interrupt handler.
	 */
	static int		_interrupt(int vector, void *context);
	void			_do_interrupt();

	/**
	 * Cancel any DMA in progress with an error.
	 */
	void			_abort_dma();

	/**
	 * Performance counters.
	 */
	perf_counter_t		_pc_txns;
	perf_counter_t		_pc_dmasetup;
	perf_counter_t		_pc_retries;
	perf_counter_t		_pc_timeouts;
	perf_counter_t		_pc_crcerrs;
	perf_counter_t		_pc_dmaerrs;
	perf_counter_t		_pc_protoerrs;
	perf_counter_t		_pc_uerrs;
	perf_counter_t		_pc_idle;
	perf_counter_t		_pc_badidle;

	/* do not allow top copying this class */
	PX4IO_serial(PX4IO_serial &);
	PX4IO_serial& operator = (const PX4IO_serial &);

};

IOPacket PX4IO_serial::_dma_buffer;
static PX4IO_serial *g_interface;

device::Device
*PX4IO_serial_interface()
{
	return new PX4IO_serial();
}

PX4IO_serial::PX4IO_serial() :
	Device("PX4IO_serial"),
	_tx_dma(nullptr),
	_rx_dma(nullptr),
	_rx_dma_status(_dma_status_inactive),
	_bus_semaphore(SEM_INITIALIZER(0)),
	_completion_semaphore(SEM_INITIALIZER(0)),
	_pc_txns(perf_alloc(PC_ELAPSED, "io_txns     ")),
	_pc_dmasetup(perf_alloc(PC_ELAPSED,	"io_dmasetup ")),
	_pc_retries(perf_alloc(PC_COUNT,	"io_retries  ")),
	_pc_timeouts(perf_alloc(PC_COUNT,	"io_timeouts ")),
	_pc_crcerrs(perf_alloc(PC_COUNT,	"io_crcerrs  ")),
	_pc_dmaerrs(perf_alloc(PC_COUNT,	"io_dmaerrs  ")),
	_pc_protoerrs(perf_alloc(PC_COUNT,	"io_protoerrs")),
	_pc_uerrs(perf_alloc(PC_COUNT,		"io_uarterrs ")),
	_pc_idle(perf_alloc(PC_COUNT,		"io_idle     ")),
	_pc_badidle(perf_alloc(PC_COUNT,	"io_badidle  "))
{
	g_interface = this;
}

PX4IO_serial::~PX4IO_serial()
{
	if (_tx_dma != nullptr) {
		stm32_dmastop(_tx_dma);
		stm32_dmafree(_tx_dma);
	}
	if (_rx_dma != nullptr) {
		stm32_dmastop(_rx_dma);
		stm32_dmafree(_rx_dma);
	}

	/* reset the UART */
	rCR1 = 0;
	rCR2 = 0;
	rCR3 = 0;

	/* detach our interrupt handler */
	up_disable_irq(PX4IO_SERIAL_VECTOR);
	irq_detach(PX4IO_SERIAL_VECTOR);

	/* restore the GPIOs */
	stm32_unconfiggpio(PX4IO_SERIAL_TX_GPIO);
	stm32_unconfiggpio(PX4IO_SERIAL_RX_GPIO);

	/* and kill our semaphores */
	sem_destroy(&_completion_semaphore);
	sem_destroy(&_bus_semaphore);

	perf_free(_pc_txns);
	perf_free(_pc_dmasetup);
	perf_free(_pc_retries);
	perf_free(_pc_timeouts);
	perf_free(_pc_crcerrs);
	perf_free(_pc_dmaerrs);
	perf_free(_pc_protoerrs);
	perf_free(_pc_uerrs);
	perf_free(_pc_idle);
	perf_free(_pc_badidle);

	if (g_interface == this)
		g_interface = nullptr;
}

int
PX4IO_serial::init()
{
	/* allocate DMA */
	_tx_dma = stm32_dmachannel(PX4IO_SERIAL_TX_DMAMAP);
	_rx_dma = stm32_dmachannel(PX4IO_SERIAL_RX_DMAMAP);
	if ((_tx_dma == nullptr) || (_rx_dma == nullptr))
		return -1;

	/* configure pins for serial use */
	stm32_configgpio(PX4IO_SERIAL_TX_GPIO);
	stm32_configgpio(PX4IO_SERIAL_RX_GPIO);

	/* reset & configure the UART */
	rCR1 = 0;
	rCR2 = 0;
	rCR3 = 0;

	/* eat any existing interrupt status */
	(void)rSR;
	(void)rDR;

	/* configure line speed */
	uint32_t usartdiv32 = PX4IO_SERIAL_CLOCK / (PX4IO_SERIAL_BITRATE / 2);
	uint32_t mantissa = usartdiv32 >> 5;
	uint32_t fraction = (usartdiv32 - (mantissa << 5) + 1) >> 1;
	rBRR = (mantissa << USART_BRR_MANT_SHIFT) | (fraction << USART_BRR_FRAC_SHIFT);

	/* attach serial interrupt handler */
	irq_attach(PX4IO_SERIAL_VECTOR, _interrupt);
	up_enable_irq(PX4IO_SERIAL_VECTOR);

	/* enable UART in DMA mode, enable error and line idle interrupts */
	rCR3 = USART_CR3_EIE;

	rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;

	/* create semaphores */
	sem_init(&_completion_semaphore, 0, 0);
	sem_init(&_bus_semaphore, 0, 1);


	/* XXX this could try talking to IO */

	return 0;
}

int
PX4IO_serial::ioctl(unsigned operation, unsigned &arg)
{

	switch (operation) {

	case 1:		/* XXX magic number - test operation */
		switch (arg) {
		case 0:
			lowsyslog("test 0\n");

			/* kill DMA, this is a PIO test */
			stm32_dmastop(_tx_dma);
			stm32_dmastop(_rx_dma);
			rCR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);

			for (;;) {
				while (!(rSR & USART_SR_TXE))
					;
				rDR = 0x55;
			}
			return 0;

		case 1:
			{
				unsigned fails = 0;
				for (unsigned count = 0;; count++) {
					uint16_t value = count & 0xffff;

					if (write((PX4IO_PAGE_TEST << 8) | PX4IO_P_TEST_LED, &value, 1) != 0)
						fails++;
						
					if (count >= 5000) {
						lowsyslog("==== test 1 : %u failures ====\n", fails);
						perf_print_counter(_pc_txns);
						perf_print_counter(_pc_dmasetup);
						perf_print_counter(_pc_retries);
						perf_print_counter(_pc_timeouts);
						perf_print_counter(_pc_crcerrs);
						perf_print_counter(_pc_dmaerrs);
						perf_print_counter(_pc_protoerrs);
						perf_print_counter(_pc_uerrs);
						perf_print_counter(_pc_idle);
						perf_print_counter(_pc_badidle);
						count = 0;
					}
				}
				return 0;
			}
		case 2:
			lowsyslog("test 2\n");
			return 0;
		}
	default:
		break;
	}

	return -1;
}

int
PX4IO_serial::write(unsigned address, void *data, unsigned count)
{
	uint8_t page = address >> 8;
	uint8_t offset = address & 0xff;
	const uint16_t *values = reinterpret_cast<const uint16_t *>(data);

	if (count > PKT_MAX_REGS)
		return -EINVAL;

	sem_wait(&_bus_semaphore);

	int result;
	for (unsigned retries = 0; retries < 3; retries++) {

		_dma_buffer.count_code = count | PKT_CODE_WRITE;
		_dma_buffer.page = page;
		_dma_buffer.offset = offset;
		memcpy((void *)&_dma_buffer.regs[0], (void *)values, (2 * count));
		for (unsigned i = count; i < PKT_MAX_REGS; i++)
			_dma_buffer.regs[i] = 0x55aa;

		/* XXX implement check byte */

		/* start the transaction and wait for it to complete */
		result = _wait_complete();

		/* successful transaction? */
		if (result == OK) {

			/* check result in packet */
			if (PKT_CODE(_dma_buffer) == PKT_CODE_ERROR) {

				/* IO didn't like it - no point retrying */
				result = -EINVAL;
				perf_count(_pc_protoerrs);
			}

			break;
		}
		perf_count(_pc_retries);
	}

	sem_post(&_bus_semaphore);

	if (result == OK)
		result = count;
	return result;
}

int
PX4IO_serial::read(unsigned address, void *data, unsigned count)
{
	uint8_t page = address >> 8;
	uint8_t offset = address & 0xff;
	uint16_t *values = reinterpret_cast<uint16_t *>(data);

	if (count > PKT_MAX_REGS)
		return -EINVAL;

	sem_wait(&_bus_semaphore);

	int result;
	for (unsigned retries = 0; retries < 3; retries++) {

		_dma_buffer.count_code = count | PKT_CODE_READ;
		_dma_buffer.page = page;
		_dma_buffer.offset = offset;

		/* start the transaction and wait for it to complete */
		result = _wait_complete();

		/* successful transaction? */
		if (result == OK) {

			/* check result in packet */
			if (PKT_CODE(_dma_buffer) == PKT_CODE_ERROR) {

				/* IO didn't like it - no point retrying */
				result = -EINVAL;
				perf_count(_pc_protoerrs);

			/* compare the received count with the expected count */
			} else if (PKT_COUNT(_dma_buffer) != count) {

				/* IO returned the wrong number of registers - no point retrying */
				result = -EIO;
				perf_count(_pc_protoerrs);

			/* successful read */				
			} else {

				/* copy back the result */
				memcpy(values, &_dma_buffer.regs[0], (2 * count));
			}

			break;
		}
		perf_count(_pc_retries);
	}

	sem_post(&_bus_semaphore);

	if (result == OK)
		result = count;
	return result;
}

int
PX4IO_serial::_wait_complete()
{
	/* clear any lingering error status */
	(void)rSR;
	(void)rDR;

	/* start RX DMA */
	perf_begin(_pc_txns);
	perf_begin(_pc_dmasetup);

	/* DMA setup time ~3µs */
	_rx_dma_status = _dma_status_waiting;

	/*
	 * Note that we enable circular buffer mode as a workaround for
	 * there being no API to disable the DMA FIFO. We need direct mode
	 * because otherwise when the line idle interrupt fires there
	 * will be packet bytes still in the DMA FIFO, and we will assume
	 * that the idle was spurious.
	 *
	 * XXX this should be fixed with a NuttX change.
	 */
	stm32_dmasetup(
		_rx_dma,
		PX4IO_SERIAL_BASE + STM32_USART_DR_OFFSET,
		reinterpret_cast<uint32_t>(&_dma_buffer),
		sizeof(_dma_buffer),
		DMA_SCR_CIRC		|	/* XXX see note above */
		DMA_SCR_DIR_P2M		|
		DMA_SCR_MINC		|
		DMA_SCR_PSIZE_8BITS	|
		DMA_SCR_MSIZE_8BITS	|
		DMA_SCR_PBURST_SINGLE	|
		DMA_SCR_MBURST_SINGLE);
	stm32_dmastart(_rx_dma, _dma_callback, this, false);
	rCR3 |= USART_CR3_DMAR;

	/* start TX DMA - no callback if we also expect a reply */
	/* DMA setup time ~3µs */
	_dma_buffer.crc = 0;
	_dma_buffer.crc = crc_packet(&_dma_buffer);
	stm32_dmasetup(
		_tx_dma,
		PX4IO_SERIAL_BASE + STM32_USART_DR_OFFSET,
		reinterpret_cast<uint32_t>(&_dma_buffer),
		PKT_SIZE(_dma_buffer),
		DMA_SCR_DIR_M2P		|
		DMA_SCR_MINC		|
		DMA_SCR_PSIZE_8BITS	|
		DMA_SCR_MSIZE_8BITS	|
		DMA_SCR_PBURST_SINGLE	|
		DMA_SCR_MBURST_SINGLE);
	stm32_dmastart(_tx_dma, nullptr, nullptr, false);
	//rCR1 &= ~USART_CR1_TE;
	//rCR1 |= USART_CR1_TE;
	rCR3 |= USART_CR3_DMAT;

	perf_end(_pc_dmasetup);

	/* compute the deadline for a 10ms timeout */
	struct timespec abstime;
	clock_gettime(CLOCK_REALTIME, &abstime);
	abstime.tv_nsec += 10*1000*1000;
	if (abstime.tv_nsec >= 1000*1000*1000) {
		abstime.tv_sec++;
		abstime.tv_nsec -= 1000*1000*1000;
	}

	/* wait for the transaction to complete - 64 bytes @ 1.5Mbps ~426µs */
	int ret;
	for (;;) {
		ret = sem_timedwait(&_completion_semaphore, &abstime);

		if (ret == OK) {
			/* check for DMA errors */
			if (_rx_dma_status & DMA_STATUS_TEIF) {
				perf_count(_pc_dmaerrs);
				ret = -EIO;
				break;
			}

			/* check packet CRC - corrupt packet errors mean IO receive CRC error */
			uint8_t crc = _dma_buffer.crc;
			_dma_buffer.crc = 0;
			if ((crc != crc_packet(&_dma_buffer)) | (PKT_CODE(_dma_buffer) == PKT_CODE_CORRUPT)) {
				perf_count(_pc_crcerrs);
				ret = -EIO;
				break;
			}

			/* successful txn (may still be reporting an error) */
			break;
		}

		if (errno == ETIMEDOUT) {
			/* something has broken - clear out any partial DMA state and reconfigure */
			_abort_dma();
			perf_count(_pc_timeouts);
			perf_cancel(_pc_txns);		/* don't count this as a transaction */
			break;
		}

		/* we might? see this for EINTR */
		lowsyslog("unexpected ret %d/%d\n", ret, errno);
	}

	/* reset DMA status */
	_rx_dma_status = _dma_status_inactive;

	/* update counters */
	perf_end(_pc_txns);

	return ret;
}

void
PX4IO_serial::_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	if (arg != nullptr) {
		PX4IO_serial *ps = reinterpret_cast<PX4IO_serial *>(arg);

		ps->_do_rx_dma_callback(status);
	}
}

void
PX4IO_serial::_do_rx_dma_callback(unsigned status)
{
	/* on completion of a reply, wake the waiter */
	if (_rx_dma_status == _dma_status_waiting) {

		/* check for packet overrun - this will occur after DMA completes */
		uint32_t sr = rSR;
		if (sr & (USART_SR_ORE | USART_SR_RXNE)) {
			(void)rDR;
			status = DMA_STATUS_TEIF;
		}

		/* save RX status */
		_rx_dma_status = status;

		/* disable UART DMA */
		rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

		/* complete now */
		sem_post(&_completion_semaphore);
	}
}

int
PX4IO_serial::_interrupt(int irq, void *context)
{
	if (g_interface != nullptr)
		g_interface->_do_interrupt();
	return 0;
}

void
PX4IO_serial::_do_interrupt()
{
	uint32_t sr = rSR;	/* get UART status register */
	(void)rDR;		/* read DR to clear status */

	if (sr & (USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
		USART_SR_NE |		/* noise error - we have lost a byte due to noise */
		USART_SR_FE)) {		/* framing error - start/stop bit lost or line break */
		
		/* 
		 * If we are in the process of listening for something, these are all fatal;
		 * abort the DMA with an error.
		 */
		if (_rx_dma_status == _dma_status_waiting) {
			_abort_dma();

			perf_count(_pc_uerrs);
			/* complete DMA as though in error */
			_do_rx_dma_callback(DMA_STATUS_TEIF);

			return;
		}

		/* XXX we might want to use FE / line break as an out-of-band handshake ... handle it here */

		/* don't attempt to handle IDLE if it's set - things went bad */
		return;
	}

	if (sr & USART_SR_IDLE) {

		/* if there is DMA reception going on, this is a short packet */
		if (_rx_dma_status == _dma_status_waiting) {

			/* verify that the received packet is complete */
			size_t length = sizeof(_dma_buffer) - stm32_dmaresidual(_rx_dma);
			if ((length < 1) || (length < PKT_SIZE(_dma_buffer))) {
				perf_count(_pc_badidle);

				/* stop the receive DMA */
				stm32_dmastop(_rx_dma);

				/* complete the short reception */
				_do_rx_dma_callback(DMA_STATUS_TEIF);
				return;
			}

			perf_count(_pc_idle);

			/* stop the receive DMA */
			stm32_dmastop(_rx_dma);

			/* complete the short reception */
			_do_rx_dma_callback(DMA_STATUS_TCIF);
		}
	}
}

void
PX4IO_serial::_abort_dma()
{
	/* disable UART DMA */
	rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
	(void)rSR;
	(void)rDR;
	(void)rDR;

	/* stop DMA */
	stm32_dmastop(_tx_dma);
	stm32_dmastop(_rx_dma);
}

#endif /* PX4IO_SERIAL_BASE */
