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
  * @file interface_serial.cpp
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

#include <arch/board/board.h>

/* XXX might be able to prune these */
#include <nuttx/arch.h>
#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>
#include <stm32_internal.h>

#include <debug.h>

#include <drivers/drv_hrt.h>
#include <drivers/boards/px4fmuv2/px4fmu_internal.h>	/* XXX should really not be hardcoding v2 here */

#include <systemlib/perf_counter.h>

#include <modules/px4iofirmware/protocol.h>

#include "interface.h"

const unsigned	max_rw_regs = 32; // by agreement w/IO

#pragma pack(push, 1)
struct IOPacket {
	uint8_t 	count;
#define PKT_CTRL_WRITE			(1<<7)
	uint8_t 	spare;
	uint8_t 	page;
	uint8_t 	offset;
	uint16_t	regs[max_rw_regs];
};
#pragma pack(pop)

/* serial register accessors */
#define REG(_x)		(*(volatile uint32_t *)(PX4IO_SERIAL_BASE + _x))
#define rSR		REG(STM32_USART_SR_OFFSET)
#define rDR		REG(STM32_USART_DR_OFFSET)
#define rBRR		REG(STM32_USART_BRR_OFFSET)
#define rCR1		REG(STM32_USART_CR1_OFFSET)
#define rCR2		REG(STM32_USART_CR2_OFFSET)
#define rCR3		REG(STM32_USART_CR3_OFFSET)
#define rGTPR		REG(STM32_USART_GTPR_OFFSET)

#define PKT_SIZE(_n)	(4 + (2 * (_n)))

class PX4IO_serial : public PX4IO_interface
{
public:
	PX4IO_serial();
	virtual ~PX4IO_serial();

	virtual int	set_reg(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);
	virtual int	get_reg(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	virtual bool	ok();
	virtual int	test(unsigned mode);

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
	volatile unsigned	_rx_length;

	/** saved DMA status */
	static const unsigned	_dma_status_inactive = 0x80000000;	// low bits overlap DMA_STATUS_* values
	static const unsigned	_dma_status_waiting  = 0x00000000;
	volatile unsigned	_tx_dma_status;
	volatile unsigned	_rx_dma_status;

	/** bus-ownership lock */
	sem_t			_bus_semaphore;

	/** client-waiting lock/signal */
	sem_t			_completion_semaphore;

	/**
	 * Start the transaction with IO and wait for it to complete.
	 *
	 * @param expect_reply		If true, expect a reply from IO.
	 */
	int			_wait_complete(bool expect_reply, unsigned tx_length);

	/**
	 * DMA completion handler.
	 */
	static void		_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
	void			_do_tx_dma_callback(unsigned status);
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
	perf_counter_t		_perf_dmasetup;
	perf_counter_t		_perf_timeouts;
	perf_counter_t		_perf_errors;
	perf_counter_t		_perf_wr;
	perf_counter_t		_perf_rd;

};

IOPacket PX4IO_serial::_dma_buffer;
static PX4IO_serial *g_interface;

PX4IO_interface	*io_serial_interface()
{
	return new PX4IO_serial();
}

PX4IO_serial::PX4IO_serial() :
	_tx_dma(nullptr),
	_rx_dma(nullptr),
	_rx_length(0),
	_tx_dma_status(_dma_status_inactive),
	_rx_dma_status(_dma_status_inactive),
	_perf_dmasetup(perf_alloc(PC_ELAPSED,	"dmasetup")),
	_perf_timeouts(perf_alloc(PC_COUNT,	"timeouts")),
	_perf_errors(perf_alloc(PC_COUNT,	"errors  ")),
	_perf_wr(perf_alloc(PC_ELAPSED,		"writes  ")),
	_perf_rd(perf_alloc(PC_ELAPSED,		"reads   "))
{
	/* allocate DMA */
	_tx_dma = stm32_dmachannel(PX4IO_SERIAL_TX_DMAMAP);
	_rx_dma = stm32_dmachannel(PX4IO_SERIAL_RX_DMAMAP);
	if ((_tx_dma == nullptr) || (_rx_dma == nullptr))
		return;

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
	/* rCR3 = USART_CR3_EIE;*/
	rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE /*| USART_CR1_IDLEIE*/;

	/* create semaphores */
	sem_init(&_completion_semaphore, 0, 0);
	sem_init(&_bus_semaphore, 0, 1);

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

	if (g_interface == this)
		g_interface = nullptr;
}

bool
PX4IO_serial::ok()
{
	if (_tx_dma == nullptr)
		return false;
	if (_rx_dma == nullptr)
		return false;

	return true;
}

int
PX4IO_serial::test(unsigned mode)
{

	switch (mode) {
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
		lowsyslog("test 1\n");
		{
			for (unsigned count = 0;; count++) {
				uint16_t value = count & 0xffff;

				if (set_reg(PX4IO_PAGE_TEST, PX4IO_P_TEST_LED, &value, 1) != OK)
					return -2;
				if (count > 100) {
					perf_print_counter(_perf_dmasetup);
					perf_print_counter(_perf_wr);
					count = 0;
				}
				usleep(100000);
			}
			return 0;
		}
	case 2:
		lowsyslog("test 2\n");
		return 0;
	}
	return -1;
}

int
PX4IO_serial::set_reg(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	if (num_values > max_rw_regs)
		return -EINVAL;

	sem_wait(&_bus_semaphore);

	_dma_buffer.count = num_values | PKT_CTRL_WRITE;
	_dma_buffer.spare = 0;
	_dma_buffer.page = page;
	_dma_buffer.offset = offset;
	memcpy((void *)&_dma_buffer.regs[0], (void *)values, (2 * num_values));
	for (unsigned i = num_values; i < max_rw_regs; i++)
		_dma_buffer.regs[i] = 0x55aa;

	/* XXX implement check byte */

	/* start the transaction and wait for it to complete */
	int result = _wait_complete(false, PKT_SIZE(num_values));

	sem_post(&_bus_semaphore);
	return result;
}

int
PX4IO_serial::get_reg(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	if (num_values > max_rw_regs)
		return -EINVAL;

	sem_wait(&_bus_semaphore);

	_dma_buffer.count = num_values;
	_dma_buffer.spare = 0;
	_dma_buffer.page = page;
	_dma_buffer.offset = offset;

	/* start the transaction and wait for it to complete */
	int result = _wait_complete(true, PKT_SIZE(0));
	if (result != OK)
		goto out;

	/* compare the received count with the expected count */
	if (_dma_buffer.count != num_values) {
		return -EIO;
	} else {
		/* XXX implement check byte */
		/* copy back the result */
		memcpy(values, &_dma_buffer.regs[0], (2 * num_values));
	}
out:
	sem_post(&_bus_semaphore);
	return OK;
}

int
PX4IO_serial::_wait_complete(bool expect_reply, unsigned tx_length)
{

	/* clear any lingering error status */
	(void)rSR;
	(void)rDR;

	/* start RX DMA */
	if (expect_reply) {
		perf_begin(_perf_rd);
		perf_begin(_perf_dmasetup);

		/* DMA setup time ~3µs */
		_rx_dma_status = _dma_status_waiting;
		_rx_length = 0;
		stm32_dmasetup(
			_rx_dma,
			PX4IO_SERIAL_BASE + STM32_USART_DR_OFFSET,
			reinterpret_cast<uint32_t>(&_dma_buffer),
			sizeof(_dma_buffer),
			DMA_SCR_DIR_P2M		|
			DMA_SCR_MINC		|
			DMA_SCR_PSIZE_8BITS	|
			DMA_SCR_MSIZE_8BITS	|
			DMA_SCR_PBURST_SINGLE	|
			DMA_SCR_MBURST_SINGLE);
		stm32_dmastart(_rx_dma, _dma_callback, this, false);
		rCR3 |= USART_CR3_DMAR;

		perf_end(_perf_dmasetup);
	} else {
		perf_begin(_perf_wr);
	}

	/* start TX DMA - no callback if we also expect a reply */
	/* DMA setup time ~3µs */
	perf_begin(_perf_dmasetup);
	_tx_dma_status = _dma_status_waiting;
	stm32_dmasetup(
		_tx_dma,
		PX4IO_SERIAL_BASE + STM32_USART_DR_OFFSET,
		reinterpret_cast<uint32_t>(&_dma_buffer),
		sizeof(_dma_buffer),		/* XXX should be tx_length */
		DMA_SCR_DIR_M2P		|
		DMA_SCR_MINC		|
		DMA_SCR_PSIZE_8BITS	|
		DMA_SCR_MSIZE_8BITS	|
		DMA_SCR_PBURST_SINGLE	|
		DMA_SCR_MBURST_SINGLE);
	stm32_dmastart(_tx_dma, /*expect_reply ? nullptr :*/ _dma_callback, this, false);
	rCR3 |= USART_CR3_DMAT;

	perf_end(_perf_dmasetup);

	/* compute the deadline for a 5ms timeout */
	struct timespec abstime;
	clock_gettime(CLOCK_REALTIME, &abstime);
#if 1
	abstime.tv_sec++;		/* long timeout for testing */
#else
	abstime.tv_nsec += 5000000;	/* 5ms timeout */
	while (abstime.tv_nsec > 1000000000) {
		abstime.tv_sec++;
		abstime.tv_nsec -= 1000000000;
	}
#endif

	/* wait for the transaction to complete - 64 bytes @ 1.5Mbps ~426µs */
	int ret;
	for (;;) {
		ret = sem_timedwait(&_completion_semaphore, &abstime);

		if (ret == OK) {
			/* check for DMA errors */
			if (_tx_dma_status & DMA_STATUS_TEIF) {
				lowsyslog("DMA transmit error\n");
				ret = -1;
			}
			if (_rx_dma_status & DMA_STATUS_TEIF) {
				lowsyslog("DMA receive error\n");
				ret = -1;
			}
			break;
		}

		if (errno == ETIMEDOUT) {
			lowsyslog("timeout waiting for PX4IO link (%d/%d)\n", stm32_dmaresidual(_tx_dma), stm32_dmaresidual(_rx_dma));
			/* something has broken - clear out any partial DMA state and reconfigure */
			_abort_dma();
			perf_count(_perf_timeouts);
			break;
		}
		lowsyslog("unexpected ret %d/%d\n", ret, errno);
	}

	/* reset DMA status */
	_tx_dma_status = _dma_status_inactive;
	_rx_dma_status = _dma_status_inactive;

	/* update counters */
	perf_end(expect_reply ? _perf_rd : _perf_wr);
	if (ret != OK)
		perf_count(_perf_errors);

	return ret;
}

void
PX4IO_serial::_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	if (arg != nullptr) {
		PX4IO_serial *ps = reinterpret_cast<PX4IO_serial *>(arg);

		if (handle == ps->_tx_dma) {
			ps->_do_tx_dma_callback(status);
		} else if (handle == ps->_rx_dma) {
			ps->_do_rx_dma_callback(status);
		}
	}
}

void
PX4IO_serial::_do_tx_dma_callback(unsigned status)
{
	/* on completion of a no-reply transmit, wake the sender */
	if (_tx_dma_status == _dma_status_waiting) {

		/* save TX status */
		_tx_dma_status = status;

		/* disable UART DMA */
		rCR3 &= ~USART_CR3_DMAT;

		/* if we aren't going on to expect a reply, complete now */
		if (_rx_dma_status != _dma_status_waiting)
			sem_post(&_completion_semaphore);
	}
}

void
PX4IO_serial::_do_rx_dma_callback(unsigned status)
{
	ASSERT(_tx_dma_status != _dma_status_waiting);

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
		rCR3 &= ~USART_CR3_DMAR;

		/* DMA may have stopped short */
		_rx_length = sizeof(IOPacket) - stm32_dmaresidual(_rx_dma);

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

	/* handle error/exception conditions */
	if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_IDLE)) {
		/* read DR to clear status */
		(void)rDR;
	} else {
		ASSERT(0);
	}

	if (sr & (USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
		USART_SR_NE |		/* noise error - we have lost a byte due to noise */
		USART_SR_FE)) {		/* framing error - start/stop bit lost or line break */
		
		/* 
		 * If we are in the process of listening for something, these are all fatal;
		 * abort the DMA with an error.
		 */
		if (_rx_dma_status == _dma_status_waiting) {
			_abort_dma();
			return;
		}

		/* XXX we might want to use FE / line break as an out-of-band handshake ... handle it here */

		/* don't attempt to handle IDLE if it's set - things went bad */
		return;
	}

	if (sr & USART_SR_IDLE) {

		/* if there was DMA transmission still going, this is an error */
		if (_tx_dma_status == _dma_status_waiting) {

			/* babble from IO */
			_abort_dma();
			return;
		}

		/* if there is DMA reception going on, this is a short packet */
		if (_rx_dma_status == _dma_status_waiting) {

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

	/* complete DMA as though in error */
	_do_tx_dma_callback(DMA_STATUS_TEIF);
	_do_rx_dma_callback(DMA_STATUS_TEIF);

	_tx_dma_status = _dma_status_inactive;
	_rx_dma_status = _dma_status_inactive;
}