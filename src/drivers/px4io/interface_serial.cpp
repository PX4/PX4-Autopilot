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
#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>
#include <stm32_internal.h>

#include <debug.h>

#include <drivers/drv_hrt.h>
#include <drivers/boards/px4fmuv2/px4fmu_internal.h>	/* XXX should really not be hardcoding v2 here */

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

	/** set if we have started a transaction that expects a reply */
	bool			_expect_reply;

	/** saved DMA status (in case we care) */
	uint8_t			_dma_status;

	/** bus-ownership lock */
	sem_t			_bus_semaphore;

	/** client-waiting lock/signal */
	sem_t			_completion_semaphore;

	/**
	 * Start the transaction with IO and wait for it to complete.
	 *
	 * @param expect_reply		If true, expect a reply from IO.
	 */
	int			_wait_complete(bool expect_reply);

	/**
	 * DMA completion handler.
	 */
	static void		_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
	void			_do_dma_callback(DMA_HANDLE handle, uint8_t status);

	/**
	 * (re)configure the DMA
	 */
	void			_reset_dma();

	/**
	 * Serial register accessors.
	 */
	volatile uint32_t	&_sreg(unsigned offset)
	{
		return *(volatile uint32_t *)(PX4IO_SERIAL_BASE + offset);
	}
	uint32_t	_SR()   		{ return _sreg(STM32_USART_SR_OFFSET);   }
	void		_SR(uint32_t val)	{ _sreg(STM32_USART_SR_OFFSET) = val;    }
	uint32_t	_DR()   		{ return _sreg(STM32_USART_DR_OFFSET);   }
	void		_DR(uint32_t val)	{ _sreg(STM32_USART_DR_OFFSET) = val;    }
	uint32_t	_BRR()  		{ return _sreg(STM32_USART_BRR_OFFSET);  }
	void		_BRR(uint32_t val)	{ _sreg(STM32_USART_BRR_OFFSET) = val;   }
	uint32_t	_CR1()  		{ return _sreg(STM32_USART_CR1_OFFSET);  }
	void		_CR1(uint32_t val)	{ _sreg(STM32_USART_CR1_OFFSET) = val;   }
	uint32_t	_CR2()  		{ return _sreg(STM32_USART_CR2_OFFSET);  }
	void		_CR2(uint32_t val)	{ _sreg(STM32_USART_CR2_OFFSET) = val;   }
	uint32_t	_CR3()  		{ return _sreg(STM32_USART_CR3_OFFSET);  }
	void		_CR3(uint32_t val)	{ _sreg(STM32_USART_CR3_OFFSET) = val;   }
	uint32_t	_GTPR() 		{ return _sreg(STM32_USART_GTPR_OFFSET); }
	void		_GTPR(uint32_t val)	{ _sreg(STM32_USART_GTPR_OFFSET) = val;  }

};

IOPacket PX4IO_serial::_dma_buffer;

PX4IO_interface	*io_serial_interface()
{
	return new PX4IO_serial();
}

PX4IO_serial::PX4IO_serial() :
	_tx_dma(nullptr),
	_rx_dma(nullptr),
	_expect_reply(false)
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
	_CR1(0);
	_CR2(0);
	_CR3(0);

	/* configure line speed */
	uint32_t usartdiv32 = PX4IO_SERIAL_CLOCK / (PX4IO_SERIAL_BITRATE / 2);
	uint32_t mantissa = usartdiv32 >> 5;
	uint32_t fraction = (usartdiv32 - (mantissa << 5) + 1) >> 1;
	_BRR((mantissa << USART_BRR_MANT_SHIFT) | (fraction << USART_BRR_FRAC_SHIFT));

	/* enable UART and DMA but no interrupts */
	_CR3(USART_CR3_DMAR | USART_CR3_DMAT);
	_CR1(USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);

	/* configure DMA */
	_reset_dma();

	/* create semaphores */
	sem_init(&_completion_semaphore, 0, 0);
	sem_init(&_bus_semaphore, 0, 1);
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
	_CR1(0);
	_CR2(0);
	_CR3(0);

	/* restore the GPIOs */
	stm32_unconfiggpio(PX4IO_SERIAL_TX_GPIO);
	stm32_unconfiggpio(PX4IO_SERIAL_RX_GPIO);

	/* and kill our semaphores */
	sem_destroy(&_completion_semaphore);
	sem_destroy(&_bus_semaphore);
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
		_CR3(_CR3() & ~(USART_CR3_DMAR | USART_CR3_DMAT));

		for (;;) {
			while (!(_SR() & USART_SR_TXE))
				;
			_DR(0x55);
		}
		return 0;

	case 1:
		lowsyslog("test 1\n");
		{
			uint16_t value = 0x5555;
			for (;;) {
				if (set_reg(0x55, 0x55, &value, 1) != OK)
					return -2;
			}
			return 0;
		}
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
	/* XXX implement check byte */

	/* start the transaction and wait for it to complete */
	int result = _wait_complete(false);

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
	int result = _wait_complete(true);
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
PX4IO_serial::_wait_complete(bool expect_reply)
{

	/* save for callback use */
	_expect_reply = expect_reply;

	/* start RX DMA */
	if (expect_reply)
		stm32_dmastart(_rx_dma, _dma_callback, this, false);

	/* start TX DMA - no callback if we also expect a reply */
	stm32_dmastart(_tx_dma, /*expect_reply ? nullptr :*/ _dma_callback, this, false);

	/* compute the deadline for a 5ms timeout */
	struct timespec abstime;
	clock_gettime(CLOCK_REALTIME, &abstime);
#if 1
	abstime.tv_sec++;
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

		if (ret == OK)
			break;

		if (errno == ETIMEDOUT) {
			lowsyslog("timeout waiting for PX4IO link (%d/%d)\n", stm32_dmaresidual(_tx_dma), stm32_dmaresidual(_rx_dma));
			/* something has broken - clear out any partial DMA state and reconfigure */
			_reset_dma();
			break;
		}
		lowsyslog("unexpected ret %d/%d\n", ret, errno);
	}

	return ret;
}

void
PX4IO_serial::_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	if (arg != nullptr) {
		PX4IO_serial *ps = reinterpret_cast<PX4IO_serial *>(arg);

		ps->_do_dma_callback(handle, status);
	}
}

void
PX4IO_serial::_do_dma_callback(DMA_HANDLE handle, uint8_t status)
{
	/* on completion of a no-reply transmit, wake the sender */
	if (handle == _tx_dma) {
		if ((status & DMA_STATUS_TCIF) && !_expect_reply) {
			_dma_status = status;
			sem_post(&_completion_semaphore);
		} 
		/* XXX if we think we're going to see DMA errors, we should handle them here */
	}

	/* on completion of a reply, wake the waiter */
	if (handle == _rx_dma) {
		if (status & DMA_STATUS_TCIF) {
			/* XXX if we are worried about overrun/synch, check UART status here */
			_dma_status = status;
			sem_post(&_completion_semaphore);
		}
	}
}

void
PX4IO_serial::_reset_dma()
{
	stm32_dmastop(_tx_dma);
	stm32_dmastop(_rx_dma);

	stm32_dmasetup(
		_tx_dma,
		PX4IO_SERIAL_BASE + STM32_USART_DR_OFFSET,
		reinterpret_cast<uint32_t>(&_dma_buffer),
		sizeof(_dma_buffer),
		DMA_SCR_DIR_M2P		|
		DMA_SCR_MINC		|
		DMA_SCR_PSIZE_8BITS	|
		DMA_SCR_MSIZE_8BITS	|
		DMA_SCR_PBURST_SINGLE	|
		DMA_SCR_MBURST_SINGLE);
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
}