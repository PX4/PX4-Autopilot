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

#include <systemlib/hx_stream.h>

#include "interface.h"

class PX4IO_serial : public PX4IO_interface
{
public:
	PX4IO_serial(int port);
	virtual ~PX4IO_serial();

	virtual int	set_reg(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);
	virtual int	get_reg(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	virtual bool	ok();

private:
	volatile uint32_t	*_serial_base;
	int			_vector;

	uint8_t			*_tx_buf;
	unsigned		_tx_size;

	const uint8_t		*_rx_buf;
	unsigned		_rx_size;

	hx_stream_t		_stream;

	sem_t			_bus_semaphore;
	sem_t			_completion_semaphore;

	/**
	 * Send _tx_size bytes from the buffer, then
	 * if _rx_size is greater than zero wait for a packet
	 * to come back.
	 */
	int			_wait_complete();

	/**
	 * Interrupt handler.
	 */
	static int		_interrupt(int irq, void *context);
	void			_do_interrupt();

	/**
	 * Stream transmit callback
	 */
	static void		_tx(void *arg, uint8_t data);
	void			_do_tx(uint8_t data);

	/**
	 * Stream receive callback
	 */
	static void		_rx(void *arg, const void *data, size_t length);
	void			_do_rx(const uint8_t *data, size_t length);

	/**
	 * Serial register accessors.
	 */
	volatile uint32_t	&_sreg(unsigned offset)
	{
		return *(_serial_base + (offset / sizeof(uint32_t)));
	}
	volatile uint32_t	&_SR()   { return _sreg(STM32_USART_SR_OFFSET);   }
	volatile uint32_t	&_DR()   { return _sreg(STM32_USART_DR_OFFSET);   }
	volatile uint32_t	&_BRR()  { return _sreg(STM32_USART_BRR_OFFSET);  }
	volatile uint32_t	&_CR1()  { return _sreg(STM32_USART_CR1_OFFSET);  }
	volatile uint32_t	&_CR2()  { return _sreg(STM32_USART_CR2_OFFSET);  }
	volatile uint32_t	&_CR3()  { return _sreg(STM32_USART_CR3_OFFSET);  }
	volatile uint32_t	&_GTPR() { return _sreg(STM32_USART_GTPR_OFFSET); }
};

/* XXX hack to avoid expensive IRQ lookup */
static PX4IO_serial *io_serial;

PX4IO_interface	*io_serial_interface(int port)
{
	return new PX4IO_serial(port);
}

PX4IO_serial::PX4IO_serial(int port) :
	_serial_base(0),
	_vector(0),
	_tx_buf(nullptr),
	_tx_size(0),
	_rx_size(0),
	_stream(0)
{
	/* only allow one instance */
	if (io_serial != nullptr)
		return;

	switch (port) {
	case 5:
		_serial_base = (volatile uint32_t *)STM32_UART5_BASE;
		_vector = STM32_IRQ_UART5;
		break;
	default:
		/* not a supported port */
		return;
	}

	/* need space for worst-case escapes + hx protocol overhead */
	/* XXX this is kinda gross, but hx transmits a byte at a time */
	_tx_buf = new uint8_t[HX_STREAM_MAX_FRAME];

	irq_attach(_vector, &_interrupt);

	_stream = hx_stream_init(-1, _rx, this);

	sem_init(&_completion_semaphore, 0, 0);
	sem_init(&_bus_semaphore, 0, 1);
}

PX4IO_serial::~PX4IO_serial()
{

	if (_tx_buf != nullptr)
		delete[] _tx_buf;

	if (_vector)
		irq_detach(_vector);

	if (io_serial == this)
		io_serial = nullptr;

	if (_stream)
		hx_stream_free(_stream);

	sem_destroy(&_completion_semaphore);
	sem_destroy(&_bus_semaphore);
}

bool
PX4IO_serial::ok()
{
	if (_serial_base == 0)
		return false;
	if (_vector == 0)
		return false;
	if (_tx_buf == nullptr)
		return false;
	if (!_stream)
		return false;

	return true;
}

int
PX4IO_serial::set_reg(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{

	unsigned count = num_values * sizeof(*values);
	if (count > (HX_STREAM_MAX_FRAME - 2))
		return -EINVAL;

	sem_wait(&_bus_semaphore);

	_tx_buf[0] = page;
	_tx_buf[1] = offset;
	memcpy(&_tx_buf[2], (void *)values, count);

	_tx_size = count + 2;
	_rx_size = 0;

	/* start the transaction and wait for it to complete */
	int result = _wait_complete();

	sem_post(&_bus_semaphore);
	return result;
}

int
PX4IO_serial::get_reg(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{

	unsigned count = num_values * sizeof(*values);
	if (count > HX_STREAM_MAX_FRAME)
		return -EINVAL;

	sem_wait(&_bus_semaphore);

	_tx_buf[0] = page;
	_tx_buf[1] = offset;
	_tx_buf[2] = num_values;

	_tx_size = 3;		/* this tells IO that this is a read request */
	_rx_size = count;

	/* start the transaction and wait for it to complete */
	int result = _wait_complete();
	if (result != OK)
		goto out;

	/* compare the received count with the expected count */
	if (_rx_size != count) {
		return -EIO;
	} else {
		/* copy back the result */
		memcpy(values, &_tx_buf[0], count);
	}
out:
	sem_post(&_bus_semaphore);
	return OK;
}

int
PX4IO_serial::_wait_complete()
{
	/* prepare the stream for transmission */
	hx_stream_reset(_stream);
	hx_stream_start(_stream, _tx_buf, _tx_size);

	/* enable UART */
	_CR1() |= USART_CR1_RE |
		  USART_CR1_TE |
		  USART_CR1_TXEIE |
		  USART_CR1_RXNEIE |
		  USART_CR1_UE;

	/* compute the deadline for a 5ms timeout */
	struct timespec abstime;
	clock_gettime(CLOCK_REALTIME, &abstime);
	abstime.tv_nsec += 5000000;	/* 5ms timeout */
	while (abstime.tv_nsec > 1000000000) {
		abstime.tv_sec++;
		abstime.tv_nsec -= 1000000000;
	}

	/* wait for the transaction to complete */
	int ret = sem_timedwait(&_completion_semaphore, &abstime);

	/* disable the UART */
	_CR1() &= ~(USART_CR1_RE |
		    USART_CR1_TE |
		    USART_CR1_TXEIE |
		    USART_CR1_RXNEIE |
		    USART_CR1_UE);

	return ret;
}

int
PX4IO_serial::_interrupt(int irq, void *context)
{
	/* ... because NuttX doesn't give us a handle per vector */
	io_serial->_do_interrupt();
	return 0;
}

void
PX4IO_serial::_do_interrupt()
{
	uint32_t sr = _SR();

	/* handle transmit completion */
	if (sr & USART_SR_TXE) {
		int c = hx_stream_send_next(_stream);
		if (c == -1) {
			/* transmit (nearly) done, not interested in TX-ready interrupts now */
			_CR1() &= ~USART_CR1_TXEIE;

			/* was this a tx-only operation? */
			if (_rx_size == 0) {
				/* wake up waiting sender */
				sem_post(&_completion_semaphore);
			}
		} else {
			_DR() = c;
		}
	}

	if (sr & USART_SR_RXNE) {
		uint8_t c = _DR();

		hx_stream_rx(_stream, c);
	}
}

void
PX4IO_serial::_rx(void *arg, const void *data, size_t length)
{
	PX4IO_serial *pserial = reinterpret_cast<PX4IO_serial *>(arg);

	pserial->_do_rx((const uint8_t *)data, length);
}

void
PX4IO_serial::_do_rx(const uint8_t *data, size_t length)
{
	_rx_buf = data;

	if (length < _rx_size)
		_rx_size = length;

	/* notify waiting receiver */
	sem_post(&_completion_semaphore);
}



