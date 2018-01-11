/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file px4io_driver.h
 *
 * Interface for PX4IO
 */

#pragma once

/* XXX trim includes */
#include <px4_config.h>
#include <px4_posix.h>

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
#include <systemlib/perf_counter.h>

#include <board_config.h>

#include <drivers/device/device.h>
#include <modules/px4iofirmware/protocol.h>

class PX4IO_serial : public device::Device
{
public:
	PX4IO_serial();
	virtual ~PX4IO_serial();

	virtual int	init() = 0;
	virtual int	read(unsigned offset, void *data, unsigned count = 1);
	virtual int	write(unsigned address, void *data, unsigned count = 1);

protected:
	/**
	 * Does the PX4IO_serial instance initialization.
	 * @param io_buffer The IO buffer that should be used for transfers.
	 * @return 0 on success.
	 */
	int		init(IOPacket *io_buffer);

	/**
	 * Start the transaction with IO and wait for it to complete.
	 */
	virtual int	_bus_exchange(IOPacket *_packet) = 0;

	/**
	 * Performance counters.
	 */
	perf_counter_t		_pc_txns;
	perf_counter_t		_pc_retries;
	perf_counter_t		_pc_timeouts;
	perf_counter_t		_pc_crcerrs;
	perf_counter_t		_pc_protoerrs;
	perf_counter_t		_pc_uerrs;
	perf_counter_t		_pc_idle;
	perf_counter_t		_pc_badidle;
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
	IOPacket		*_io_buffer_ptr;

	/** bus-ownership lock */
	px4_sem_t			_bus_semaphore;

	/* do not allow top copying this class */
	PX4IO_serial(PX4IO_serial &);
	PX4IO_serial &operator = (const PX4IO_serial &);
};

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F4XXX)
/** XXX use F4 implementation for F1 as well. **/

#define PX4IO_INTERFACE_CLASS PX4IO_serial_f4
#define PX4IO_INTERFACE_F4

class PX4IO_serial_f4 : public PX4IO_serial
{
public:
	PX4IO_serial_f4();
	~PX4IO_serial_f4();

	virtual int	init();
	virtual int	ioctl(unsigned operation, unsigned &arg);

protected:
	/**
	 * Start the transaction with IO and wait for it to complete.
	 */
	int		_bus_exchange(IOPacket *_packet);

private:
	DMA_HANDLE		_tx_dma;
	DMA_HANDLE		_rx_dma;

	IOPacket *_current_packet;

	/** saved DMA status */
	static const unsigned	_dma_status_inactive = 0x80000000;	// low bits overlap DMA_STATUS_* values
	static const unsigned	_dma_status_waiting  = 0x00000000;
	volatile unsigned	_rx_dma_status;

	/** client-waiting lock/signal */
	px4_sem_t			_completion_semaphore;

	/**
	 * DMA completion handler.
	 */
	static void		_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
	void			_do_rx_dma_callback(unsigned status);

	/**
	 * Serial interrupt handler.
	 */
	static int		_interrupt(int vector, void *context, void *arg);
	void			_do_interrupt();

	/**
	 * Cancel any DMA in progress with an error.
	 */
	void			_abort_dma();

	/**
	 * Performance counters.
	 */
	perf_counter_t		_pc_dmasetup;
	perf_counter_t		_pc_dmaerrs;

	/* do not allow top copying this class */
	PX4IO_serial_f4(PX4IO_serial_f4 &);
	PX4IO_serial_f4 &operator = (const PX4IO_serial_f4 &);

	/**
	 * IO Buffer storage
	 */
	static IOPacket		_io_buffer_storage;		// XXX static to ensure DMA-able memory
};

#elif defined(CONFIG_ARCH_CHIP_STM32F7)

#define PX4IO_INTERFACE_CLASS PX4IO_serial_f7
#define PX4IO_INTERFACE_F7

#include <stm32_dma.h>


class PX4IO_serial_f7 : public PX4IO_serial
{
public:
	PX4IO_serial_f7();
	~PX4IO_serial_f7();

	virtual int	init();
	virtual int	ioctl(unsigned operation, unsigned &arg);

protected:
	/**
	 * Start the transaction with IO and wait for it to complete.
	 */
	int		_bus_exchange(IOPacket *_packet);

private:
	DMA_HANDLE		_tx_dma;
	DMA_HANDLE		_rx_dma;

	IOPacket *_current_packet;

	/** saved DMA status */
	static const unsigned	_dma_status_inactive = 0x80000000;	// low bits overlap DMA_STATUS_* values
	static const unsigned	_dma_status_waiting  = 0x00000000;
	volatile unsigned	_rx_dma_status;

	/** client-waiting lock/signal */
	px4_sem_t			_completion_semaphore;

	/**
	 * DMA completion handler.
	 */
	static void		_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
	void			_do_rx_dma_callback(unsigned status);

	/**
	 * Serial interrupt handler.
	 */
	static int		_interrupt(int vector, void *context, void *arg);
	void			_do_interrupt();

	/**
	 * Cancel any DMA in progress with an error.
	 */
	void			_abort_dma();

	/**
	 * Performance counters.
	 */
	perf_counter_t		_pc_dmasetup;
	perf_counter_t		_pc_dmaerrs;

	/* do not allow top copying this class */
	PX4IO_serial_f7(PX4IO_serial_f7 &);
	PX4IO_serial_f7 &operator = (const PX4IO_serial_f7 &);

	/**
	 * IO Buffer storage
	 */
	static uint8_t _io_buffer_storage[];
};

#else
#error "Interface not implemented for this chip"
#endif
