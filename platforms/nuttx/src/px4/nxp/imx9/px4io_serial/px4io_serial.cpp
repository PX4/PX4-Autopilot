/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
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
 * Serial interface for PX4IO on IMX9
 */

#include <px4_arch/px4io_serial.h>

#include <syslog.h>

#include <termios.h>
#include <debug.h>
#include <imx9_lowputc.h>
#include <hardware/imx9_lpuart.h>
#include <hardware/imx93/imx93_dmamux.h>

#define ERR_FLAGS_MASK (LPUART_STAT_PF | LPUART_STAT_FE | LPUART_STAT_NF | LPUART_STAT_OR)

#define DMA_BUFFER_MASK    (ARMV8A_DCACHE_LINESIZE - 1)
#define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)

uint8_t ArchPX4IOSerial::_io_buffer_storage[DMA_ALIGN_UP(sizeof(IOPacket))];

static const struct uart_config_s g_px4io_config = {
	.baud      = PX4IO_SERIAL_BITRATE,    /* Configured baud */
	.parity    = 0,                    /* 0=none, 1=odd, 2=even */
	.bits      = 8,                    /* Number of bits (5-9) */
	.stopbits2 = false,                /* true: Configure with 2 stop bits instead of 1 */
};

ArchPX4IOSerial::ArchPX4IOSerial() :
	_current_packet(nullptr),
	_recv_sem(SEM_INITIALIZER(0))
{
	px4_sem_setprotocol(&_recv_sem, SEM_PRIO_NONE);

	/* Allocate the DMA channels */

	_tx_dma = imx9_dmach_alloc(PX4IO_SERIAL_TX_DMAMAP, 0);
	_rx_dma = imx9_dmach_alloc(PX4IO_SERIAL_RX_DMAMAP, 0);
}

ArchPX4IOSerial::~ArchPX4IOSerial()
{
	/* Stop any ongoing DMA transfer */

	_stop_dma();

	/* Detach our interrupt handler */

	up_disable_irq(PX4IO_SERIAL_VECTOR);
	irq_detach(PX4IO_SERIAL_VECTOR);

	/* Free the DMA channels */

	imx9_dmach_free(_rx_dma);
	imx9_dmach_free(_tx_dma);
}

int
ArchPX4IOSerial::init()
{
	/* Initialize base implementation */

	int r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0]);

	if (r != 0) {
		return r;
	}

	/* Inititalize uart hw */

	int ret = imx9_lpuart_configure(PX4IO_SERIAL_BASE, PX4IO_SERIAL_NUM, &g_px4io_config);

	if (ret == OK) {
		/* Attach serial interrupt handler */

		irq_attach(PX4IO_SERIAL_VECTOR, _interrupt, this);
		up_enable_irq(PX4IO_SERIAL_VECTOR);

		/* Enable error and line idle interrupts */

		modreg32(LPUART_CTRL_PEIE | LPUART_CTRL_FEIE | LPUART_CTRL_NEIE | LPUART_CTRL_ORIE | LPUART_CTRL_ILIE,
			 LPUART_ALL_INTS,
			 PX4IO_SERIAL_BASE + IMX9_LPUART_CTRL_OFFSET);

		/* Enable RDMA and TDMA */

		modreg32(LPUART_BAUD_RDMAE | LPUART_BAUD_TDMAE, LPUART_BAUD_RDMAE | LPUART_BAUD_TDMAE,
			 PX4IO_SERIAL_BASE + IMX9_LPUART_BAUD_OFFSET);
	}

	return ret;
}

int
ArchPX4IOSerial::ioctl(unsigned operation, unsigned &arg)
{
	int ret = 0;

	switch (operation) {

	case 1:		/* XXX magic number - test operation */
		switch (arg) {
		case 0:
			syslog(LOG_INFO, "test 0\n");

			while ((getreg32(PX4IO_SERIAL_BASE + IMX9_LPUART_STAT_OFFSET) &
				LPUART_STAT_TDRE) == 0) {
			}

			putreg32(0x55, PX4IO_SERIAL_BASE + IMX9_LPUART_DATA_OFFSET);

			break;

		case 1: {
				unsigned fails = 0;

				for (unsigned count = 0;; count++) {
					uint16_t value = count & 0xffff;

					if (write((PX4IO_PAGE_TEST << 8) | PX4IO_P_TEST_LED, &value, 1) != 0) {
						fails++;
					}

					if (count >= 5000) {
						syslog(LOG_INFO, "==== test 1 : %u failures ====\n", fails);
						perf_print_counter(_pc_txns);
						perf_print_counter(_pc_retries);
						perf_print_counter(_pc_timeouts);
						perf_print_counter(_pc_crcerrs);
						perf_print_counter(_pc_protoerrs);
						perf_print_counter(_pc_uerrs);
						count = 0;
					}
				}
			}
			break;

		case 2:
			syslog(LOG_INFO, "test 2\n");
			break;

		default:
			break;
		}

		break;

	default:
		ret = -1;
		break;
	}

	return ret;
}

int
ArchPX4IOSerial::_bus_exchange(IOPacket *_packet)
{
	int ret = OK;

	size_t send_size = PKT_SIZE(*_packet);

	_current_packet = _packet;

	/* Check packet size */

	if (send_size > sizeof(*_current_packet)) {
		perf_count(_pc_protoerrs);
		return -EINVAL;
	}

	/* Measure exchange time */

	perf_begin(_pc_txns);

#ifndef CONFIG_ARM64_DCACHE_DISABLE

	/* Make sure the tx/rx buffer area is all clean */

	up_clean_dcache((uintptr_t)_current_packet,
			(uintptr_t)_current_packet + DMA_ALIGN_UP(sizeof(IOPacket)));
#endif

	/* Start the RX DMA */

	_waiting_for_dma = true;

	struct imx9_edma_xfrconfig_s rx_config;
	rx_config.saddr  = PX4IO_SERIAL_BASE + IMX9_LPUART_DATA_OFFSET;
	rx_config.daddr  = reinterpret_cast<uintptr_t>(_current_packet);
	rx_config.soff   = 0;
	rx_config.doff   = 1;
	rx_config.iter   = sizeof(*_current_packet);
	rx_config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
	rx_config.ssize  = EDMA_8BIT;
	rx_config.dsize  = EDMA_8BIT;
	rx_config.nbytes = 1;
#ifdef CONFIG_IMX9_EDMA_ELINK
	rx_config.linkch = NULL;
#endif

	imx9_dmach_xfrsetup(_rx_dma, &rx_config);
	imx9_dmach_start(_rx_dma, _dma_callback, (void *)this);

	/* Start the TX DMA */

	struct imx9_edma_xfrconfig_s tx_config;

	tx_config.saddr  = reinterpret_cast<uintptr_t>(_current_packet);
	tx_config.daddr  = PX4IO_SERIAL_BASE + IMX9_LPUART_DATA_OFFSET;
	tx_config.soff   = 1;
	tx_config.doff   = 0;
	tx_config.iter   = send_size;
	tx_config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
	tx_config.ssize  = EDMA_8BIT;
	tx_config.dsize  = EDMA_8BIT;
	tx_config.nbytes = 1;
#ifdef CONFIG_IMX9_EDMA_ELINK
	tx_config.linkch = NULL;
#endif

	imx9_dmach_xfrsetup(_tx_dma, &tx_config);
	imx9_dmach_start(_tx_dma, nullptr, nullptr);

	/* Wait for response, max 10 ms */

	ret = nxsem_tickwait_uninterruptible(&_recv_sem, MSEC2TICK(10));

	if (ret == -ETIMEDOUT) {
		_stop_dma();
		nxsem_reset(&_recv_sem, 0);
		perf_count(_pc_timeouts);
		perf_cancel(_pc_txns); /* don't count this as a transaction */
	}

	if (ret == OK) {
		/* Check packet CRC */

		uint8_t crc = _current_packet->crc;
		_current_packet->crc = 0;

		if ((crc != crc_packet(_current_packet)) || (PKT_CODE(*_current_packet) == PKT_CODE_CORRUPT)) {
			perf_count(_pc_crcerrs);
			ret = -EIO;
		}
	}

	perf_end(_pc_txns);

	return ret;
}

void
ArchPX4IOSerial::_dma_callback(DMACH_HANDLE handle, void *arg, bool done, int result)
{
	if (arg != nullptr) {
		ArchPX4IOSerial *_this = reinterpret_cast<ArchPX4IOSerial *>(arg);

		_this->_rx_dma_callback();
	}
}

void
ArchPX4IOSerial::_rx_dma_callback()
{
	/* Bail out if not expecting a callback */

	if (!_waiting_for_dma) {
		return;
	}

	/* invalidate cache to make received data available */

	up_invalidate_dcache((uintptr_t)_current_packet,
			     (uintptr_t)_current_packet + DMA_ALIGN_UP(sizeof(IOPacket)));


	/* complete now */

	_waiting_for_dma = false;
	nxsem_post(&_recv_sem);
}

int
ArchPX4IOSerial::_interrupt(int irq, void *context, void *arg)
{
	if (arg != nullptr) {
		ArchPX4IOSerial *instance = reinterpret_cast<ArchPX4IOSerial *>(arg);

		instance->_do_interrupt();
	}

	return 0;
}

void
ArchPX4IOSerial::_do_interrupt()
{
	/* Read out and clear any error status flags */

	uint32_t status = getreg32(PX4IO_SERIAL_BASE + IMX9_LPUART_STAT_OFFSET);
	modreg32(status, ERR_FLAGS_MASK, PX4IO_SERIAL_BASE + IMX9_LPUART_STAT_OFFSET);

	if (status & LPUART_STAT_IDLE) {
		if (_waiting_for_dma) {
			perf_count(_pc_idle);

		} else {
			perf_count(_pc_badidle);
		}

		/* If this was a short packet, dma is still running and stopping it will
		 * call the callback function. Stopping a non-running dma has no effect.
		 */

		imx9_dmach_stop(_rx_dma);
	}

	/* count uart errors */

	if (status & ERR_FLAGS_MASK) {
		perf_count(_pc_uerrs);
	}
}

void
ArchPX4IOSerial::_stop_dma()
{
	/* Mark that we don't care about any DMA data any more */

	_waiting_for_dma = false;

	/* Stop the DMA channels */

	imx9_dmach_stop(_tx_dma);
	imx9_dmach_stop(_rx_dma);

	/* Read out any remaining data from rx fifo */

	while (getreg32(PX4IO_SERIAL_BASE + IMX9_LPUART_STAT_OFFSET) & LPUART_STAT_RDRF) {
		getreg32(PX4IO_SERIAL_BASE + IMX9_LPUART_DATA_OFFSET);
	}

	/* Clear any error status flags */

	modreg32(ERR_FLAGS_MASK, ERR_FLAGS_MASK, PX4IO_SERIAL_BASE + IMX9_LPUART_STAT_OFFSET);
}
