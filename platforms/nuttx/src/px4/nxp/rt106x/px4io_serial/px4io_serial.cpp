/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * Serial interface for PX4IO on RT1062
 */

#include <syslog.h>

#include <px4_arch/px4io_serial.h>
#include <nuttx/cache.h>
#include "hardware/imxrt_lpuart.h"
#include "hardware/imxrt_dmamux.h"
#include "imxrt_lowputc.h"
#include "imxrt_edma.h"
#include "imxrt_periphclks.h"


/* serial register accessors */
#define REG(_x)   (*(volatile uint32_t *)(PX4IO_SERIAL_BASE + (_x)))
#define rBAUD   REG(IMXRT_LPUART_BAUD_OFFSET)
#define rSTAT_ERR_FLAGS_MASK (LPUART_STAT_PF | LPUART_STAT_FE | LPUART_STAT_NF | LPUART_STAT_OR)
#define rSTAT   REG(IMXRT_LPUART_STAT_OFFSET)
#define rCTRL   REG(IMXRT_LPUART_CTRL_OFFSET)
#define rDATA   REG(IMXRT_LPUART_DATA_OFFSET)

#define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)

uint8_t ArchPX4IOSerial::_io_buffer_storage[DMA_ALIGN_UP(sizeof(IOPacket))];

ArchPX4IOSerial::ArchPX4IOSerial() :
	_tx_dma(nullptr),
	_rx_dma(nullptr),
	_current_packet(nullptr),
	_rx_dma_result(_dma_status_inactive),
	_completion_semaphore(SEM_INITIALIZER(0)),
	_pc_dmaerrs(perf_alloc(PC_COUNT, MODULE_NAME": DMA errors"))
{
}

ArchPX4IOSerial::~ArchPX4IOSerial()
{
	if (_tx_dma != nullptr) {
		imxrt_dmach_stop(_tx_dma);
		imxrt_dmach_free(_tx_dma);
	}

	if (_rx_dma != nullptr) {
		imxrt_dmach_stop(_rx_dma);
		imxrt_dmach_free(_rx_dma);
	}

	/* reset the UART */
	rCTRL = 0;

	/* detach our interrupt handler */
	up_disable_irq(PX4IO_SERIAL_VECTOR);
	irq_detach(PX4IO_SERIAL_VECTOR);

	/* restore the GPIOs */
	px4_arch_unconfiggpio(PX4IO_SERIAL_TX_GPIO);
	px4_arch_unconfiggpio(PX4IO_SERIAL_RX_GPIO);

	/* Disable clock for the USART peripheral */
	PX4IO_SERIAL_CLOCK_OFF();

	/* and kill our semaphores */
	px4_sem_destroy(&_completion_semaphore);

	perf_free(_pc_dmaerrs);
}

int
ArchPX4IOSerial::init()
{
	/* initialize base implementation */
	int r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0]);

	if (r != 0) {
		return r;
	}

	/* allocate DMA */
	_tx_dma = imxrt_dmach_alloc(PX4IO_SERIAL_TX_DMAMAP | DMAMUX_CHCFG_ENBL, 0);
	_rx_dma = imxrt_dmach_alloc(PX4IO_SERIAL_RX_DMAMAP | DMAMUX_CHCFG_ENBL, 0);

	if ((_tx_dma == nullptr) || (_rx_dma == nullptr)) {
		return -1;
	}

	struct uart_config_s config = {
		.baud = PX4IO_SERIAL_BITRATE,
		.parity = 0,        /* 0=none, 1=odd, 2=even */
		.bits = 8,          /* Number of bits (5-9) */
		.stopbits2 = false, /* true: Configure with 2 stop bits instead of 1 */
		.userts = false,    /* True: Assert RTS when there are data to be sent */
		.invrts = false,    /* True: Invert sense of RTS pin (true=active high) */
		.usects = false,   /* True: Condition transmission on CTS asserted */
		.users485 = false, /* True: Assert RTS while transmission progresses */
	};


	int rv = imxrt_lpuart_configure(PX4IO_SERIAL_BASE, &config);

	if (rv == OK) {
		/* configure pins for serial use */
		px4_arch_configgpio(PX4IO_SERIAL_TX_GPIO);
		px4_arch_configgpio(PX4IO_SERIAL_RX_GPIO);

		/* attach serial interrupt handler */
		irq_attach(PX4IO_SERIAL_VECTOR, _interrupt, this);
		up_enable_irq(PX4IO_SERIAL_VECTOR);

		/* Idel after Stop,   , enable error and line idle interrupts */
		uint32_t regval = rCTRL;
		regval &= ~(LPUART_CTRL_IDLECFG_MASK | LPUART_CTRL_ILT);
		regval |=  LPUART_CTRL_ILT | LPUART_CTRL_IDLECFG_1 | LPUART_CTRL_ILIE |
			   LPUART_CTRL_RE  | LPUART_CTRL_TE;
		rCTRL = regval;

		/* create semaphores */
		px4_sem_init(&_completion_semaphore, 0, 0);

		/* _completion_semaphore use case is a signal */

		px4_sem_setprotocol(&_completion_semaphore, SEM_PRIO_NONE);

		/* XXX this could try talking to IO */
	}

	return rv;
}

int
ArchPX4IOSerial::ioctl(unsigned operation, unsigned &arg)
{
	switch (operation) {

	case 1:   /* XXX magic number - test operation */
		switch (arg) {
		case 0:
			syslog(LOG_INFO, "test 0\n");

			/* kill DMA, this is a PIO test */
			imxrt_dmach_stop(_tx_dma);
			imxrt_dmach_stop(_rx_dma);

			/* disable UART DMA */

			rBAUD &= ~(LPUART_BAUD_RDMAE | LPUART_BAUD_TDMAE);

			for (;;) {
				while (!(rSTAT & LPUART_STAT_TDRE))
					;

				rDATA = 0x55;
			}

			return 0;

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
			syslog(LOG_INFO, "test 2\n");
			return 0;
		}

	default:
		break;
	}

	return -1;
}

int
ArchPX4IOSerial::_bus_exchange(IOPacket *_packet)
{
	// to be paranoid ensure all previous DMA transfers are cleared
	_abort_dma();

	_current_packet = _packet;

	/* clear data that may be in the RDR and clear overrun error: */
	while (rSTAT & LPUART_STAT_RDRF) {
		(void)rDATA;
	}

	rSTAT |= (rSTAT_ERR_FLAGS_MASK | LPUART_STAT_IDLE);  /* clear the flags */

	/* start RX DMA */
	perf_begin(_pc_txns);

	/* DMA setup time ~3µs */
	_rx_dma_result = _dma_status_waiting;

	struct imxrt_edma_xfrconfig_s rx_config;
	rx_config.saddr  = PX4IO_SERIAL_BASE + IMXRT_LPUART_DATA_OFFSET;
	rx_config.daddr  = reinterpret_cast<uint32_t>(_current_packet);
	rx_config.soff   = 0;
	rx_config.doff   = 1;
	rx_config.iter   = sizeof(*_current_packet);
	rx_config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
	rx_config.ssize  = EDMA_8BIT;
	rx_config.dsize  = EDMA_8BIT;
	rx_config.nbytes = 1;
#ifdef CONFIG_IMXRT_EDMA_ELINK
	rx_config.linkch = NULL;
#endif
	imxrt_dmach_xfrsetup(_rx_dma, &rx_config);

	/* Enable receive DMA for the UART */

	rBAUD |= LPUART_BAUD_RDMAE;

	imxrt_dmach_start(_rx_dma, _dma_callback, (void *)this);

	/* Clean _current_packet, so DMA can see the data */
	up_clean_dcache((uintptr_t)_current_packet,
			(uintptr_t)_current_packet + DMA_ALIGN_UP(sizeof(IOPacket)));

	/* start TX DMA - no callback if we also expect a reply */
	/* DMA setup time ~3µs */

	struct imxrt_edma_xfrconfig_s tx_config;
	tx_config.saddr  = reinterpret_cast<uint32_t>(_current_packet);
	tx_config.daddr  = PX4IO_SERIAL_BASE + IMXRT_LPUART_DATA_OFFSET;
	tx_config.soff   = 1;
	tx_config.doff   = 0;
	tx_config.iter   = sizeof(*_current_packet);
	tx_config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
	tx_config.ssize  = EDMA_8BIT;
	tx_config.dsize  = EDMA_8BIT;
	tx_config.nbytes = 1;
#ifdef CONFIG_IMXRT_EDMA_ELINK
	tx_config.linkch = NULL;
#endif
	imxrt_dmach_xfrsetup(_tx_dma, &tx_config);


	/* Enable transmit DMA for the UART */

	rBAUD |= LPUART_BAUD_TDMAE;

	imxrt_dmach_start(_tx_dma, nullptr, nullptr);

	/* compute the deadline for a 10ms timeout */
	struct timespec abstime;
	clock_gettime(CLOCK_REALTIME, &abstime);
	abstime.tv_nsec += 10 * 1000 * 1000;

	if (abstime.tv_nsec >= 1000 * 1000 * 1000) {
		abstime.tv_sec++;
		abstime.tv_nsec -= 1000 * 1000 * 1000;
	}

	/* wait for the transaction to complete - 64 bytes @ 1.5Mbps ~426µs */
	int ret;

	for (;;) {
		ret = sem_timedwait(&_completion_semaphore, &abstime);

		if (ret == OK) {
			/* check for DMA errors */
			if (_rx_dma_result != OK) {
				// stream transfer error, ensure all DMA is also stopped before exiting early
				_abort_dma();
				perf_count(_pc_dmaerrs);
				ret = -EIO;
				break;
			}

			/* check packet CRC - corrupt packet errors mean IO receive CRC error */
			uint8_t crc = _current_packet->crc;
			_current_packet->crc = 0;

			if ((crc != crc_packet(_current_packet)) || (PKT_CODE(*_current_packet) == PKT_CODE_CORRUPT)) {
				_abort_dma();
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
			perf_cancel(_pc_txns);    /* don't count this as a transaction */
			break;
		}

		/* we might? see this for EINTR */
		syslog(LOG_ERR, "unexpected ret %d/%d\n", ret, errno);
	}

	/* reset DMA status */
	_rx_dma_result = _dma_status_inactive;

	/* update counters */
	perf_end(_pc_txns);

	return ret;
}

void
ArchPX4IOSerial::_dma_callback(DMACH_HANDLE handle, void *arg, bool done, int result)
{
	if (arg != nullptr) {
		ArchPX4IOSerial *ps = reinterpret_cast<ArchPX4IOSerial *>(arg);

		ps->_do_rx_dma_callback(done, result);
	}
}

void
ArchPX4IOSerial::_do_rx_dma_callback(bool done, int result)
{
	/* on completion of a reply, wake the waiter */

	if (done && _rx_dma_result == _dma_status_waiting) {

		if (result != OK) {

			/* check for packet overrun - this will occur after DMA completes */
			uint32_t sr = rSTAT;

			if (sr & (LPUART_STAT_OR | LPUART_STAT_RDRF)) {
				while (rSTAT & LPUART_STAT_RDRF) {
					(void)rDATA;
				}

				rSTAT = sr & (LPUART_STAT_OR);
				result = -EIO;
			}
		}

		/* save RX status */
		_rx_dma_result = result;

		/* disable UART DMA */

		rBAUD &= ~(LPUART_BAUD_RDMAE | LPUART_BAUD_TDMAE);

		/* complete now */
		px4_sem_post(&_completion_semaphore);
	}
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
	uint32_t sr = rSTAT;

	while (rSTAT & LPUART_STAT_RDRF) {
		(void)rDATA; /* read DATA register to clear RDRF */
	}

	rSTAT |= sr & rSTAT_ERR_FLAGS_MASK;  /* clear flags */

	if (sr & (LPUART_STAT_OR |   /* overrun error - packet was too big for DMA or DMA was too slow */
		  LPUART_STAT_NF |          /* noise error - we have lost a byte due to noise */
		  LPUART_STAT_FE)) {        /* framing error - start/stop bit lost or line break */

		/*
		 * If we are in the process of listening for something, these are all fatal;
		 * abort the DMA with an error.
		 */
		if (_rx_dma_result == _dma_status_waiting) {
			_abort_dma();

			perf_count(_pc_uerrs);
			/* complete DMA as though in error */
			_do_rx_dma_callback(true, -EIO);

			return;
		}

		/* XXX we might want to use FE / line break as an out-of-band handshake ... handle it here */

		/* don't attempt to handle IDLE if it's set - things went bad */
		return;
	}

	if (sr & LPUART_STAT_IDLE) {

		rSTAT |= LPUART_STAT_IDLE;  /* clear IDLE flag */

		/* if there is DMA reception going on, this is a short packet */
		if (_rx_dma_result == _dma_status_waiting) {
			/* Invalidate _current_packet, so we get fresh data from RAM */
			up_invalidate_dcache((uintptr_t)_current_packet,
					     (uintptr_t)_current_packet + DMA_ALIGN_UP(sizeof(IOPacket)));

			/* verify that the received packet is complete */
			size_t length = sizeof(*_current_packet) - imxrt_dmach_getcount(_rx_dma);

			if ((length < 1) || (length < PKT_SIZE(*_current_packet))) {
				perf_count(_pc_badidle);

				/* stop the receive DMA */
				imxrt_dmach_stop(_rx_dma);

				/* complete the short reception */
				_do_rx_dma_callback(true, -EIO);
				return;
			}

			perf_count(_pc_idle);

			/* complete the short reception */

			_do_rx_dma_callback(true, _dma_status_done);

			/* stop the receive DMA */
			imxrt_dmach_stop(_rx_dma);
		}
	}
}

void
ArchPX4IOSerial::_abort_dma()
{
	/* stop DMA */
	imxrt_dmach_stop(_tx_dma);
	imxrt_dmach_stop(_rx_dma);

	/* disable UART DMA */

	rBAUD &= ~(LPUART_BAUD_RDMAE | LPUART_BAUD_TDMAE);

	/* clear data that may be in the DATA register and clear overrun error: */
	uint32_t sr = rSTAT;

	if (sr & (LPUART_STAT_OR | LPUART_STAT_RDRF)) {

		while (rSTAT & LPUART_STAT_RDRF) {
			(void)rDATA;
		}
	}

	rSTAT |= sr & (rSTAT_ERR_FLAGS_MASK | LPUART_STAT_IDLE);  /* clear flags */
}
