/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * Serial interface for PX4IO on STM32F7
 */

#include <syslog.h>

#include <px4_arch/px4io_serial.h>
#include "stm32_uart.h"
#include <nuttx/cache.h>

/* serial register accessors */
#define REG(_x)   (*(volatile uint32_t *)(PX4IO_SERIAL_BASE + (_x)))
#define rISR    REG(STM32_USART_ISR_OFFSET)
#define rISR_ERR_FLAGS_MASK (0x1f)
#define rICR    REG(STM32_USART_ICR_OFFSET)
#define rRDR    REG(STM32_USART_RDR_OFFSET)
#define rTDR    REG(STM32_USART_TDR_OFFSET)
#define rBRR    REG(STM32_USART_BRR_OFFSET)
#define rCR1    REG(STM32_USART_CR1_OFFSET)
#define rCR2    REG(STM32_USART_CR2_OFFSET)
#define rCR3    REG(STM32_USART_CR3_OFFSET)
#define rGTPR   REG(STM32_USART_GTPR_OFFSET)

/*
 *
 * #define USART_CR1_FIFOEN          (1 << 29)  Bit 29: FIFO mode enable
 * #define USART_CR1_TXFEIE          (1 << 30)  Bit 30: TXFIFO empty interrupt enable
 * #define USART_CR1_RXFFIE          (1 << 31)  Bit 31: RXFIFO Full interrupt enable
 *
 *
 * #define USART_CR2_SLVEN           (1 << 0)   Bit 0:  Synchronous Slave mode enable
 * #define USART_CR2_DISNSS          (1 << 3)   Bit 3:  Ignore NSS pin input
 * #define USART_CR3_RXFTCFG_SHIFT   (25)       Bit 25-27: Receive FIFO threshold configuration
 * #define USART_CR3_RXFTCFG_MASK    (7 << USART_CR3_RXFTCFG_SHIFT)
 * #  define USART_CR3_RXFTCFG(n)    ((uint32_t)(n) << USART_CR3_RXFTCFG_SHIFT)
 * #  define USART_CR3_RXFTCFG_12PCT (0 << USART_CR3_RXFTCFG_SHIFT)   RXFIFO 1/8 full
 * #  define USART_CR3_RXFTCFG_25PCT (1 << USART_CR3_RXFTCFG_SHIFT)   RXFIFO 1/4 full
 *#  define USART_CR3_RXFTCFG_50PCT (2 << USART_CR3_RXFTCFG_SHIFT)   RXFIFO 1/2 full
 *#  define USART_CR3_RXFTCFG_75PCT (3 << USART_CR3_RXFTCFG_SHIFT)   RXFIFO 3/4 full
 * #  define USART_CR3_RXFTCFG_88PCT (4 << USART_CR3_RXFTCFG_SHIFT)   RXFIFO 7/8 full
 * #  define USART_CR3_RXFTCFG_FULL  (5 << USART_CR3_RXFTCFG_SHIFT)   RXIFO full
 * #define USART_CR3_RXFTIE          (1 << 28)  Bit 28: RXFIFO threshold interrupt enable
 *#define USART_CR3_TXFTCFG_SHIFT   (29)       Bits 29-31: TXFIFO threshold configuration
 * #define USART_CR3_TXFTCFG_MASK    (7 << USART_CR3_TXFTCFG_SHIFT)
 * #  define USART_CR3_TXFTCFG(n)    ((uint32_t)(n) << USART_CR3_TXFTCFG_SHIFT)
 * #  define USART_CR3_TXFTCFG_12PCT (0 << USART_CR3_TXFTCFG_SHIFT) TXFIFO 1/8 full
 *#  define USART_CR3_TXFTCFG_24PCT (1 << USART_CR3_TXFTCFG_SHIFT)  TXFIFO 1/4 full
 *#  define USART_CR3_TXFTCFG_50PCT (2 << USART_CR3_TXFTCFG_SHIFT)  TXFIFO 1/2 full
 * #  define USART_CR3_TXFTCFG_75PCT (3 << USART_CR3_TXFTCFG_SHIFT) TXFIFO 3/4 full
 * #  define USART_CR3_TXFTCFG_88PCT (4 << USART_CR3_TXFTCFG_SHIFT) TXFIFO 7/8 full
 * #  define USART_CR3_TXFTCFG_EMPY  (5 << USART_CR3_TXFTCFG_SHIFT) TXFIFO empty
#define USART_ISR_RWU             (1 << 19)  Bit 19: Receiver wakeup from Mute mode
#define USART_ISR_WUF             (1 << 20)  Bit 20: Wakeup from low-power mode flag
#define USART_ISR_TEACK           (1 << 21)  Bit 21: Transmit enable acknowledge flag
#define USART_ISR_REACK           (1 << 22)  Bit 22: Receive enable acknowledge flag
#define USART_ISR_TXFE            (1 << 23)  Bit 23: TXFIFO Empty
#define USART_ISR_RXFF            (1 << 24)  Bit 24: RXFIFO Full *
#define USART_ISR_TCBGT           (1 << 25)  Bit 25: Transmission complete before guard time flag
#define USART_ISR_RXFT            (1 << 26)  Bit 26: RXFIFO threshold flag uint8_t
#define USART_ISR_TXFT            (1 << 27)  Bit 27: TXFIFO threshold flag
*/

#define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)

uint8_t ArchPX4IOSerial::_io_buffer_storage[DMA_ALIGN_UP(sizeof(IOPacket))];

ArchPX4IOSerial::ArchPX4IOSerial() :
	_tx_dma(nullptr),
	_rx_dma(nullptr),
	_current_packet(nullptr),
	_rx_dma_status(_dma_status_inactive),
	_completion_semaphore(SEM_INITIALIZER(0)),
#if 0
	_pc_dmasetup(perf_alloc(PC_ELAPSED, "io_dmasetup ")),
	_pc_dmaerrs(perf_alloc(PC_COUNT,  "io_dmaerrs  "))
#else
	_pc_dmasetup(nullptr),
	_pc_dmaerrs(nullptr)
#endif
{
}

ArchPX4IOSerial::~ArchPX4IOSerial()
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
	px4_arch_unconfiggpio(PX4IO_SERIAL_TX_GPIO);
	px4_arch_unconfiggpio(PX4IO_SERIAL_RX_GPIO);

	/* Disable APB clock for the USART peripheral */
	modifyreg32(PX4IO_SERIAL_RCC_REG, PX4IO_SERIAL_RCC_EN, 0);

	/* and kill our semaphores */
	px4_sem_destroy(&_completion_semaphore);

	perf_free(_pc_dmasetup);
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
	_tx_dma = stm32_dmachannel(PX4IO_SERIAL_TX_DMAMAP);
	_rx_dma = stm32_dmachannel(PX4IO_SERIAL_RX_DMAMAP);

	if ((_tx_dma == nullptr) || (_rx_dma == nullptr)) {
		return -1;
	}

	/* Enable the APB clock for the USART peripheral */
	modifyreg32(PX4IO_SERIAL_RCC_REG, 0, PX4IO_SERIAL_RCC_EN);

	/* configure pins for serial use */
	px4_arch_configgpio(PX4IO_SERIAL_TX_GPIO);
	px4_arch_configgpio(PX4IO_SERIAL_RX_GPIO);

	/* reset & configure the UART */
	rCR1 = 0;
	rCR2 = 0;
	rCR3 = 0;

	/* clear data that may be in the RDR and clear overrun error: */
	if (rISR & USART_ISR_RXNE) {
		(void)rRDR;
	}

	rICR = rISR & rISR_ERR_FLAGS_MASK;  /* clear the flags */

	/* configure line speed */
	uint32_t usartdiv32 = (PX4IO_SERIAL_CLOCK + (PX4IO_SERIAL_BITRATE) / 2) / (PX4IO_SERIAL_BITRATE);
	rBRR = usartdiv32;

	/* attach serial interrupt handler */
	irq_attach(PX4IO_SERIAL_VECTOR, _interrupt, this);
	up_enable_irq(PX4IO_SERIAL_VECTOR);

	/* enable UART in DMA mode, enable error and line idle interrupts */
	rCR3 = USART_CR3_EIE;
	/* TODO: maybe use DDRE */

	rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;
	/* TODO: maybe we need to adhere to the procedure as described in the reference manual page 1251 (34.5.2) */

	/* create semaphores */
	px4_sem_init(&_completion_semaphore, 0, 0);

	/* _completion_semaphore use case is a signal */

	px4_sem_setprotocol(&_completion_semaphore, SEM_PRIO_NONE);

	/* XXX this could try talking to IO */

	return 0;
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
			stm32_dmastop(_tx_dma);
			stm32_dmastop(_rx_dma);
			rCR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);

			for (;;) {
				while (!(rISR & USART_ISR_TXE))
					;

				rTDR = 0x55;
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
	_current_packet = _packet;

	/* clear data that may be in the RDR and clear overrun error: */
	if (rISR & USART_ISR_RXNE) {
		(void)rRDR;
	}

	rICR = rISR & rISR_ERR_FLAGS_MASK;  /* clear the flags */

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
	stm32_dmacfg_t rxdmacfg;
	rxdmacfg.paddr = PX4IO_SERIAL_BASE + STM32_USART_RDR_OFFSET;
	rxdmacfg.maddr = reinterpret_cast<uint32_t>(_current_packet);
	rxdmacfg.ndata = sizeof(*_current_packet);
	rxdmacfg.cfg1  = (DMA_SCR_CIRC        |
			  DMA_SCR_DIR_P2M       |
			  DMA_SCR_MINC          |
			  DMA_SCR_PSIZE_8BITS   |
			  DMA_SCR_MSIZE_8BITS   |
			  DMA_SCR_PBURST_SINGLE |
			  DMA_SCR_MBURST_SINGLE);
	rxdmacfg.cfg2  = 0;


	stm32_dmasetup(_rx_dma, &rxdmacfg);

	rCR3 |= USART_CR3_DMAR;
	stm32_dmastart(_rx_dma, _dma_callback, this, false);

	/* Clean _current_packet, so DMA can see the data */
	up_clean_dcache((uintptr_t)_current_packet,
			(uintptr_t)_current_packet + DMA_ALIGN_UP(sizeof(IOPacket)));

	/* start TX DMA - no callback if we also expect a reply */
	/* DMA setup time ~3µs */

	stm32_dmacfg_t txdmacfg;
	txdmacfg.paddr = PX4IO_SERIAL_BASE + STM32_USART_TDR_OFFSET;
	txdmacfg.maddr = reinterpret_cast<uint32_t>(_current_packet);
	txdmacfg.ndata = PKT_SIZE(*_current_packet);
	txdmacfg.cfg1  = (DMA_SCR_DIR_M2P      |
			  DMA_SCR_MINC          |
			  DMA_SCR_PSIZE_8BITS   |
			  DMA_SCR_MSIZE_8BITS   |
			  DMA_SCR_PBURST_SINGLE |
			  DMA_SCR_MBURST_SINGLE);
	txdmacfg.cfg2  = 0;

	stm32_dmasetup(_tx_dma, &txdmacfg);
	rCR3 |= USART_CR3_DMAT;
	stm32_dmastart(_tx_dma, nullptr, nullptr, false);
	//rCR1 &= ~USART_CR1_TE;
	//rCR1 |= USART_CR1_TE;

	perf_end(_pc_dmasetup);

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
			if (_rx_dma_status & DMA_STATUS_TEIF) {
				// stream transfer error, ensure TX DMA is also stopped before exiting early
				stm32_dmastop(_tx_dma);
				perf_count(_pc_dmaerrs);
				ret = -EIO;
				break;
			}

			/* check packet CRC - corrupt packet errors mean IO receive CRC error */
			uint8_t crc = _current_packet->crc;
			_current_packet->crc = 0;

			if ((crc != crc_packet(_current_packet)) || (PKT_CODE(*_current_packet) == PKT_CODE_CORRUPT)) {
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
	_rx_dma_status = _dma_status_inactive;

	/* update counters */
	perf_end(_pc_txns);

	return ret;
}

void
ArchPX4IOSerial::_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	if (arg != nullptr) {
		ArchPX4IOSerial *ps = reinterpret_cast<ArchPX4IOSerial *>(arg);

		ps->_do_rx_dma_callback(status);
	}
}

void
ArchPX4IOSerial::_do_rx_dma_callback(unsigned status)
{
	/* on completion of a reply, wake the waiter */
	if (_rx_dma_status == _dma_status_waiting) {

		/* check for packet overrun - this will occur after DMA completes */
		uint32_t sr = rISR;

		if (sr & (USART_ISR_ORE | USART_ISR_RXNE)) {
			(void)rRDR;
			rICR = sr & (USART_ISR_ORE | USART_ISR_RXNE);
			status = DMA_STATUS_TEIF;
		}

		/* save RX status */
		_rx_dma_status = status;

		/* disable UART DMA */
		rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

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
	uint32_t sr = rISR; /* get UART status register */

	if (sr & USART_ISR_RXNE) {
		(void)rRDR; /* read DR to clear RXNE */
	}

	rICR = sr & rISR_ERR_FLAGS_MASK;  /* clear flags */

	if (sr & (USART_ISR_ORE |   /* overrun error - packet was too big for DMA or DMA was too slow */
		  USART_ISR_NE |          /* noise error - we have lost a byte due to noise */
		  USART_ISR_FE)) {        /* framing error - start/stop bit lost or line break */

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

	if (sr & USART_ISR_IDLE) {

		/* if there is DMA reception going on, this is a short packet */
		if (_rx_dma_status == _dma_status_waiting) {
			/* Invalidate _current_packet, so we get fresh data from RAM */
			up_invalidate_dcache((uintptr_t)_current_packet,
					     (uintptr_t)_current_packet + DMA_ALIGN_UP(sizeof(IOPacket)));

			/* verify that the received packet is complete */
			size_t length = sizeof(*_current_packet) - stm32_dmaresidual(_rx_dma);

			if ((length < 1) || (length < PKT_SIZE(*_current_packet))) {
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
ArchPX4IOSerial::_abort_dma()
{
	/* stop DMA */
	stm32_dmastop(_tx_dma);
	stm32_dmastop(_rx_dma);

	/* disable UART DMA */
	rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

	/* clear data that may be in the RDR and clear overrun error: */
	if (rISR & USART_ISR_RXNE) {
		(void)rRDR;
	}

	rICR = rISR & rISR_ERR_FLAGS_MASK;  /* clear the flags */
}
