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
 * Serial interface for PX4IO on STM32F4
 */
#include <syslog.h>

#include <px4_arch/px4io_serial.h>

/* serial register accessors */
#define REG(_x)		(*(volatile uint32_t *)(PX4IO_SERIAL_BASE + _x))

#define rRESET		(*(volatile uint32_t *)(RP2040_RESETS_BASE + RP2040_RESETS_RESET_OFFSET))
#define	rDR		REG(RP2040_UART_UARTDR_OFFSET)			// Data Register
#define	rRSR		REG(RP2040_UART_UARTRSR_OFFSET)			// Receive Status Register/Error Clear Register
#define	rFR		REG(RP2040_UART_UARTFR_OFFSET)			// Flag Register
#define	rILPR		REG(RP2040_UART_UARTILPR_OFFSET)		// IrDA Low-Power Counter Register
#define	rIBRD		REG(RP2040_UART_UARTIBRD_OFFSET)		// Integer Baud Rate Register
#define	rFBRD		REG(RP2040_UART_UARTFBRD_OFFSET)		// Fractional Baud Rate Register
#define	rLCR_H		REG(RP2040_UART_UARTLCR_H_OFFSET)		// Line Control Register
#define	rCR		REG(RP2040_UART_UARTCR_OFFSET)			// Control Register
#define	rIFLS		REG(RP2040_UART_UARTIFLS_OFFSET)		// Interrupt FIFO Level Select Register
#define	rIMSC		REG(RP2040_UART_UARTIMSC_OFFSET)		// Interrupt Mask Set/Clear Register
#define	rRIS		REG(RP2040_UART_UARTRIS_OFFSET)			// Raw Interrupt Status Register
#define	rMIS		REG(RP2040_UART_UARTMIS_OFFSET)			// Masked Interrupt Status Register
#define	rICR		REG(RP2040_UART_UARTICR_OFFSET)			// Interrupt Clear Register
#define	rDMACR		REG(RP2040_UART_UARTDMACR_OFFSET)		// DMA Control Register

#define rSR		REG(STM32_USART_SR_OFFSET)
#define rDR		REG(STM32_USART_DR_OFFSET)
#define rBRR		REG(STM32_USART_BRR_OFFSET)
#define rCR1		REG(STM32_USART_CR1_OFFSET)
#define rCR2		REG(STM32_USART_CR2_OFFSET)
#define rCR3		REG(STM32_USART_CR3_OFFSET)

uint8_t ArchPX4IOSerial::_io_buffer_storage[sizeof(IOPacket)];

ArchPX4IOSerial::ArchPX4IOSerial() :
	_tx_dma(nullptr),
	_rx_dma(nullptr),
	_current_packet(nullptr),
	_rx_dma_status(_dma_status_inactive),
	_completion_semaphore(SEM_INITIALIZER(0)),
#if 0
	_pc_dmasetup(perf_alloc(PC_ELAPSED,	"io_dmasetup ")),
	_pc_dmaerrs(perf_alloc(PC_COUNT,	"io_dmaerrs  "))
#else
	_pc_dmasetup(nullptr),
	_pc_dmaerrs(nullptr)
#endif
{
}

ArchPX4IOSerial::~ArchPX4IOSerial()
{
	if (_tx_dma != nullptr) {
		rp2040_dmastop(_tx_dma);
		rp2040_dmafree(_tx_dma);
	}

	if (_rx_dma != nullptr) {
		rp2040_dmastop(_rx_dma);
		rp2040_dmafree(_rx_dma);
	}

	/* reset the UART */
	rLCR_H = 0;
	rCR = 0;

	rRESET |= PX4IO_SERIAL_RESET;

	/* detach our interrupt handler */
	up_disable_irq(PX4IO_SERIAL_VECTOR);
	irq_detach(PX4IO_SERIAL_VECTOR);

	/* restore the GPIOs */
	px4_arch_unconfiggpio(PX4IO_SERIAL_TX_GPIO);
	px4_arch_unconfiggpio(PX4IO_SERIAL_RX_GPIO);

	/* Disable APB clock for the USART peripheral */
	// modifyreg32(PX4IO_SERIAL_RCC_REG, PX4IO_SERIAL_RCC_EN, 0);	// Not required for rp2040

	/* and kill our semaphores */
	px4_sem_destroy(&_completion_semaphore);

	perf_free(_pc_dmasetup);
	perf_free(_pc_dmaerrs);
}

int
ArchPX4IOSerial::init()
{
	/* initialize base implementation */
	int r;

	if ((r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0])) != 0) {
		return r;
	}

	/* allocate DMA */
	_tx_dma = rp2040_dmachannel();	// PX4IO_SERIAL_TX_DMAMAP not required. Managed by dma_config in rp2040_txdmasetup
	_rx_dma = rp2040_dmachannel();	// PX4IO_SERIAL_RX_DMAMAP not required. Managed by dma_config in rp2040_rxdmasetup

	if ((_tx_dma == nullptr) || (_rx_dma == nullptr)) {	// This is not possible as per the implementation of dmachannel
		return -1;
	}

	/* Enable the APB clock for the USART peripheral */
	// modifyreg32(PX4IO_SERIAL_RCC_REG, 0, PX4IO_SERIAL_RCC_EN);	// Not required for rp2040

	/* configure pins for serial use */
	px4_arch_configgpio(PX4IO_SERIAL_TX_GPIO);
	px4_arch_configgpio(PX4IO_SERIAL_RX_GPIO);

	/* reset & configure the UART */
	rRESET |= PX4IO_SERIAL_RESET;
	rRESET &= ~PX4IO_SERIAL_RESET;

	/* eat any existing interrupt status */
	(void)rRSR;
	(void)rDR;

	/* configure line speed */
	uint32_t usartdiv128 = 8 * PX4IO_SERIAL_CLOCK / PX4IO_SERIAL_BITRATE;
	uint32_t mantissa = usartdiv128 >> 7;
	uint32_t fraction = ((usartdiv128 & 0x7f) + 1) >> 1;
	rIBRD = mantissa; rFBRD = fraction;

	rLCR_H |= RP2040_UART_UARTLCR_H_WLEN_MASK;

	/* attach serial interrupt handler */
	irq_attach(PX4IO_SERIAL_VECTOR, _interrupt, this);
	up_enable_irq(PX4IO_SERIAL_VECTOR);

	/* enable UART in DMA mode, enable error and line idle interrupts */
	rCR |= RP2040_UART_UARTCR_UARTEN | RP2040_UART_UARTCR_TXE | RP2040_UART_UARTCR_RXE ;
	rLCR_H |= RP2040_UART_UARTLCR_H_FEN;

	rDMACR |= RP2040_UART_UARTDMACR_TXDMAE | RP2040_UART_UARTDMACR_RXDMAE;

	rIMSC |= RP2040_UART_UARTIMSC_OEIM | RP2040_UART_UARTIMSC_FEIM | RP2040_UART_UARTIMSC_BEIM;

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

	case 1:		/* XXX magic number - test operation */
		switch (arg) {
		case 0:
			syslog(LOG_INFO, "test 0\n");

			/* kill DMA, this is a PIO test */
			rp2040_dmastop(_tx_dma);
			rp2040_dmastop(_rx_dma);
			rDMACR &= ~(RP2040_UART_UARTDMACR_TXDMAE | RP2040_UART_UARTDMACR_RXDMAE);

			for (;;) {
				while (!(rFR & RP2040_UART_UARTFR_TXFE))
					;

				rDR = 0x55;
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

	/* clear any lingering error status */
	(void)rRSR;
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
	rp2040_rxdmasetup(
		_rx_dma,
		PX4IO_SERIAL_BASE + RP2040_UART_UARTDR_OFFSET,
		reinterpret_cast<uint32_t>(_current_packet),
		sizeof(*_current_packet),
		dma_config_t{
			.dreq = PX4IO_SERIAL_RX_DREQ,
			.size = RP2040_DMA_SIZE_BYTE,
			.noincr = 0			// Do increment the memory address
		});
	rp2040_dmastart(_rx_dma, _dma_callback, this);
	rDMACR |= RP2040_UART_UARTDMACR_RXDMAE;

	/* start TX DMA - no callback if we also expect a reply */
	/* DMA setup time ~3µs */
	rp2040_txdmasetup(
		_tx_dma,
		PX4IO_SERIAL_BASE + RP2040_UART_UARTDR_OFFSET,
		reinterpret_cast<uint32_t>(_current_packet),
		PKT_SIZE(*_current_packet),
		dma_config_t{
			.dreq = PX4IO_SERIAL_TX_DREQ,
			.size = RP2040_DMA_SIZE_BYTE,
			.noincr = 0			// Do increment the memory address
		});
	rp2040_dmastart(_tx_dma, nullptr, nullptr);
	rDMACR |= RP2040_UART_UARTDMACR_TXDMAE;

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
			if (_rx_dma_status & RP2040_DMA_CTRL_TRIG_AHB_ERROR) {
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
			perf_cancel(_pc_txns);		/* don't count this as a transaction */
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
		ArchPX4IOSerial *ps = static_cast<ArchPX4IOSerial *>(arg);

		ps->_do_rx_dma_callback(status);
	}
}

void
ArchPX4IOSerial::_do_rx_dma_callback(unsigned status)
{
	/* on completion of a reply, wake the waiter */
	if (_rx_dma_status == _dma_status_waiting) {

		/* check for packet overrun - this will occur after DMA completes */
		uint32_t sr = rRSR;
		uint32_t fr = rFR;

		if ((sr & RP2040_UART_UARTRSR_OE) | !(fr & RP2040_UART_UARTFR_RXFE))) {
			(void)rDR;
			status = RP2040_DMA_CTRL_TRIG_AHB_ERROR;
		}

		/* save RX status */
		_rx_dma_status = status;

		/* disable UART DMA */
		rDMACR &= ~(RP2040_UART_UARTDMACR_TXDMAE | RP2040_UART_UARTDMACR_RXDMAE);

		/* complete now */
		px4_sem_post(&_completion_semaphore);
	}
}

int
ArchPX4IOSerial::_interrupt(int irq, void *context, void *arg)
{
	if (arg != nullptr) {
		ArchPX4IOSerial *instance = static_cast<ArchPX4IOSerial *>(arg);

		instance->_do_interrupt();
	}

	return 0;
}

void
ArchPX4IOSerial::_do_interrupt()
{
	uint32_t sr = rRSR;	/* get UART status register */
	(void)rDR;		/* read DR to clear status */

	if (sr & (RP2040_UART_UARTRSR_OE |	/* overrun error - packet was too big for DMA or DMA was too slow */
		  RP2040_UART_UARTRSR_BE |		/* break error - we have lost a byte due to noise */
		  RP2040_UART_UARTRSR_FE)) {		/* framing error - start/stop bit lost or line break */

		/*
		 * If we are in the process of listening for something, these are all fatal;
		 * abort the DMA with an error.
		 */
		if (_rx_dma_status == _dma_status_waiting) {
			_abort_dma();

			perf_count(_pc_uerrs);
			/* complete DMA as though in error */
			_do_rx_dma_callback(RP2040_DMA_CTRL_TRIG_AHB_ERROR);

			return;
		}

		/* XXX we might want to use FE / line break as an out-of-band handshake ... handle it here */

		/* don't attempt to handle IDLE if it's set - things went bad */
		return;
	}

	// RP2040 doesn't have IDLE detection. So, following is probably not used.
	// if (sr & USART_SR_IDLE) {

	// 	/* if there is DMA reception going on, this is a short packet */
	// 	if (_rx_dma_status == _dma_status_waiting) {

	// 		/* verify that the received packet is complete */
	// 		size_t length = sizeof(*_current_packet) - stm32_dmaresidual(_rx_dma);

	// 		if ((length < 1) || (length < PKT_SIZE(*_current_packet))) {
	// 			perf_count(_pc_badidle);

	// 			/* stop the receive DMA */
	// 			stm32_dmastop(_rx_dma);

	// 			/* complete the short reception */
	// 			_do_rx_dma_callback(DMA_STATUS_TEIF);
	// 			return;
	// 		}

	// 		perf_count(_pc_idle);

	// 		/* stop the receive DMA */
	// 		stm32_dmastop(_rx_dma);

	// 		/* complete the short reception */
	// 		_do_rx_dma_callback(DMA_STATUS_TCIF);
	// 	}
	// }
}

void
ArchPX4IOSerial::_abort_dma()
{
	/* disable UART DMA */
	rDMACR &= ~(RP2040_UART_UARTDMACR_TXDMAE | RP2040_UART_UARTDMACR_RXDMAE);
	(void)rRSR;
	(void)rDR;
	(void)rDR;

	/* stop DMA */
	rp2040_dmastop(_tx_dma);
	rp2040_dmastop(_rx_dma);
}
