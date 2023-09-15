/****************************************************************************
 *
 *   Copyright (c) 2023 Technology Innovation Institute.
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
 * Serial interface for PX4IO on MPFS
 */

#include <syslog.h>

#include <px4_arch/px4io_serial.h>
#include <termios.h>
#include <debug.h>

/* Nb. this register only exists in FPGA UART implementation. For MSS uart
 * this is a scratch register, and the code works by IRQ on each byte
 */

#define MPFS_UART_RFT_OFFSET 0x1C

uint8_t ArchPX4IOSerial::_io_buffer_storage[sizeof(IOPacket)];

ArchPX4IOSerial::ArchPX4IOSerial() :
	_current_packet(nullptr),
	_completion_semaphore(SEM_INITIALIZER(0))
{
}

ArchPX4IOSerial::~ArchPX4IOSerial()
{

	/* detach our interrupt handler */
	up_disable_irq(PX4IO_SERIAL_VECTOR);
	irq_detach(PX4IO_SERIAL_VECTOR);

	/* and kill our semaphores */
	px4_sem_destroy(&_completion_semaphore);
}


int ArchPX4IOSerial::init_uart()
{
	/* Reset off */

	modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
		    SYSREG_SOFT_RESET_CR_FIC3 | SYSREG_SOFT_RESET_CR_FPGA, 0);

	/* Clock on */

	modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET,
		    0, SYSREG_SUBBLK_CLOCK_CR_FIC3);


	/* Disable interrupts */

	putreg32(0, PX4IO_SERIAL_BASE + MPFS_UART_IER_OFFSET);
	putreg32(0, PX4IO_SERIAL_BASE + MPFS_UART_IEM_OFFSET);

	/* Clear fifos */

	putreg32((UART_FCR_RFIFOR | UART_FCR_XFIFOR), PX4IO_SERIAL_BASE + MPFS_UART_FCR_OFFSET);

	/* Set filter to minimum value */

	putreg32(0, PX4IO_SERIAL_BASE + MPFS_UART_GFR_OFFSET);

	/* Set default TX time guard */

	putreg32(0, PX4IO_SERIAL_BASE + MPFS_UART_TTG_OFFSET);

	/* 8 bits, no parity 1 stop */

	uint32_t lcr = UART_LCR_DLS_8BITS;

	/* Set bits, enter dlab */

	putreg32(lcr | UART_LCR_DLAB, PX4IO_SERIAL_BASE + MPFS_UART_LCR_OFFSET);

	/*  Baud_value = PCLK_Frequency / (baud_rate * 16)
	 *
	 *  Calculate in fixed point 26.6 format (fraction is presented by 6 bits in PF)
	 */

	uint32_t baud26_6 = ((uint32_t)MPFS_FPGA_PERIPHERAL_CLK << 2) / PX4IO_SERIAL_BITRATE;
	uint16_t baud = baud26_6 >> 6;
	uint8_t fraction = baud26_6 & 0x3F;

	putreg32(baud >> 8, PX4IO_SERIAL_BASE + MPFS_UART_DLH_OFFSET);
	putreg32(baud & 0xff, PX4IO_SERIAL_BASE + MPFS_UART_DLL_OFFSET);

	if (baud > 1) {
		/* Enable Fractional baud rate */

		uint8_t mm0 = getreg32(PX4IO_SERIAL_BASE + MPFS_UART_MM0_OFFSET);
		mm0 |= UART_MM0_EFBR;
		putreg32(mm0, PX4IO_SERIAL_BASE + MPFS_UART_MM0_OFFSET);
		putreg32(fraction, PX4IO_SERIAL_BASE + MPFS_UART_DFR_OFFSET);
	}

	/* Clear dlab */

	putreg32(lcr, PX4IO_SERIAL_BASE + MPFS_UART_LCR_OFFSET);

	/* Configure the FIFO's */

	putreg32((UART_FCR_XFIFOR | UART_FCR_RFIFOR |
		  UART_FCR_FIFOE), PX4IO_SERIAL_BASE + MPFS_UART_FCR_OFFSET);

	/* Empty the shift register */

	if ((getreg32(PX4IO_SERIAL_BASE + MPFS_UART_LSR_OFFSET) & UART_LSR_DR) != 0) {
		getreg32(PX4IO_SERIAL_BASE + MPFS_UART_RBR_OFFSET);
	}

	/* Clear interrupts */

	getreg32(PX4IO_SERIAL_BASE + MPFS_UART_IIR_OFFSET);

	/* Attach serial interrupt handler */

	irq_attach(PX4IO_SERIAL_VECTOR, _interrupt, this);
	up_enable_irq(PX4IO_SERIAL_VECTOR);

	/* Create semaphores */

	px4_sem_init(&_completion_semaphore, 0, 0);

	/* _completion_semaphore use case is a signal */

	px4_sem_setprotocol(&_completion_semaphore, SEM_PRIO_NONE);

	return 0;
}

int
ArchPX4IOSerial::init()
{
	/* initialize base implementation */
	int r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0]);

	if (r != 0) {
		return r;
	}

	r = init_uart();

	return r;
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

			while ((getreg32(PX4IO_SERIAL_BASE + MPFS_UART_LSR_OFFSET)
				& UART_LSR_THRE) == 0);

			putreg32(0x55, PX4IO_SERIAL_BASE + MPFS_UART_THR_OFFSET);

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

	uint8_t *packet = (uint8_t *)_packet;
	size_t send_size = PKT_SIZE(*_packet);

	/* Protect from concurrent access */

	_current_packet = _packet;
	_packet_cnt = 0;

	/* Incoming packet size too large ? */

	if (send_size > sizeof(*_current_packet)) {
		perf_count(_pc_protoerrs);
		return -EINVAL;
	}

	/* Measure exchange time */

	perf_begin(_pc_txns);

	/* Clear fifos */

	putreg32((UART_FCR_RFIFOR | UART_FCR_XFIFOR), PX4IO_SERIAL_BASE + MPFS_UART_FCR_OFFSET);

	/* Empty the shift register */

	if ((getreg32(PX4IO_SERIAL_BASE + MPFS_UART_LSR_OFFSET) & UART_LSR_DR) != 0) {
		getreg32(PX4IO_SERIAL_BASE + MPFS_UART_RBR_OFFSET);
	}

	/* Clear pending interrupts */

	getreg32(PX4IO_SERIAL_BASE + MPFS_UART_IIR_OFFSET);

	/* Set first interrupt to occur when at least one byte is available */

	putreg32(0, PX4IO_SERIAL_BASE + MPFS_UART_RFT_OFFSET);

	/* Enable received data available interrupt */

	putreg32(UART_IER_ERBFI, PX4IO_SERIAL_BASE + MPFS_UART_IER_OFFSET);

	/* Write package to tx fifo */

	/* Don't allow interrupts during the UART fifo fill; any delay here could timeout the px4io */

	irqstate_t flags = px4_enter_critical_section();

	for (size_t i = 0; i < send_size; i++) {
		putreg32(packet[i], PX4IO_SERIAL_BASE + MPFS_UART_THR_OFFSET);
	}

	px4_leave_critical_section(flags);

	/* Wait for response, max 10 ms */

	ret = nxsem_tickwait_uninterruptible(&_completion_semaphore, MSEC2TICK(10));

	/* Disable interrupts */

	putreg32(0, PX4IO_SERIAL_BASE + MPFS_UART_IER_OFFSET);

	if (ret == -ETIMEDOUT) {
		perf_count(_pc_timeouts);
		perf_cancel(_pc_txns);		/* don't count this as a transaction */
	}

	if (ret == OK) {
		/* Check packet CRC - corrupt packet errors mean IO receive CRC error */
		uint8_t crc = _current_packet->crc;
		_current_packet->crc = 0;

		if ((crc != crc_packet(_current_packet)) || (PKT_CODE(*_current_packet) == PKT_CODE_CORRUPT)) {
			perf_count(_pc_crcerrs);
			ret = -EIO;
		}
	}

	/* Update counters */

	perf_end(_pc_txns);

	return ret;
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
	/* Get the current status */

	uint32_t status = getreg32(PX4IO_SERIAL_BASE + MPFS_UART_IIR_OFFSET);
	uint8_t *packet = (uint8_t *)_current_packet;
	bool first_rcv = _packet_cnt == 0 ? true : false;
	int ret = OK;
	uint32_t ch;

	switch (status & UART_IIR_IID_MASK) {

	case UART_IIR_IID_RECV:
	case UART_IIR_IID_TIMEOUT:

		/* Get all the available characters from the RX fifo */

		while ((getreg32(PX4IO_SERIAL_BASE + MPFS_UART_LSR_OFFSET) & UART_LSR_DR) != 0) {
			/* Read the data out regardless to clear the DR interrupt */

			ch = getreg32(PX4IO_SERIAL_BASE + MPFS_UART_RBR_OFFSET);

			/* Write to user buffer, if there is room */

			if (_packet_cnt < sizeof(*_current_packet)) {
				packet[_packet_cnt++] = ch;
			}
		}

		/* If first bytes of the package were received, extract the packet size */

		if (first_rcv) {
			_expected_size = PKT_SIZE(*_current_packet);

			if (_expected_size > sizeof(*_current_packet)) {
				perf_count(_pc_uerrs);
				ret = ERROR;
				break;
			}
		}

		if (_packet_cnt >= _expected_size) {
			/* Receive complete,
			 * Disable all interrupts, post the completion semaphore
			 */

			putreg32(0, PX4IO_SERIAL_BASE + MPFS_UART_IER_OFFSET);
			sem_post(&_completion_semaphore);

		} else {
			/* Set next interrupt to occur when the rest of the bytes are available */

			putreg32(_expected_size - _packet_cnt - 1, PX4IO_SERIAL_BASE + MPFS_UART_RFT_OFFSET);
		}

		break;

	default:
		/* Unexpected interrupt */
		ret = ERROR;
		break;
	}

	if (ret != OK) {
		/* For any errors, disable all interrupts, exchange will timeout */
		putreg32(0, PX4IO_SERIAL_BASE + MPFS_UART_IER_OFFSET);
	}
}
