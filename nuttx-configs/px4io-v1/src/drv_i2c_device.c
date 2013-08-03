/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
  * @file A simple, polled I2C slave-mode driver.
  *
  * The master writes to and reads from a byte buffer, which the caller 
  * can update inbetween calls to the FSM.
  */

#include <stdbool.h>

#include "stm32_i2c.h"

#include <string.h>

/*
 * I2C register definitions.
 */
#define I2C_BASE	STM32_I2C1_BASE

#define REG(_reg)	(*(volatile uint32_t *)(I2C_BASE + _reg))

#define rCR1		REG(STM32_I2C_CR1_OFFSET)
#define rCR2		REG(STM32_I2C_CR2_OFFSET)
#define rOAR1		REG(STM32_I2C_OAR1_OFFSET)
#define rOAR2		REG(STM32_I2C_OAR2_OFFSET)
#define rDR		REG(STM32_I2C_DR_OFFSET)
#define rSR1		REG(STM32_I2C_SR1_OFFSET)
#define rSR2		REG(STM32_I2C_SR2_OFFSET)
#define rCCR		REG(STM32_I2C_CCR_OFFSET)
#define rTRISE		REG(STM32_I2C_TRISE_OFFSET)

/*
 * "event" values (cr2 << 16 | cr1) as described in the ST DriverLib
 */
#define  I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED          ((uint32_t)0x00020002) /* BUSY and ADDR flags */
#define  I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED       ((uint32_t)0x00060082) /* TRA, BUSY, TXE and ADDR flags */
#define  I2C_EVENT_SLAVE_BYTE_RECEIVED                     ((uint32_t)0x00020040) /* BUSY and RXNE flags */
#define  I2C_EVENT_SLAVE_STOP_DETECTED                     ((uint32_t)0x00000010) /* STOPF flag */
#define  I2C_EVENT_SLAVE_BYTE_TRANSMITTED                  ((uint32_t)0x00060084) /* TRA, BUSY, TXE and BTF flags */
#define  I2C_EVENT_SLAVE_BYTE_TRANSMITTING                 ((uint32_t)0x00060080) /* TRA, BUSY and TXE flags */
#define  I2C_EVENT_SLAVE_ACK_FAILURE                       ((uint32_t)0x00000400) /* AF flag */

/**
 * States implemented by the I2C FSM.
 */
enum fsm_state {
	BAD_PHASE,	 // must be zero, default exit on a bad state transition

	WAIT_FOR_MASTER,

	/* write from master */
	WAIT_FOR_COMMAND,
	RECEIVE_COMMAND,
	RECEIVE_DATA,
	HANDLE_COMMAND,

	/* read from master */
	WAIT_TO_SEND,
	SEND_STATUS,
	SEND_DATA,

	NUM_STATES
};

/**
 * Events recognised by the I2C FSM.
 */
enum fsm_event {
	/* automatic transition */
	AUTO,

	/* write from master */
	ADDRESSED_WRITE,
	BYTE_RECEIVED,
	STOP_RECEIVED,

	/* read from master */
	ADDRESSED_READ,
	BYTE_SENDABLE,
	ACK_FAILED,

	NUM_EVENTS
};

/**
 * Context for the I2C FSM
 */
static struct fsm_context {
	enum fsm_state		state;

	/* XXX want to eliminate these */
	uint8_t			command;
	uint8_t			status;

	uint8_t			*data_ptr;
	uint32_t		data_count;

	size_t			buffer_size;
	uint8_t			*buffer;
} context;

/**
 * Structure defining one FSM state and its outgoing transitions.
 */
struct fsm_transition {
		void (*handler)(void);
		enum fsm_state next_state[NUM_EVENTS];
};

static bool	i2c_command_received;

static void	fsm_event(enum fsm_event event);

static void	go_bad(void);
static void	go_wait_master(void);

static void	go_wait_command(void);
static void	go_receive_command(void);
static void	go_receive_data(void);
static void	go_handle_command(void);

static void	go_wait_send(void);
static void	go_send_status(void);
static void	go_send_buffer(void);

/**
 * The FSM state graph.
 */
static const struct fsm_transition fsm[NUM_STATES] = {
		[BAD_PHASE] = {
				.handler = go_bad,
				.next_state = {
						[AUTO] = WAIT_FOR_MASTER,
				},
		},

		[WAIT_FOR_MASTER] = {
				.handler = go_wait_master,
				.next_state = {
						[ADDRESSED_WRITE] = WAIT_FOR_COMMAND,
						[ADDRESSED_READ] = WAIT_TO_SEND,
				},
		},

		/* write from master*/
		[WAIT_FOR_COMMAND] = {
				.handler = go_wait_command,
				.next_state = {
						[BYTE_RECEIVED] = RECEIVE_COMMAND,
						[STOP_RECEIVED] = WAIT_FOR_MASTER,
				},
		},
		[RECEIVE_COMMAND] = {
				.handler = go_receive_command,
				.next_state = {
						[BYTE_RECEIVED] = RECEIVE_DATA,
						[STOP_RECEIVED] = HANDLE_COMMAND,
				},
		},
		[RECEIVE_DATA] = {
				.handler = go_receive_data,
				.next_state = {
						[BYTE_RECEIVED] = RECEIVE_DATA,
						[STOP_RECEIVED] = HANDLE_COMMAND,
				},
		},
		[HANDLE_COMMAND] = {
				.handler = go_handle_command,
				.next_state = {
						[AUTO] = WAIT_FOR_MASTER,
				},
		},

		/* buffer send */
		[WAIT_TO_SEND] = {
			.handler = go_wait_send,
			.next_state = {
					[BYTE_SENDABLE] = SEND_STATUS,
			},
		},
		[SEND_STATUS] = {
				.handler = go_send_status,
				.next_state = {
						[BYTE_SENDABLE] = SEND_DATA,
						[ACK_FAILED] = WAIT_FOR_MASTER,
				},
		},
		[SEND_DATA] = {
				.handler = go_send_buffer,
				.next_state = {
						[BYTE_SENDABLE] = SEND_DATA,
						[ACK_FAILED] = WAIT_FOR_MASTER,
				},
		},
};


/* debug support */
#if 1
struct fsm_logentry {
	char		kind;
	uint32_t	code;
};

#define LOG_ENTRIES	32
static struct fsm_logentry fsm_log[LOG_ENTRIES];
int	fsm_logptr;
#define LOG_NEXT(_x)	(((_x) + 1) % LOG_ENTRIES)
#define LOGx(_kind, _code)		\
		do {					\
			fsm_log[fsm_logptr].kind = _kind; \
			fsm_log[fsm_logptr].code = _code; \
			fsm_logptr = LOG_NEXT(fsm_logptr); \
			fsm_log[fsm_logptr].kind = 0; \
		} while(0)

#define LOG(_kind, _code) \
		do {\
			if (fsm_logptr < LOG_ENTRIES) { \
				fsm_log[fsm_logptr].kind = _kind; \
				fsm_log[fsm_logptr].code = _code; \
				fsm_logptr++;\
			}\
		}while(0)

#else
#define LOG(_kind, _code)
#endif


static void i2c_setclock(uint32_t frequency);

/**
 * Initialise I2C
 *
 */
void
i2c_fsm_init(uint8_t *buffer, size_t buffer_size)
{
	/* save the buffer */
	context.buffer = buffer;
	context.buffer_size = buffer_size;

	// initialise the FSM
	context.status = 0;
	context.command = 0;
	context.state = BAD_PHASE;
	fsm_event(AUTO);

#if 0
	// enable the i2c block clock and reset it
	modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_I2C1EN);
	modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_I2C1RST);
	modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST, 0);

	// configure the i2c GPIOs
	stm32_configgpio(GPIO_I2C1_SCL);
	stm32_configgpio(GPIO_I2C1_SDA);

	// set the peripheral clock to match the APB clock
	rCR2 = STM32_PCLK1_FREQUENCY / 1000000;

	// configure for 100kHz operation
	i2c_setclock(100000);

	// enable i2c
	rCR1 = I2C_CR1_PE;
#endif
}

/**
 * Run the I2C FSM for some period.
 *
 * @return True if the buffer has been updated by a command.
 */
bool
i2c_fsm(void)
{
	uint32_t	event;
	int		idle_iterations = 0;

	for (;;) {
		// handle bus error states by discarding the current operation
		if (rSR1 & I2C_SR1_BERR) {
			context.state = WAIT_FOR_MASTER;
			rSR1 = ~I2C_SR1_BERR;
		}

		// we do not anticipate over/underrun errors as clock-stretching is enabled

		// fetch the most recent event
		event = ((rSR2 << 16) | rSR1) & 0x00ffffff;

		// generate FSM events based on I2C events
		switch (event) {
		case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
			LOG('w', 0);
			fsm_event(ADDRESSED_WRITE);
			break;

		case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
			LOG('r', 0);
			fsm_event(ADDRESSED_READ);
			break;

		case I2C_EVENT_SLAVE_BYTE_RECEIVED:
			LOG('R', 0);
			fsm_event(BYTE_RECEIVED);
			break;

		case I2C_EVENT_SLAVE_STOP_DETECTED:
			LOG('s', 0);
			fsm_event(STOP_RECEIVED);
			break;

		case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
		//case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
			LOG('T', 0);
			fsm_event(BYTE_SENDABLE);
			break;

		case I2C_EVENT_SLAVE_ACK_FAILURE:
			LOG('a', 0);
			fsm_event(ACK_FAILED);
			break;

		default:
			idle_iterations++;
//			if ((event) && (event != 0x00020000))
//				LOG('e', event);
			break;
		}

		/* if we have just received something, drop out and let the caller handle it */
		if (i2c_command_received) {
			i2c_command_received = false;
			return true;
		}

		/* if we have done nothing recently, drop out and let the caller have a slice */
		if (idle_iterations > 1000)
			return false;
	}
}

/**
 * Update the FSM with an event
 *
 * @param event	New event.
 */
static void
fsm_event(enum fsm_event event)
{
	// move to the next state
	context.state = fsm[context.state].next_state[event];

	LOG('f', context.state);

	// call the state entry handler
	if (fsm[context.state].handler) {
		fsm[context.state].handler();
	}
}

static void
go_bad()
{
	LOG('B', 0);
	fsm_event(AUTO);
}

/**
 * Wait for the master to address us.
 *
 */
static void
go_wait_master()
{
	// We currently don't have a command byte.
	//
	context.command = '\0';

	// The data pointer starts pointing to the start of the data buffer.
	//
	context.data_ptr = context.buffer;

	// The data count is either:
	// - the size of the data buffer
	// - some value less than or equal the size of the data buffer during a write or a read
	//
	context.data_count = context.buffer_size;

	// (re)enable the peripheral, clear the stop event flag in
	// case we just finished receiving data
	rCR1 |= I2C_CR1_PE;

	// clear the ACK failed flag in case we just finished sending data
	rSR1 = ~I2C_SR1_AF;
}

/**
 * Prepare to receive a command byte.
 *
 */
static void
go_wait_command()
{
	// NOP
}

/**
 * Command byte has been received, save it and prepare to handle the data.
 *
 */
static void
go_receive_command()
{

	// fetch the command byte
	context.command = (uint8_t)rDR;
	LOG('c', context.command);

}

/**
 * Receive a data byte.
 *
 */
static void
go_receive_data()
{
	uint8_t	d;

	// fetch the byte
	d = (uint8_t)rDR;
	LOG('d', d);

	// if we have somewhere to put it, do so
	if (context.data_count) {
		*context.data_ptr++ = d;
		context.data_count--;
	}
}

/**
 * Handle a command once the host is done sending it to us.
 *
 */
static void
go_handle_command()
{
	// presume we are happy with the command
	context.status = 0;

	// make a note that the buffer contains a fresh command
	i2c_command_received = true;

	// kick along to the next state
	fsm_event(AUTO);
}

/**
 * Wait to be able to send the status byte.
 *
 */
static void
go_wait_send()
{
	// NOP
}

/**
 * Send the status byte.
 *
 */
static void
go_send_status()
{
	rDR = context.status;
	LOG('?', context.status);
}

/**
 * Send a data or pad byte.
 *
 */
static void
go_send_buffer()
{
	if (context.data_count) {
		LOG('D', *context.data_ptr);
		rDR = *(context.data_ptr++);
		context.data_count--;
	} else {
		LOG('-', 0);
		rDR = 0xff;
	}
}

/* cribbed directly from the NuttX master driver */
static void 
i2c_setclock(uint32_t frequency)
{
	uint16_t cr1;
	uint16_t ccr;
	uint16_t trise;
	uint16_t freqmhz;
	uint16_t speed;

	/* Disable the selected I2C peripheral to configure TRISE */

	cr1 = rCR1;
	rCR1 &= ~I2C_CR1_PE;

	/* Update timing and control registers */

	freqmhz = (uint16_t)(STM32_PCLK1_FREQUENCY / 1000000);
	ccr = 0;

	/* Configure speed in standard mode */

	if (frequency <= 100000) {
		/* Standard mode speed calculation */

		speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency << 1));

		/* The CCR fault must be >= 4 */

		if (speed < 4) {
			/* Set the minimum allowed value */

			speed = 4;  
		}
		ccr |= speed;

		/* Set Maximum Rise Time for standard mode */

		trise = freqmhz + 1; 
				
	/* Configure speed in fast mode */
	} else { /* (frequency <= 400000) */
		/* Fast mode speed calculation with Tlow/Thigh = 16/9 */

#ifdef CONFIG_I2C_DUTY16_9
	      speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency * 25));

		/* Set DUTY and fast speed bits */

		ccr |= (I2C_CCR_DUTY|I2C_CCR_FS);
#else
		/* Fast mode speed calculation with Tlow/Thigh = 2 */

		speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency * 3));

		/* Set fast speed bit */

		ccr |= I2C_CCR_FS;
#endif

		/* Verify that the CCR speed value is nonzero */

		if (speed < 1) {
	        	/* Set the minimum allowed value */

	        	speed = 1;  
	        }
		ccr |= speed;

		/* Set Maximum Rise Time for fast mode */

		trise = (uint16_t)(((freqmhz * 300) / 1000) + 1);  
	}

	/* Write the new values of the CCR and TRISE registers */

	rCCR = ccr;
	rTRISE = trise;

	/* Bit 14 of OAR1 must be configured and kept at 1 */

	rOAR1 = I2C_OAR1_ONE);

	/* Re-enable the peripheral (or not) */

	rCR1 = cr1;
}
