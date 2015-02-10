
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <debug.h>

#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>

#include "appdebug.h"


static volatile uint32_t msg_counter;
static volatile uint32_t last_msg_counter;
static volatile uint8_t msg_next_out, msg_next_in;

/*
 * a set of debug buffers to allow us to send debug information from ISRs
 */
#define NUM_MSG 2
static char msg[NUM_MSG][40];

/*
 * WARNING: too large buffers here consume the memory required
 * for mixer handling. Do not allocate more than 80 bytes for
 * output.
 */

void isr_debug(uint8_t level, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg[msg_next_in], sizeof(msg[0]), fmt, ap);
	va_end(ap);
	msg_next_in = (msg_next_in + 1) % NUM_MSG;
	msg_counter++;
}

/*
 * show all pending debug messages
 */
void show_debug_messages(void)
{
	if (msg_counter != last_msg_counter) {
		uint32_t n = msg_counter - last_msg_counter;

		if (n > NUM_MSG) { n = NUM_MSG; }

		last_msg_counter = msg_counter;

		while (n--) {
			debug("%s", msg[msg_next_out]);
			msg_next_out = (msg_next_out + 1) % NUM_MSG;
		}
	}
}

static uint64_t last_debug_time = 0;

void isr_debug_tick(void)
{
	/* post debug state at ~1Hz */
	if (hrt_absolute_time() - last_debug_time > (1000 * 1000)) {
		/* Put debug info here */
		last_debug_time = hrt_absolute_time();
	}
}
