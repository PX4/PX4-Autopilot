/*
 * perf_counter.c

 *
 *  Created on: Sep 24, 2014
 *      Author: roman
 */
#include <stdlib.h>
#include <stdio.h>
#include <systemlib/perf_counter.h>



perf_counter_t	perf_alloc(enum perf_counter_type type, const char *name)
{
	return NULL;
}

/**
 * Free a counter.
 *
 * @param handle		The performance counter's handle.
 */
void perf_free(perf_counter_t handle)
{

}

/**
 * Count a performance event.
 *
 * This call only affects counters that take single events; PC_COUNT, PC_INTERVAL etc.
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_count(perf_counter_t handle)
{

}

/**
 * Begin a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_begin(perf_counter_t handle)
{

}

/**
 * End a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * If a call is made without a corresopnding perf_begin call, or if perf_cancel
 * has been called subsequently, no change is made to the counter.
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_end(perf_counter_t handle)
{

}

/**
 * Cancel a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * It reverts the effect of a previous perf_begin.
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_cancel(perf_counter_t handle)
{

}

/**
 * Reset a performance counter.
 *
 * This call resets performance counter to initial state
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_reset(perf_counter_t handle)
{

}

/**
 * Print one performance counter to stdout
 *
 * @param handle		The counter to print.
 */
void		perf_print_counter(perf_counter_t handle)
{

}

/**
 * Print one performance counter to a fd.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 * @param handle		The counter to print.
 */
void		perf_print_counter_fd(int fd, perf_counter_t handle)
{

}

/**
 * Print all of the performance counters.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 */
void		perf_print_all(int fd)
{

}

/**
 * Reset all of the performance counters.
 */
void		perf_reset_all(void)
{

}

/**
 * Return current event_count
 *
 * @param handle		The counter returned from perf_alloc.
 * @return			event_count
 */
uint64_t	perf_event_count(perf_counter_t handle)
{

}


