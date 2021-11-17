#pragma once
/*
This file contains tools for measuring code performance. To maximize
compatibility with existing code, it is written to implement the same
interface as the "perf" tool used onboard the crazyflie, but using the
std::chrono library. This means we can evaluate the performance of the
same code both onboard and offboard the crazyflie without having to
rewrite the timing-related code.

Additionally, this file provides a nice C++ interface for timing operations
that happen repeatedly, allowing you to time a bunch of things, and then
print all of the results e.g. when your program terminates.
*/

#ifdef _MICROCONTROLLER
// If we're using the crazyflie, use the existing perf tool
#include <perf/perf_counter.h>

#else
// Otherwise, we use our own performance timer which implements the same
// interface as perf.
// NOTE: Code here is borrowed heavily from perf_counter
#include <chrono>
#include <forward_list>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/**
 * Counter types.
 */
enum perf_counter_type {
	PC_COUNT,		/**< count the number of times an event occurs */
	PC_ELAPSED,		/**< measure the time elapsed performing an event */
	PC_INTERVAL		/**< measure the interval between instances of an event */
};

struct perf_ctr_header;
typedef struct perf_ctr_header	*perf_counter_t;

/**
 * Create a new local counter.
 *
 * @param type			The type of the new counter.
 * @param name			The counter name.
 * @return			Handle for the new counter, or NULL if a counter
 *				could not be allocated.
 */
perf_counter_t	perf_alloc(enum perf_counter_type type, const char *name);

/**
 * Get the reference to an existing counter or create a new one if it does not exist.
 *
 * @param type			The type of the counter.
 * @param name			The counter name.
 * @return			Handle for the counter, or NULL if a counter
 *				could not be allocated.
 */
perf_counter_t	perf_alloc_once(enum perf_counter_type type, const char *name);

/**
 * Free a counter.
 *
 * @param handle		The performance counter's handle.
 */
void		perf_free(perf_counter_t handle);

/**
 * Count a performance event.
 *
 * This call only affects counters that take single events; PC_COUNT, PC_INTERVAL etc.
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_count(perf_counter_t handle);

/**
 * Begin a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_begin(perf_counter_t handle);

/**
 * End a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * If a call is made without a corresponding perf_begin call, or if perf_cancel
 * has been called subsequently, no change is made to the counter.
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_end(perf_counter_t handle);

/**
 * Register a measurement
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * If a call is made without a corresponding perf_begin call. It sets the
 * value provided as argument as a new measurement.
 *
 * @param handle		The handle returned from perf_alloc.
 * @param elapsed		The time elapsed. Negative values lead to incrementing the overrun counter.
 */
void		perf_set_elapsed(perf_counter_t handle, int64_t elapsed);

/**
 * Set a counter
 *
 * This call applies to counters of type PC_COUNT. It (re-)sets the count.
 *
 * @param handle		The handle returned from perf_alloc.
 * @param count			The counter value to be set.
 */
void		perf_set_count(perf_counter_t handle, uint64_t count);

/**
 * Cancel a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * It reverts the effect of a previous perf_begin.
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_cancel(perf_counter_t handle);

/**
 * Reset a performance counter.
 *
 * This call resets performance counter to initial state
 *
 * @param handle		The handle returned from perf_alloc.
 */
void		perf_reset(perf_counter_t handle);

/**
 * Print one performance counter to stdout
 *
 * @param handle		The counter to print.
 */
void		perf_print_counter(perf_counter_t handle);

/**
 * Print one performance counter to a fd.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 * @param handle		The counter to print.
 */
void		perf_print_counter_fd(int fd, perf_counter_t handle);

/**
 * Print one performance counter to a buffer.
 *
 * @param buffer			buffer to write to
 * @param length			buffer length
 * @param handle			The counter to print.
 * @param return			number of bytes written
 */
int		perf_print_counter_buffer(char *buffer, int length, perf_counter_t handle);

/**
 * Print all of the performance counters.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 */
void		perf_print_all(int fd);


typedef void (*perf_callback)(perf_counter_t handle, void *user);

/**
 * Iterate over all performance counters using a callback.
 *
 * Caution: This will aquire the mutex, so do not call any other perf_* method
 * that aquire the mutex as well from the callback (If this is needed, configure
 * the mutex to be reentrant).
 *
 * @param cb callback method
 * @param user custom argument for the callback
 */
void	perf_iterate_all(perf_callback cb, void *user);

/**
 * Print hrt latency counters.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 */
void		perf_print_latency(int fd);

/**
 * Reset all of the performance counters.
 */
void		perf_reset_all(void);

/**
 * Return current event_count
 *
 * @param handle		The counter returned from perf_alloc.
 * @return			event_count
 */
uint64_t	perf_event_count(perf_counter_t handle);

// Added by Nathan, not in original PX4 implementaiton:
/**
 * Print all of the performance counters ordered by total elapsed time (or count or average interval).
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 */
void		perf_print_all_ordered(int fd);

#endif
