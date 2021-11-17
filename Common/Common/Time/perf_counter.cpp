/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#ifndef _MICROCONTROLLER

#include "perf_counter.hpp"

using namespace std::chrono;

/**
 * Header common to all counters.
 */
struct perf_ctr_header {
	enum perf_counter_type	type;	/**< counter type */
	const char		*name;	/**< counter name */
};

/**
 * PC_EVENT counter.
 */
struct perf_ctr_count {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
};

/**
 * PC_ELAPSED counter.
 */
struct perf_ctr_elapsed {
	struct perf_ctr_header	hdr;
	uint64_t		                        event_count;
	high_resolution_clock::time_point		time_start;
	microseconds		                    time_total;
	microseconds		                    time_least;
	microseconds		                    time_most;
	double			                        mean;
	double			                        M2;
};

/**
 * PC_INTERVAL counter.
 */
struct perf_ctr_interval {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
	high_resolution_clock::time_point		time_first;
	high_resolution_clock::time_point		time_last;
	microseconds		time_least;
	microseconds		time_most;
	double			mean;
	double			M2;
};

/**
 * List of all known counters.
 */
std::forward_list<perf_counter_t> perf_counters;

/**
 * mutex protecting access to the perf_counters linked list (which is read from & written to by different threads)
 */
pthread_mutex_t perf_counters_mutex = PTHREAD_MUTEX_INITIALIZER;
// FIXME: the mutex does **not** protect against access to/from the perf
// counter's data. It can still happen that a counter is updated while it is
// printed. This can lead to inconsistent output, or completely bogus values
// (especially the 64bit values which are in general not atomically updated).
// The same holds for shared perf counters (perf_alloc_once), that can be updated
// concurrently (this affects the 'ctrl_latency' counter).


perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{
	perf_counter_t ctr = NULL;

	switch (type) {
	case PC_COUNT:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_count), 1);
		break;

	case PC_ELAPSED:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_elapsed), 1);
		break;

	case PC_INTERVAL:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_interval), 1);

		break;

	default:
		break;
	}

	if (ctr != NULL) {
		ctr->type = type;
		ctr->name = name;
		pthread_mutex_lock(&perf_counters_mutex);
        perf_counters.push_front(ctr);
		pthread_mutex_unlock(&perf_counters_mutex);
	}

	return ctr;
}

perf_counter_t
perf_alloc_once(enum perf_counter_type type, const char *name)
{
	pthread_mutex_lock(&perf_counters_mutex);
    for (auto handle = perf_counters.begin(); handle != perf_counters.end(); ++handle) {
        if (!strcmp((*handle)->name, name)) {
			if (type == (*handle)->type) {
				/* they are the same counter */
				pthread_mutex_unlock(&perf_counters_mutex);
				return *handle;

			} else {
				/* same name but different type, assuming this is an error and not intended */
				pthread_mutex_unlock(&perf_counters_mutex);
				return NULL;
			}
		}
    }

	pthread_mutex_unlock(&perf_counters_mutex);

	/* if the execution reaches here, no existing counter of that name was found */
	return perf_alloc(type, name);
}

void
perf_free(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	pthread_mutex_lock(&perf_counters_mutex);
    // NOTE: This is perhaps slightly less efficient than the original version?
    perf_counters.remove(handle);
	pthread_mutex_unlock(&perf_counters_mutex);
	free(handle);
}

void
perf_count(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count++;
		break;

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			high_resolution_clock::time_point now = high_resolution_clock::now();

			switch (pci->event_count) {
			case 0:
				pci->time_first = now;
				break;

			case 1:
				pci->time_least = duration_cast<microseconds>(now - pci->time_last);
				pci->time_most = duration_cast<microseconds>(now - pci->time_last);
				pci->mean = pci->time_least.count() / 1e6f;
				pci->M2 = 0;
				break;

			default: {
					microseconds interval = duration_cast<microseconds>(now - pci->time_last);

					if (interval < pci->time_least) {
						pci->time_least = interval;
					}

					if (interval > pci->time_most) {
						pci->time_most = interval;
					}

					// maintain mean and variance of interval in seconds
					// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
					double dt = interval.count() / 1e6;
					double delta_intvl = dt - pci->mean;
					pci->mean += delta_intvl / pci->event_count;
					pci->M2 += delta_intvl * (dt - pci->mean);
					break;
				}
			}

			pci->time_last = now;
			pci->event_count++;
			break;
		}

	default:
		break;
	}
}

void
perf_begin(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((struct perf_ctr_elapsed *)handle)->time_start = high_resolution_clock::now();
		break;

	default:
		break;
	}
}

void
perf_end(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (pce->time_start.time_since_epoch().count() != 0) {
				microseconds elapsed = duration_cast<microseconds>(high_resolution_clock::now() - pce->time_start);

				if (elapsed.count() >= 0) {

					pce->event_count++;
					pce->time_total += elapsed;

					if ((pce->time_least > elapsed) || (pce->time_least == microseconds::zero())) {
						pce->time_least = elapsed;
					}

					if (pce->time_most < elapsed) {
						pce->time_most = elapsed;
					}

					// maintain mean and variance of the elapsed time in seconds
					// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
					double dt = elapsed.count() / 1e6;
					double delta_intvl = dt - pce->mean;
					pce->mean += delta_intvl / pce->event_count;
					pce->M2 += delta_intvl * (dt - pce->mean);

					pce->time_start = {};
				}
			}
		}
		break;

	default:
		break;
	}
}

void
perf_set_elapsed(perf_counter_t handle, microseconds elapsed)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (elapsed.count() >= 0) {

				pce->event_count++;
				pce->time_total += elapsed;

				if ((pce->time_least > elapsed) || (pce->time_least == microseconds::zero())) {
					pce->time_least = elapsed;
				}

				if (pce->time_most < elapsed) {
					pce->time_most = elapsed;
				}

				// maintain mean and variance of the elapsed time in seconds
				// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
				double dt = elapsed.count() / 1e6;
				double delta_intvl = dt - pce->mean;
				pce->mean += delta_intvl / pce->event_count;
				pce->M2 += delta_intvl * (dt - pce->mean);

				pce->time_start = {};
			}
		}
		break;

	default:
		break;
	}
}

void
perf_set_count(perf_counter_t handle, uint64_t count)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT: {
			((struct perf_ctr_count *)handle)->event_count = count;
		}
		break;

	default:
		break;
	}

}

void
perf_cancel(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			pce->time_start = {};
		}
		break;

	default:
		break;
	}
}

void
perf_reset(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count = 0;
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			pce->event_count = 0;
			pce->time_start = {};
			pce->time_total = microseconds::zero();
			pce->time_least = microseconds::zero();
			pce->time_most = microseconds::zero();
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			pci->event_count = 0;
			pci->time_first = {};
			pci->time_last = {};
			pci->time_least = microseconds::zero();
			pci->time_most = microseconds::zero();
			break;
		}
	}
}

void
perf_print_counter(perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	perf_print_counter_fd(1, handle);
}

void
perf_print_counter_fd(int fd, perf_counter_t handle)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		dprintf(fd, "%s: %llu events\n",
			handle->name,
			(unsigned long long)((struct perf_ctr_count *)handle)->event_count);
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			double rms = sqrt(pce->M2 / (pce->event_count - 1));
			dprintf(fd, "%s: %lu events, %luus elapsed, %luus avg, min %luus max %luus %5.3fus rms\n",
				handle->name,
				pce->event_count,
				pce->time_total.count(),
				(pce->event_count == 0) ? 0 : pce->time_total.count() / pce->event_count,
				pce->time_least.count(),
				pce->time_most.count(),
				1e6 * rms);
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			double rms = sqrt(pci->M2 / (pci->event_count - 1));

			dprintf(fd, "%s: %lu events, %luus avg, min %luus max %luus %5.3fus rms\n",
				handle->name,
				pci->event_count,
				(pci->event_count == 0) ? 0 : duration_cast<microseconds>(pci->time_last - pci->time_first).count() / pci->event_count,
				pci->time_least.count(),
				pci->time_most.count(),
				1e6 * rms);
			break;
		}

	default:
		break;
	}
}


int
perf_print_counter_buffer(char *buffer, int length, perf_counter_t handle)
{
	int num_written = 0;

	if (handle == NULL) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		num_written = snprintf(buffer, length, "%s: %lu events",
				       handle->name,
				       ((struct perf_ctr_count *)handle)->event_count);
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			float rms = sqrtf(pce->M2 / (pce->event_count - 1));
			num_written = snprintf(buffer, length, "%s: %lu events, %luus elapsed, %luus avg, min %luus max %luus %5.3fus rms",
					       handle->name,
					       pce->event_count,
					       pce->time_total.count(),
					       (pce->event_count == 0) ? 0 : pce->time_total.count() / pce->event_count,
					       pce->time_least.count(),
					       pce->time_most.count(),
					       1e6 * rms);
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			float rms = sqrtf(pci->M2 / (pci->event_count - 1));

			num_written = snprintf(buffer, length, "%s: %lu events, %luus avg, min %luus max %luus %5.3fus rms",
					       handle->name,
					       pci->event_count,
					       (pci->event_count == 0) ? 0 : duration_cast<microseconds>(pci->time_last - pci->time_first).count() / pci->event_count,
					       pci->time_least.count(),
					       pci->time_most.count(),
					       1e6 * rms);
			break;
		}

	default:
		break;
	}

	buffer[length - 1] = 0; // ensure 0-termination
	return num_written;
}

uint64_t
perf_event_count(perf_counter_t handle)
{
	if (handle == NULL) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		return ((struct perf_ctr_count *)handle)->event_count;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			return pce->event_count;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			return pci->event_count;
		}

	default:
		break;
	}

	return 0;
}

void
perf_iterate_all(perf_callback cb, void *user)
{
	pthread_mutex_lock(&perf_counters_mutex);
    for (auto handle = perf_counters.begin(); handle != perf_counters.end(); ++handle) {
        cb(*handle, user);
    }
	pthread_mutex_unlock(&perf_counters_mutex);
}

void
perf_print_all(int fd)
{
	pthread_mutex_lock(&perf_counters_mutex);
	for (auto handle = perf_counters.begin(); handle != perf_counters.end(); ++handle) {
		perf_print_counter_fd(fd, *handle);
	}
	pthread_mutex_unlock(&perf_counters_mutex);
}

void
perf_reset_all(void)
{
	pthread_mutex_lock(&perf_counters_mutex);
    for (auto handle = perf_counters.begin(); handle != perf_counters.end(); ++handle) {
		perf_reset(*handle);
	}
	pthread_mutex_unlock(&perf_counters_mutex);
}

// Added by Nathan, not present in PX4 perf implementation
bool perf_comparator(perf_counter_t handle_a, perf_counter_t handle_b) {
  if (handle_a->type == handle_b->type) {
    // Each counter has the same type
    switch (handle_a->type) {
      case PC_COUNT: {
        // Compare number of events
        uint64_t count_a = ((struct perf_ctr_count *)handle_a)->event_count;
        uint64_t count_b = ((struct perf_ctr_count *)handle_b)->event_count;
        return count_a > count_b;
      }
      case PC_ELAPSED: {
        // Compare total time elapsed
        long unsigned int elapsed_a =
            ((struct perf_ctr_elapsed *)handle_a)->time_total.count();
        long unsigned int elapsed_b =
            ((struct perf_ctr_elapsed *)handle_b)->time_total.count();
        return elapsed_a > elapsed_b;
      }
      case PC_INTERVAL: {
        // Compare average interval times
        struct perf_ctr_interval *pci_a = (struct perf_ctr_interval *)handle_a;
        struct perf_ctr_interval *pci_b = (struct perf_ctr_interval *)handle_b;
        if (pci_a->event_count == 0) {
          return false;
        } else if (pci_b->event_count == 0) {
          return true;
        } else {
          long unsigned int interval_a =
              duration_cast<microseconds>(pci_a->time_last - pci_a->time_first)
                  .count() /
              pci_a->event_count;
          long unsigned int interval_b =
              duration_cast<microseconds>(pci_b->time_last - pci_b->time_first)
                  .count() /
              pci_b->event_count;
          return interval_a > interval_b;
        }
        return false;
      }
      default:
        return false;
    }
  } else {
    if (handle_a->type == PC_ELAPSED || handle_b->type == PC_COUNT) {
      return true;
    } else {
      return false;
    }
  }
}

// Same as perf_print_all, but ordered by time elapsed
void perf_print_all_ordered(int fd) {
  pthread_mutex_lock(&perf_counters_mutex);
  perf_counters.sort(perf_comparator);
  for (auto handle = perf_counters.begin(); handle != perf_counters.end();
       ++handle) {
    perf_print_counter_fd(fd, *handle);
  }
  pthread_mutex_unlock(&perf_counters_mutex);
}

#endif