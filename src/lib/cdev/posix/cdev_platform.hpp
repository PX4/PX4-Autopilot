
#pragma once

#include <inttypes.h>
#include <string.h>

#if defined(__PX4_ZEPHYR)
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

static inline void cdev_atomic_leave(unsigned int key)
{
	irq_unlock(key);

	// irq_unlock() is not a reschedule point: a work queue that a publication made ready under
	// the lock preempts the publisher now, instead of when the publisher blocks
	k_reschedule();
}

// The node lock masks the interrupts, as on NuttX: with a semaphore, a low priority thread
// preempted while it copies a topic would block the higher priority threads that use it
#define ATOMIC_ENTER unsigned int atomic_key = irq_lock()
#define ATOMIC_LEAVE cdev_atomic_leave(atomic_key)
#define ATOMIC_PAUSE irq_unlock(atomic_key)
#define ATOMIC_RESUME atomic_key = irq_lock()
#else
#define ATOMIC_ENTER lock()
#define ATOMIC_LEAVE unlock()
#define ATOMIC_PAUSE
#define ATOMIC_RESUME
#endif

namespace cdev
{

struct file_operations {
	void *op;
};

using px4_file_operations_t = struct file_operations;
using mode_t = uint32_t;

class CDev;

struct file_t {
	int f_oflags{0};
	void *f_priv{nullptr};
	CDev *cdev{nullptr};

	file_t() = default;
	file_t(int f, CDev *c) : f_oflags(f), cdev(c) {}
};

} // namespace cdev

extern "C" __EXPORT int register_driver(const char *name, const cdev::px4_file_operations_t *fops,
					cdev::mode_t mode, void *data);
extern "C" __EXPORT int unregister_driver(const char *path);
