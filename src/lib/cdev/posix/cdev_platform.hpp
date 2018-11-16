
#pragma once

#include <cinttypes>

#define ATOMIC_ENTER lock()
#define ATOMIC_LEAVE unlock()

namespace cdev
{

struct file_operations {
	void *op;
};

using px4_file_operations_t = struct file_operations;
using mode_t = uint32_t;

struct file_t {
	int flags;
	void *priv;
	void *vdev;

	file_t() : flags(0), priv(nullptr), vdev(nullptr) {}
	file_t(int f, void *c) : flags(f), priv(nullptr), vdev(c) {}
};

} // namespace cdev

extern "C" __EXPORT int register_driver(const char *name, const cdev::px4_file_operations_t *fops,
					cdev::mode_t mode, void *data);
extern "C" __EXPORT int unregister_driver(const char *path);
