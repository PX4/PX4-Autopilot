
#pragma once

#include <cinttypes>

#define ATOMIC_ENTER lock()
#define ATOMIC_LEAVE unlock()

namespace device
{

struct file_operations {
	void *op;
};

using px4_file_operations_t = struct file_operations;
using mode_t = uint32_t;

struct vdev_file {
	int fd;
	int flags;
	mode_t mode;
	void *priv;
	void *vdev;

	vdev_file() : fd(-1), flags(0), priv(nullptr), vdev(nullptr) {}

	vdev_file(int f, void *c, int d) : fd(d), flags(f), priv(nullptr), vdev(c) {}
};

using file_t = struct vdev_file;

} // namespace device

extern "C" __EXPORT int register_driver(const char *name, const device::px4_file_operations_t *fops,
					device::mode_t mode, void *data);
extern "C" __EXPORT int unregister_driver(const char *path);
