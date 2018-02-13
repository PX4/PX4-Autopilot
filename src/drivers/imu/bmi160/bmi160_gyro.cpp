
#include "bmi160_gyro.hpp"
#include "bmi160.hpp"

BMI160_gyro::BMI160_gyro(BMI160 *parent, const char *path) : CDev("BMI160_gyro", path),
	_parent(parent),
	_gyro_topic(nullptr),
	_gyro_orb_class_instance(-1),
	_gyro_class_instance(-1)
{
}

BMI160_gyro::~BMI160_gyro()
{
	if (_gyro_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
	}
}

int
BMI160_gyro::init()
{
	int ret;

	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	return ret;
}

void
BMI160_gyro::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
BMI160_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->gyro_read(filp, buffer, buflen);
}

int
BMI160_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{

	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->gyro_ioctl(filp, cmd, arg);
	}
}
