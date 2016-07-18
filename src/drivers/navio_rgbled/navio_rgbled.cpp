#include "navio_gpio.h"
#include "DevObj.hpp"

#include <px4_posix.h>
#include <drivers/drv_rgbled.h>

#define RGBLED_BASE_DEVICE_PATH "/dev/rgbled"
#define RGBLED_DEVICE_PATH  "/dev/rgbled0"

#define GPIO_LED_CNF    (GPIO_CNF_OUTPUT)
#define GPIO_LED_R      (GPIO_PIN4)
#define GPIO_LED_G      (GPIO_PIN27)
#define GPIO_LED_B      (GPIO_PIN6)

using namespace DriverFramework;
using namespace navio_gpio;

class RGBLED : public DevObj
{
public:
	RGBLED(const char *name) :
		DevObj(name,
		       RGBLED_DEVICE_PATH,
		       RGBLED_BASE_DEVICE_PATH,
		       DeviceBusType_UNKNOWN,
		       0),
		_rgbsets{
		{0, 0, 0}, /* OFF */
		{1, 0, 0}, /* red */
		{1, 1, 0}, /* yellow */
		{1, 0, 1}, /* purple */
		{0, 1, 0}, /* green */
		{0, 0, 1}, /* blue */
		{1, 1, 1}, /* white */
	},
	_max_color(7),
		   _rgb{0, 0, 0},
		   _turn(true)
	{ };
	virtual ~RGBLED()
	{ };

	int start();
	int stop();
	int devIOCTL(unsigned long request, unsigned long arg);

protected:
	void _measure();

private:
	Gpio _gpio;
	const rgbled_rgbset_t _rgbsets[7];
	const int _max_color;
	rgbled_rgbset_t _rgb;
	bool _turn;
};

int RGBLED::start()
{
	int res;

	res = DevObj::init();

	if (res != 0) {
		DF_LOG_ERR("error: could not init DevObj");
		return res;
	}

	_gpio.start();

	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_R);
	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_G);
	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_B);

	return 0;
}

int RGBLED::stop()
{
	int res;

	_gpio.stop();

	res = DevObj::stop();

	if (res < 0) {
		DF_LOG_ERR("error: could not stop DevObj");
		//this may not be an error for this device
		return res;
	}

	return 0;
}

int RGBLED::devIOCTL(unsigned long request, unsigned long arg)
{
	int ret = ENOTTY;
	rgbled_rgbset_t *rgb;

	switch (request) {
	case RGBLED_SET_RGB:
		ret = 0;
		rgb = (rgbled_rgbset_t *)arg;
		_rgb.red = (rgb->red != 0) ? 1 : 0;
		_rgb.green = (rgb->green != 0) ? 1 : 0;
		_rgb.blue = (rgb->blue != 0) ? 1 : 0;
		_gpio.gpiowrite(GPIO_LED_R, _rgb.red);
		_gpio.gpiowrite(GPIO_LED_G, _rgb.green);
		_gpio.gpiowrite(GPIO_LED_B, _rgb.blue);
		break;

	case RGBLED_SET_COLOR:
		if (arg > _max_color) {
			ret = ENOTSUP;

		} else {
			_rgb = _rgbsets[arg];
			_gpio.gpiowrite(GPIO_LED_R, _rgb.red);
			_gpio.gpiowrite(GPIO_LED_G, _rgb.green);
			_gpio.gpiowrite(GPIO_LED_B, _rgb.blue);
			ret = 0;
		}

		break;

	case RGBLED_SET_MODE:
		ret = 0;

		switch (arg) {
		case RGBLED_MODE_ON:
			DevObj::setSampleInterval(0);
			break;

		case RGBLED_MODE_BLINK_SLOW:
			DevObj::setSampleInterval(20000 * 1000);
			break;

		case RGBLED_MODE_BLINK_NORMAL:
			DevObj::setSampleInterval(5000 * 1000);
			break;

		case RGBLED_MODE_BLINK_FAST:
			DevObj::setSampleInterval(1000 * 1000);
			break;

		case RGBLED_MODE_BREATHE:
			DevObj::setSampleInterval(15000 * 1000);
			break;

		default:
			ret = ENOTSUP;
		}

		if (!m_work_handle.isValid()) {
			// this can fail
			DevObj::start();
		}

		break;

	case RGBLED_PLAY_SCRIPT_NAMED:
	case RGBLED_PLAY_SCRIPT:
	case RGBLED_SET_USER_SCRIPT:
	case RGBLED_SET_PATTERN:
		ret = ENOTSUP;
		break;

	default:
		ret = DevObj::devIOCTL(request, arg);
		break;
	}

	return ret;
}

void RGBLED::_measure()
{
	if (_turn) {
		_gpio.gpiowrite(GPIO_LED_R, 0);
		_gpio.gpiowrite(GPIO_LED_G, 0);
		_gpio.gpiowrite(GPIO_LED_B, 0);
		_turn = false;

	} else {
		_gpio.gpiowrite(GPIO_LED_R, _rgb.red);
		_gpio.gpiowrite(GPIO_LED_G, _rgb.green);
		_gpio.gpiowrite(GPIO_LED_B, _rgb.blue);
		_turn = true;
	}
}

extern "C" { __EXPORT int navio_rgbled_main(int argc, char *argv[]); }

namespace navio_rgbled
{
int start();
int stop();
void usage();

RGBLED *g_dev = nullptr;

int start()
{
	g_dev = new RGBLED("navio_rgbled");

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating RGBLED");
		return -1;
	}

	return g_dev->start();
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return -1;
	}

	g_dev->stop();

	delete g_dev;
	g_dev = nullptr;

	return 0;
}

void usage()
{
	PX4_WARN("Usage: navio_rgbled 'start', 'stop'");
}

} //namespace navio_rgbled

int navio_rgbled_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		navio_rgbled::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = navio_rgbled::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = navio_rgbled::stop();
	}

	else {
		navio_rgbled::usage();
		return 1;
	}

	return ret;
}

