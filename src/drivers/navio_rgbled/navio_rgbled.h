
#include "DevObj.hpp"

#include "navio_gpio.h"

#define RGBLED_BASE_DEVICE_PATH "/dev/rgbled"
#define RGBLED_DEVICE_PATH  "/dev/rgbled0"

// inverted
#define LED_ON  0
#define LED_OFF 1


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
		{LED_OFF, LED_OFF, LED_OFF}, /* OFF */
		{LED_ON,  LED_OFF, LED_OFF}, /* red */
		{LED_ON,  LED_ON,  LED_OFF}, /* yellow */
		{LED_ON,  LED_OFF, LED_ON},  /* purple */
		{LED_OFF, LED_ON,  LED_OFF}, /* green */
		{LED_OFF, LED_OFF, LED_ON},  /* blue */
		{LED_ON,  LED_ON,  LED_ON},  /* white */
	},
	_max_color(7),
		   _rgb{LED_OFF, LED_OFF, LED_OFF},
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
