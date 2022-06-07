#include <string.h>
#include <px4_platform_common/px4_config.h>

#include <lib/led/led.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_apa102.h>
#include <drivers/device/spi.h>

class APA102 : public device::SPI, public px4::ScheduledWorkItem, public ModuleBase<APA102>
{
public:
	APA102(unsigned int number_of_packages, int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	virtual ~APA102();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;


	int			init();
	int			status();

private:
	// We don't want to use BOARD_HAS_N_S_RGB_LED, SPI leds are external
	unsigned int _number_of_packages;

	uint8_t buf[(BOARD_MAX_LEDS * 4) + 8];
	uint8_t rbuf[(BOARD_MAX_LEDS * 4) + 8];

	// LedController is a driver that takes care of LEDs using the led_control ORB msg.
	// Do we really want to use this?
	LedController		_led_controller;

	// ??
	APA102(const APA102 &) = delete;
	APA102 operator=(const APA102 &) = delete;

	// What is this?
	// _leds is the actual buffer I believe
	apa102::APA102LEDData *_leds;
};