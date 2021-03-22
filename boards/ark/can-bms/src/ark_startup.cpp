#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/smbus/SMBus.hpp>
#include <uORB/topics/battery_status.h>

#define BATT_SMBUS_ADDR                                 0x0B
#define BATT_SMBUS_VOLTAGE                              0x09
#define BATT_SMBUS_CURRENT                              0x0A
#define BATT_SMBUS_RELATIVE_SOC							0x0D
#define MAC_BA_DATA_BUFFER_SIZE                         32 // MAC buffer size
#define BQ40Z80_MAC_BA_DASTATUS1                        0x0071 ///< returns the cell voltages, pack voltage, bat voltage, cell currents, cell powers, power, and average power
#define BQ40Z80_DASTATUS1_PACK_VOLTAGE_MSB              11
#define BQ40Z80_DASTATUS1_PACK_VOLTAGE_LSB              10
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS            0x44

static SMBus* _interface;

static float _voltage_mv = 0;
static float _current_ma = 0;
static float _soc = 0;

// Expose these functions for use in init.c
__BEGIN_DECLS
extern void ark_bq_startup_init(void);
extern void ark_check_button();
extern void ark_clean_up();
__END_DECLS

void update_battery_data(void);
void update_led_status(void);
bool check_pack_voltage(void);
int manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length);

void ark_bq_startup_init(void)
{
	int address = BATT_SMBUS_ADDR;
	int bus = 1;

	// Create the SMBus interface and initialize it
	_interface = new SMBus(bus, address);
	_interface->init();

	up_udelay(10000);
}

void ark_clean_up(void)
{
	delete _interface;
}

void ark_check_button()
{
	static bool button_was_held = true;
	static constexpr uint64_t BUTTON_SHUTDOWN_DURATION_US = 3000000;

	auto start_time = hrt_absolute_time();
	auto time_now = start_time;

	// Button should be held for 2.5 out of 3 seconds
	while (time_now < (start_time + BUTTON_SHUTDOWN_DURATION_US)) {
		update_battery_data();

		// If we have voltage on PACK+ then boot immediately
		if (check_pack_voltage()) {
			uint64_t elapsed = time_now - start_time;
			printf("PACK Voltage took: %d us to detect\n", (int)elapsed);
			return;
		}

		up_udelay(50000); // 100ms -- because our loop delay is wrong

		bool button_held = !stm32_gpioread(GPIO_BTN_N);

		if (button_held) {

			if (!button_was_held) {
				start_time = hrt_absolute_time();
				button_was_held = true;
			}

			printf("Keep holding that button big guy\n");

		} else {

			if (button_was_held) {
				start_time = hrt_absolute_time();
				button_was_held = false;
				printf("aww you let go\n");
			}
		}

		time_now = hrt_absolute_time();
	}

	if (button_was_held) {
		// Good to go
		return;

	} else {
			printf("Button not held, powering off\n");
			stm32_gpiowrite(GPIO_nLED_RED, true);
			stm32_gpiowrite(GPIO_nLED_BLUE, true);
			stm32_gpiowrite(GPIO_PWR_EN, false);
			while(1){;};
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

void update_battery_data(void)
{
	uint16_t result = 0;

	// Get the voltage
	_interface->read_word(BATT_SMBUS_VOLTAGE, result);
	_voltage_mv = ((float)result) / 1000.0f;

	// Get the current
	_interface->read_word(BATT_SMBUS_CURRENT, result);
	_current_ma = result;

	// Get the state of charge
	_interface->read_word(BATT_SMBUS_RELATIVE_SOC, result);
	_soc = ((float)result) / 100.0f;
}

bool check_pack_voltage(void)
{
	// Check PACK to determine if FETs are open or if we are connected to a charger
	uint8_t status[MAC_BA_DATA_BUFFER_SIZE];
	int result = manufacturer_read(BQ40Z80_MAC_BA_DASTATUS1, &status, MAC_BA_DATA_BUFFER_SIZE);

	if (result != PX4_OK) {
		printf("reading DAStatus1 failed\n");
		return true;
	}

	float pack_voltage = (float)(((uint16_t)status[BQ40Z80_DASTATUS1_PACK_VOLTAGE_MSB] << 8) | status[BQ40Z80_DASTATUS1_PACK_VOLTAGE_LSB]) / 1000.0f;

	if (pack_voltage > 3.5f) {
		printf("pack voltage: %f\n", (double)pack_voltage);
		return true;
	}

	return false;
}

int manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	int result = _interface->block_write(code, address, 2, false);

	if (result != PX4_OK) {
		return result;
	}

	// Intermediary buffer to ensure we read the entire returned block
	uint8_t buf[34] = {};

	result = _interface->block_read(code, buf, length + 1, false);

	// Drop the 2 leading bytes of address info
	memcpy(data, &buf[2], length);

	return result;
}
