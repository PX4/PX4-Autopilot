#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <stdio.h>

#include "pwm_hw.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include <uORB/topics/actuator_test.h>
#ifdef __cplusplus
}
#endif

#define MOTOR_COUNT 4 // Maximum 8 on IO output

static orb_advert_t actuator_test_pub = NULL;
static bool initialized = false;

int32_t backup_disarmed[MOTOR_COUNT] = {};
int32_t backup_minimum[MOTOR_COUNT] = {};
int32_t backup_maximum[MOTOR_COUNT] = {};

bool pwmPinsInit(void)
{
	actuator_test_pub = orb_advertise(ORB_ID(actuator_test), NULL);

	struct actuator_test_s actuator_test = {};
	actuator_test.value = -1.f;
	actuator_test.action = ACTUATOR_TEST_ACTION_DO_CONTROL;

	for (int i = 0; i < MOTOR_COUNT; ++i) {
		// Set parameters to use PWM outputs as GPIO
		char buffer[14];
		snprintf(buffer, sizeof(buffer), "PWM_MAIN_DIS%u", i + 1);
		param_t param = param_find(buffer);
		param_get(param, &backup_disarmed[i]);
		int32_t value = 0; // minimum output results in low signal
		param_set_no_notification(param, &value);

		snprintf(buffer, sizeof(buffer), "PWM_MAIN_MIN%u", i + 1);
		param = param_find(buffer);
		param_get(param, &backup_minimum[i]);
		param_set_no_notification(param, &value);

		snprintf(buffer, sizeof(buffer), "PWM_MAIN_MAX%u", i + 1);
		param = param_find(buffer);
		param_get(param, &backup_maximum[i]);
		value = 2550; // maximum output results in high signal with 400Hz PWM
		param_set(param, &value);

		// Take control over PWMs and pull them low
		actuator_test.timestamp = hrt_absolute_time();
		actuator_test.function = ACTUATOR_TEST_FUNCTION_MOTOR1 + i;
		orb_publish(ORB_ID(actuator_test), actuator_test_pub, &actuator_test);
	}

	usleep(50 * 1000); // Avoid race condition where the PWM module did not apply the parameters yet
	usleep(500 * 1000); // Wait until the PWM ramp of the motor test is done

	initialized = true;
	return false;
}

void pwmPinsDeInit(void)
{
	if (!initialized) { return; }

	struct actuator_test_s actuator_test = {};

	actuator_test.value = -1.f;

	actuator_test.action = ACTUATOR_TEST_ACTION_RELEASE_CONTROL;

	for (int i = 0; i < MOTOR_COUNT; ++i) {
		// Restore PWM parameters
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "PWM_MAIN_DIS%u", i + 1);
		param_t param = param_find(buffer);
		// param_set_no_notification(param, &backup_disarmed[i]);
		param_reset_no_notification(param);

		snprintf(buffer, sizeof(buffer), "PWM_MAIN_MIN%u", i + 1);
		param = param_find(buffer);
		// param_set_no_notification(param, &backup_minimum[i]);
		param_reset_no_notification(param);

		snprintf(buffer, sizeof(buffer), "PWM_MAIN_MAX%u", i + 1);
		param = param_find(buffer);
		// param_set(param, &backup_maximum[i]);
		param_reset(param);

		// Release control over PWMs
		actuator_test.timestamp = hrt_absolute_time();
		actuator_test.function = ACTUATOR_TEST_FUNCTION_MOTOR1 + i;
		orb_publish(ORB_ID(actuator_test), actuator_test_pub, &actuator_test);
	}

	usleep(50 * 1000); // Avoid race condition where the PWM module did not apply the parameters yet

	orb_unadvertise(actuator_test_pub);
}

void set_pwm_output(unsigned long channel, bool isValueHigh)
{
	struct actuator_test_s actuator_test = {};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.value = isValueHigh ? 1.f : -1.f;
	actuator_test.action = ACTUATOR_TEST_ACTION_DO_CONTROL;
	actuator_test.function = ACTUATOR_TEST_FUNCTION_MOTOR1 + channel;
	orb_publish(ORB_ID(actuator_test), actuator_test_pub, &actuator_test);
}
