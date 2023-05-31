#include "load_CAN.h"
#include "load_CAN_app.h"

#include "led_hw.h"
#include "pwm_hw.h"

#include <string.h>
#include <stdio.h>
#include "stdbool.h"
#include <stddef.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

__EXPORT int FF_SystemLoad_main(int argc, char *argv[]);

static int status = 0;
const char ext_component_path[] = "/fs/microsd/ext_component_updated";

int FF_SystemLoad_main(int argc, char *argv[])
{
	PX4_INFO("FF Systemloader V.1.3.0");
	status = 0;

	if (argc < 2) {
		PX4_INFO("Expected arguments 'ver', 'load', 'id', or 'jump'");
		return 1;
	}

	if (!strcmp(argv[1], "load")) {//force firmware load
		sl_led_init();
		sl_rgbled_set_color_and_mode(LED_CONTROL_COLOR_PURPLE, LED_CONTROL_MODE_BLINK_FAST, 255,
					     LED_CONTROL_MAX_PRIORITY); //note that systemloader process cannot exceed the time it takes to hit 255 blinks.

		PX4_INFO("Starting load");

		if (can_load_all_modules() != PROCESS_SUCCESS) {
			status = 1;
		}

		if (status == 0) {  // Only test if previous step succeded
			PX4_INFO("Starting ID");

			if (pwmPinsInit()) {
				DEBUG_OUTPUT("Unable to initialize addressing IO");
				status = 2;
			}
		}

		if (post_finish_load_callback() != PROCESS_SUCCESS) {
			DEBUG_OUTPUT("Post load error checking failed.");
			status = 4;
		}

		// Did we succeed or not?
		if (status == 0) {
			//success
			PX4_INFO("Success");
			FILE *fp = fopen(ext_component_path, "w");
			char buffer[100];

			if (fp == NULL) {
				PX4_ERR("Error unable to open file");
				return status;
			}

			PX4_INFO("Writing ext_component_path");
			sprintf(buffer, "%s %d \n", "SUCCESS", 0);
			fputs(buffer, fp);
			fclose(fp);

			sl_rgbled_set_color_and_mode(LED_CONTROL_COLOR_PURPLE, LED_CONTROL_MODE_ON, 255, LED_CONTROL_MAX_PRIORITY);

		} else {
			//failure
			PX4_INFO("Failure");
			FILE *fp = fopen(ext_component_path, "w");
			char buffer[100];

			if (fp == NULL) {
				PX4_ERR("Error unable to open file");
				return status;
			}

			PX4_INFO("Writing ext_component_path");
			sprintf(buffer, "%s %d \n", "FAIL", status);
			fputs(buffer, fp);
			fclose(fp);
			sl_rgbled_set_color_and_mode(LED_CONTROL_COLOR_RED, LED_CONTROL_MODE_ON, 255, LED_CONTROL_MAX_PRIORITY);
		}

		DELAY_MS(3000);
		sl_led_deinit();
		pwmPinsDeInit();
		PX4_INFO("Finished load");

	} else if (!strcmp(argv[1], "ver")) { //print loaded versions
		if (print_loaded_versions()) {
			status = 1;
		}

	} else if (!strcmp(argv[1], "jump")) { //jump modules to application
		initAllCanPorts();
		PX4_INFO("Jumping CAN modules into application");
		jump_all_can_modules_to_application();
		deInitAllCanPorts();

	} else if (!strcmp(argv[1], "pwm_test")) {
		PX4_INFO("Testing PWM as GPIO");
		pwmPinsInit();
		PX4_INFO("Initialized");

		for (int i = 0; i < 4; i++) {
			PX4_INFO("Testing Pin %d", i);
			set_pwm_output(i, true);
			DELAY_MS(1000);
			set_pwm_output(i, false);
		}

		pwmPinsDeInit();
		PX4_INFO("Uninitialized");
	}

	// } else if (!strcmp(argv[1], "id")) { //ID ESCs booms.  Requires ESCs to already have been jumped to application
	// 	initAllCanPorts();

	// 	if (pwmPinsInit()) {
	// 		DEBUG_OUTPUT("Unable to initialize addressing IO");
	// 		return PROCESS_FAILURE;
	// 	}

	// 	PX4_INFO("Assigning ESC boom IDs");
	// 	assign_all_esc_boom_ids();
	// 	deInitAllCanPorts();
	// 	pwmPinsDeInit();
	// }

	return status;
}
