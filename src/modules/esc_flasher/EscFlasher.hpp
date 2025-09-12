#pragma once

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/esc_flasher_request.h>
#include <uORB/topics/esc_flasher_request_ack.h>
#include <uORB/topics/esc_flasher_status.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/vehicle_command.h>

// Firefly drones have 4 ESCs
#define ESC_COUNT (4)

class ESC_Flasher : public ModuleBase<ESC_Flasher>, public ModuleParams
{
public:

	ESC_Flasher();
	~ESC_Flasher() override;

	ESC_Flasher(const ESC_Flasher &) = delete;
	ESC_Flasher operator=(const ESC_Flasher &) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ESC_Flasher *instantiate(int argc, char *argv[]);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	void RunLoop();

	void subscribe_orb_messages(void);
	void unsubscribe_orb_messages(void);

	void update_params(void);
	uint8_t get_current_arming_state(void);

	void set_esc_type(bool print);

	void load_firmware_source(void);

	int bitbang_send_packet(uint32_t gpio, uint8_t* packet, uint16_t length, uint8_t* response, uint8_t response_length);

	uint16_t make_crc(uint8_t* buffer, uint16_t length);

	void play_tune(int tune);
	void stop_tune(void);

	void set_physical_led_outputs(uint8_t color);

	void send_vehicle_command_leds(void);

	perf_counter_t	_loop_perf; /**< loop performance counter */

	uint32_t run_delay{10000};
	const uint32_t run_delay_min = 5000; //wait 0.005 sec between running the work loop (200 Hz)
	const uint32_t run_delay_max = 1000000; //wait 1 sec between running the work loop (1 Hz)
	volatile bool wake{false};
	uint32_t delay_loops;

	hrt_abstime time_now;
	hrt_abstime time_1s;
	hrt_abstime time_250ms;
	hrt_abstime time_flash_start;
	hrt_abstime time_delayed_start;

	uint32_t run_calls;

	bool print_flag; // Turn this on to enable continuous status printing

	bool flashing_in_progress{false};
	bool flashing_in_progress_last{false};

	uint8_t vehicle_armed;  // Indicates if the vehicle is armed

	uint8_t last_color{0};

	uint8_t _am32_fw_ver_major;
	uint8_t _am32_fw_ver_minor;
	enum class AM32_FIRMWARE_SOURCE {
		MEM_ARRAY,
		SD_CARD,
		STREAM_HEAP
	};
	AM32_FIRMWARE_SOURCE _am32_fw_source{AM32_FIRMWARE_SOURCE::MEM_ARRAY};

	enum class ESCType {
		Unknown = 0,
		AM32 = 1,
		BLHELI32 = 2,
		AM32_Old = 3,
		BlueJay = 4
	};
	ESCType _esc_type{ESCType::Unknown};

	const char *esc_type_unknown = "Unknown";
	const char *esc_type_am32 = "AM32";
	const char *esc_type_blheli32 = "BLHeli32";
	const char *esc_type_am32_old = "AM32_Old";
	const char *esc_type_bluejay = "BlueJay";

	const char* esc_types_strings[5] = {esc_type_unknown, esc_type_am32, esc_type_blheli32, esc_type_am32_old, esc_type_bluejay};

	enum class ESCUpdate {
		Disabled,
		Update,
		ForceUpdate,
		Recover,
		Recover2,
		Cancel
	};
	ESCUpdate _esc_update{ESCUpdate::Disabled};
	uint32_t esc_update_state{0};
	uint32_t prev_esc_update_state{0};
	uint8_t esc_update_motors[ESC_COUNT];
	uint32_t esc_update_motor_index;
	uint32_t flashing_result;

	// Struct to track flashing progress for a single ESC
	typedef struct {
		uint8_t current_motor;
		uint32_t current_gpio;
		uint32_t fw_length;
		uint32_t fw_bytes_sent;
		uint32_t fw_bytes_left;
		uint16_t base_address;
		uint16_t next_address;
		uint8_t fw_part;        // We have 2 parts to write for AM32 firmware, am32_firmware and am32_firmware_tag
		uint16_t crc;
		uint32_t tx_length;
		uint8_t tx_buffer[258];
	} ESC_FLASHING_STATE;
	ESC_FLASHING_STATE esc_flashing_state;

	const uint16_t max_chunk_length = 256;

	// Variables to save responses from DShot
	typedef struct {
		uint8_t major;
		uint8_t minor;
	} ESC_FW_VERSIONS;
	ESC_FW_VERSIONS esc_versions[ESC_COUNT];

	uint32_t esc_gpios[ESC_COUNT];
	uint32_t gpio_high;
	hrt_abstime time_gpio_set;

	// AM32 Bootloader commands
	uint8_t response_data[16];
	const uint8_t am32_boot_init[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
	const uint8_t am32_boot_init_resp_len = 9;
	uint8_t am32_set_addr[6] = {0xFF, 0, 0x10, 0x00, 0, 0};
	const uint8_t am32_set_addr_resp_len = 1;
	uint8_t am32_set_buff_size[6] = {0xFE, 0, 1, 0, 0, 0};
	const uint8_t am32_set_buff_size_resp_len = 0;
	const uint8_t am32_send_buff_resp_len = 1;
	uint8_t am32_write[4] = {0x01, 0, 0, 0};
	const uint8_t am32_write_resp_len = 1;
	const uint8_t am32_run_app[4] = {0x00, 0, 0, 0};
	const uint8_t am32_run_app_resp_len = 0;

	// These variables are useful for starting a flash from command-line
	volatile uint32_t start_flash;
	volatile uint16_t motor_flags;
	volatile uint32_t cancel_flash;

	// Request/Ack variables
	uint32_t msg_id;
	uint32_t esc_info_timeout;

	// Tune control
	uint32_t esc_flasher_tune{0}; // Play some different tunes during flashing
	uint32_t esc_flasher_stop_tune{0}; // Stop tune 1 second after starting

	// Publication variables
	uORB::Publication<esc_flasher_request_s> _esc_flasher_request_pub{ORB_ID(esc_flasher_request)};
	uORB::Publication<esc_flasher_status_s> _esc_flasher_status_pub{ORB_ID(esc_flasher_status)};
	uORB::Publication<tune_control_s> _tune_control_pub{ORB_ID(tune_control)};
	uORB::PublicationData<led_control_s> _led_control_pub{ORB_ID(led_control)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	// Subscription variables
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _esc_flasher_request_ack_sub{ORB_ID(esc_flasher_request_ack)};
	uORB::Subscription _sensor_gps_sub {ORB_ID(sensor_gps)};
	uORB::Subscription _vehicle_status_sub {ORB_ID(vehicle_status)};

	// MAVLink
	orb_advert_t _mavlink_log_pub{nullptr};

	sensor_gps_s _sensor_gps {};                       /**< gps sensor */
	vehicle_status_s _vehicle_status {};                   /**< vehicle status */

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ESC_TYPE>) _param_esc_type,
		(ParamInt<px4::params::ESC_UPDATE>) _param_esc_update
	)

};
