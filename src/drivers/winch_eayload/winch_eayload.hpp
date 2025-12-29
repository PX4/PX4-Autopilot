#pragma once
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/winch_status.h>
#include <uORB/topics/winch_control.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
using namespace time_literals;
// Protocol Constants
#define PAYLOAD_FRAME_HEADER 0xEB
#define PAYLOAD_FUNC_WRITE_SINGLE 0x01
#define PAYLOAD_FUNC_READ_RESPONSE 0x03
#define PAYLOAD_FUNC_READ_REQUEST 0x04
#define PAYLOAD_FUNC_WRITE_MULTI 0x10
// Command Register Addresses
#define REG_CMD_STOP 10101
#define REG_CMD_DESCENT 10102
#define REG_CMD_ASCENT 10104
#define REG_CMD_HOOK_OPEN 10106
#define REG_CMD_HOOK_CLOSE 10107
#define REG_CMD_ROPE_CUT 10110
#define REG_CMD_FIXED_FRAME_MODE 10109
// Status Register Addresses
#define REG_STATUS_START 30000
#define REG_STATUS_COUNT 21
#define REG_WINCH_STATE 30003
#define REG_MOTOR_STATE 30004
#define REG_HOOK_STATE 30005
#define REG_ROPE_LENGTH 30011
#define REG_LOAD_WEIGHT 30013
#define REG_RELEASE_VOLTAGE 30019
// Buffer sizes
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 64
#define MAX_FRAME_SIZE 128
// Module name
#define MODULE_NAME "winch_payload"
enum class ConnectionState {
DISCONNECTED,
CONNECTED,
RECONNECTING
};
enum class ParseState {
WAIT_HEADER,
WAIT_FUNCTION,
WAIT_ADDRESS,
WAIT_DATA,
WAIT_CRC
};
class WinchPayload : public ModuleBase<WinchPayload>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
WinchPayload();
~WinchPayload() override;
static int task_spawn(int argc, char *argv[]);
static int custom_command(int argc, char *argv[]);
static int print_usage(const char *reason = nullptr);
bool init();
int print_status() override;
private:
void Run() override;
// Serial communication
int open_serial_port(const char *port);
void close_serial_port();
// Protocol functions
uint16_t calculate_crc16(const uint8_t *data, uint16_t length);
int apply_byte_stuffing(const uint8_t *input, uint16_t input_len, uint8_t *output, uint16_t *output_len);
int remove_byte_stuffing(const uint8_t *input, uint16_t input_len, uint8_t *output, uint16_t *output_len);
// Frame handling
int build_write_frame(uint16_t reg_addr, uint8_t value, uint8_t *frame, uint16_t *frame_len);
int build_read_frame(uint16_t start_addr, uint16_t count, uint8_t *frame, uint16_t *frame_len);
int send_frame(const uint8_t *frame, uint16_t length);
int receive_frame(uint8_t *frame, uint16_t *length, uint32_t timeout_ms);
int parse_status_response(const uint8_t *data, uint16_t length);
// Command functions
int send_command_with_retry(uint16_t reg_addr, uint8_t value, int max_retries);
bool validate_response(const uint8_t *data, uint16_t length, uint8_t expected_func);
int send_stop_command();
int send_descent_command();
int send_ascent_command();
int send_hook_open_command();
int send_hook_close_command();
int send_rope_cut_command();
int enable_fixed_frame_mode(bool enable);
int request_status();
// Helpers
bool feed_parser_byte(uint8_t byte, uint8_t *frame, uint16_t *pos, ParseState *state,
uint16_t *expected_data_len, uint16_t *data_received);
void handle_comm_error();
void check_connection_health();
// Processing
void process_winch_control();
void process_incoming_data();
// State name helpers
const char *get_winch_state_name(uint8_t state);
const char *get_motor_state_name(uint8_t state);
const char *get_hook_state_name(uint8_t state);
// Serial port
int _serial_fd{-1};
const char *_port{"/dev/ttyS3"};
// Parser state (for fixed mode)
uint8_t _parse_buffer[MAX_FRAME_SIZE]{};
uint16_t _parse_pos{0};
ParseState _parse_state{ParseState::WAIT_HEADER};
uint16_t _expected_data_len{0};
uint16_t _data_received{0};
// Receive buffer (legacy, if needed)
uint8_t _rx_buffer[RX_BUFFER_SIZE]{};
uint16_t _rx_buffer_pos{0};
// Publications
uORB::Publication<winch_status_s> _winch_status_pub{ORB_ID(winch_status)};
// Subscriptions
uORB::Subscription _winch_control_sub{ORB_ID(winch_control)};
// Current status
winch_status_s _winch_status{};
// Timing
hrt_abstime _last_status_time{0};
hrt_abstime _last_command_time{0};
hrt_abstime _last_successful_comm{0};
hrt_abstime _last_reconnect_attempt{0};
// State tracking
ConnectionState _connection_state{ConnectionState::DISCONNECTED};
bool _fixed_frame_mode{false};
uint8_t _device_address{0x01};
uint16_t _consecutive_errors{0};
uint16_t _crc_error_count{0};
// Performance counters
perf_counter_t _loop_perf{nullptr};
perf_counter_t _comms_errors{nullptr};
perf_counter_t _crc_errors{nullptr};
perf_counter_t _timeout_errors{nullptr};
perf_counter_t _reconnect_count{nullptr};
// Parameters
DEFINE_PARAMETERS(
(ParamInt<px4::params::WINCH_EN>) _param_winch_en,
(ParamInt<px4::params::WINCH_ADDR>) _param_winch_addr,
(ParamInt<px4::params::WINCH_MODE>) _param_winch_mode
)
};
