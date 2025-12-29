#include "winch_payload.hpp"
#include <px4_platform_common/log.h>
#include <errno.h>
#include <string.h>
#include <poll.h>

// Error thresholds
#define MAX_CONSECUTIVE_ERRORS 5
#define MAX_CRC_ERRORS 10
#define RECONNECT_DELAY_MS 1000
#define COMMAND_TIMEOUT_MS 200
#define STATUS_TIMEOUT_MS 500
#define COMM_LOSS_TIMEOUT_US 2000000ULL // 2 seconds

// Frame parsing states
enum class ParseState {
    WAIT_HEADER,
    WAIT_FUNCTION,
    WAIT_ADDRESS,
    WAIT_DATA,
    WAIT_CRC
};

WinchPayload::WinchPayload() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
    memset(&_winch_status, 0, sizeof(_winch_status));
    _parse_state = ParseState::WAIT_HEADER;
    _parse_pos = 0;
}

WinchPayload::~WinchPayload()
{
    ScheduleClear();
    close_serial_port();
    perf_free(_loop_perf);
    perf_free(_comms_errors);
    perf_free(_crc_errors);
    perf_free(_timeout_errors);
    perf_free(_reconnect_count);
}

bool WinchPayload::init()
{
    _device_address = _param_winch_addr.get();
    if (!attempt_connection()) {
        PX4_WARN("Initial connection failed, will retry...");
        _connection_state = ConnectionState::DISCONNECTED;
    }
    ScheduleOnInterval(20_ms); // 50Hz
    return true;
}

bool WinchPayload::attempt_connection()
{
    if (_serial_fd >= 0) {
        close_serial_port();
    }
    if (open_serial_port(_port) < 0) {
        PX4_ERR("Failed to open serial port %s", _port);
        return false;
    }
    // Flush any stale data
    tcflush(_serial_fd, TCIOFLUSH);
    // Reset error counters on new connection
    _consecutive_errors = 0;
    _crc_error_count = 0;
    // Enable fixed frame mode
    if (_param_winch_mode.get() == 1) {
        px4_usleep(100000); // Wait 100ms for winch to be ready
        if (enable_fixed_frame_mode(true) == 0) {
            _fixed_frame_mode = true;
            PX4_INFO("Fixed frame mode enabled");
        } else {
            PX4_WARN("Failed to enable fixed frame mode, using polling");
            _fixed_frame_mode = false;
        }
    }
    _connection_state = ConnectionState::CONNECTED;
    _last_successful_comm = hrt_absolute_time();
    perf_count(_reconnect_count);
    PX4_INFO("Winch connected on %s, address: 0x%02X", _port, _device_address);
    return true;
}

int WinchPayload::open_serial_port(const char *port)
{
    _serial_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_serial_fd < 0) {
        PX4_ERR("Failed to open %s: %s", port, strerror(errno));
        return -1;
    }
    struct termios config;
    if (tcgetattr(_serial_fd, &config) < 0) {
        PX4_ERR("Failed to get terminal attributes");
        close(_serial_fd);
        _serial_fd = -1;
        return -1;
    }
    // Clear config
    memset(&config, 0, sizeof(config));
    // Set baud rate: 115200
    cfsetispeed(&config, B115200);
    cfsetospeed(&config, B115200);
    // 8N1 mode
    config.c_cflag |= (CLOCAL | CREAD);
    config.c_cflag &= ~PARENB;
    config.c_cflag &= ~CSTOPB;
    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;
    // Raw input mode
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    config.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNBRK);
    config.c_oflag &= ~OPOST;
    // Non-blocking with timeout
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 1;
    if (tcsetattr(_serial_fd, TCSANOW, &config) < 0) {
        PX4_ERR("Failed to set terminal attributes");
        close(_serial_fd);
        _serial_fd = -1;
        return -1;
    }
    tcflush(_serial_fd, TCIOFLUSH);
    return 0;
}

void WinchPayload::close_serial_port()
{
    if (_serial_fd >= 0) {
        close(_serial_fd);
        _serial_fd = -1;
    }
}

uint16_t WinchPayload::calculate_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int WinchPayload::apply_byte_stuffing(const uint8_t *input, uint16_t input_len,
                                      uint8_t *output, uint16_t *output_len)
{
    if (input_len == 0 || output == nullptr || output_len == nullptr) {
        return -1;
    }
    uint16_t out_pos = 0;
    // First byte is header, don't stuff it
    output[out_pos++] = input[0];
    // Process remaining bytes
    for (uint16_t i = 1; i < input_len; i++) {
        if (out_pos >= TX_BUFFER_SIZE - 2) {
            PX4_ERR("Byte stuffing buffer overflow");
            return -1;
        }
        output[out_pos++] = input[i];
        // If byte is header, insert 0x00 after it
        if (input[i] == PAYLOAD_FRAME_HEADER) {
            output[out_pos++] = 0x00;
        }
    }
    *output_len = out_pos;
    return 0;
}

int WinchPayload::remove_byte_stuffing(const uint8_t *input, uint16_t input_len,
                                       uint8_t *output, uint16_t *output_len)
{
    if (input_len == 0 || output == nullptr || output_len == nullptr) {
        return -1;
    }
    uint16_t out_pos = 0;
    for (uint16_t i = 0; i < input_len; i++) {
        if (out_pos >= MAX_FRAME_SIZE) {
            PX4_ERR("Unstuffing buffer overflow");
            return -1;
        }
        output[out_pos++] = input[i];
        // Skip stuffed 0x00 after header (but not for the header itself)
        if (i > 0 && input[i] == PAYLOAD_FRAME_HEADER &&
            (i + 1) < input_len && input[i + 1] == 0x00) {
            i++; // Skip the stuffed 0x00
        }
    }
    *output_len = out_pos;
    return 0;
}

int WinchPayload::send_frame(const uint8_t *frame, uint16_t length)
{
    if (_serial_fd < 0 || _connection_state != ConnectionState::CONNECTED) {
        return -1;
    }
    // Flush input buffer before sending
    tcflush(_serial_fd, TCIFLUSH);
    int total_written = 0;
    int retries = 3;
    while (total_written < (int)length && retries > 0) {
        int written = write(_serial_fd, frame + total_written, length - total_written);
        if (written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                px4_usleep(1000);
                retries--;
                continue;
            }
            PX4_ERR("Write error: %s", strerror(errno));
            handle_comm_error();
            return -1;
        }
        total_written += written;
    }
    if (total_written != (int)length) {
        PX4_ERR("Incomplete write: %d/%d bytes", total_written, length);
        handle_comm_error();
        return -1;
    }
    _last_command_time = hrt_absolute_time();
    return 0;
}

int WinchPayload::send_command_with_retry(uint16_t reg_addr, uint8_t value, int max_retries)
{
    uint8_t frame[TX_BUFFER_SIZE];
    uint16_t frame_len;
    if (build_write_frame(reg_addr, value, frame, &frame_len) < 0) {
        return -1;
    }
    for (int retry = 0; retry < max_retries; retry++) {
        if (send_frame(frame, frame_len) == 0) {
            // Wait for acknowledgment
            uint8_t response[MAX_FRAME_SIZE];
            uint16_t response_len;
            if (receive_frame(response, &response_len, COMMAND_TIMEOUT_MS) == 0) {
                // Verify response
                if (validate_response(response, response_len, PAYLOAD_FUNC_WRITE_SINGLE)) {
                    _consecutive_errors = 0;
                    _last_successful_comm = hrt_absolute_time();
                    return 0;
                }
            }
        }
        PX4_DEBUG("Command retry %d/%d for register %d", retry + 1, max_retries, reg_addr);
        px4_usleep(50000); // 50ms between retries
    }
    handle_comm_error();
    return -1;
}

bool WinchPayload::validate_response(const uint8_t *data, uint16_t length, uint8_t expected_func)
{
    if (length < 5) {
        return false;
    }
    // Check header
    if (data[0] != PAYLOAD_FRAME_HEADER) {
        return false;
    }
    // Check function code
    if (data[1] != expected_func) {
        return false;
    }
    // Verify CRC
    uint8_t unstuffed[MAX_FRAME_SIZE];
    uint16_t unstuffed_len;
    if (remove_byte_stuffing(data, length, unstuffed, &unstuffed_len) < 0) {
        return false;
    }
    if (unstuffed_len < 4) {
        return false;
    }
    uint16_t received_crc = unstuffed[unstuffed_len - 2] | (unstuffed[unstuffed_len - 1] << 8);
    uint16_t calculated_crc = calculate_crc16(&unstuffed[0], unstuffed_len - 2);
    if (received_crc != calculated_crc) {
        perf_count(_crc_errors);
        _crc_error_count++;
        PX4_DEBUG("CRC mismatch: recv=0x%04X calc=0x%04X", received_crc, calculated_crc);
        return false;
    }
    return true;
}

int WinchPayload::receive_frame(uint8_t *frame, uint16_t *length, uint32_t timeout_ms)
{
    if (_serial_fd < 0) {
        return -1;
    }
    struct pollfd fds;
    fds.fd = _serial_fd;
    fds.events = POLLIN;
    hrt_abstime start_time = hrt_absolute_time();
    uint16_t pos = 0;
    ParseState state = ParseState::WAIT_HEADER;
    uint16_t expected_data_len = 0;
    uint16_t data_received = 0;
    while ((hrt_absolute_time() - start_time) < (timeout_ms * 1000ULL)) {
        int ret = poll(&fds, 1, 10);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            PX4_ERR("Poll error: %s", strerror(errno));
            return -1;
        }
        if (ret == 0) {
            continue; // Timeout, keep waiting
        }
        if (!(fds.revents & POLLIN)) {
            continue;
        }
        uint8_t byte;
        int bytes_read = read(_serial_fd, &byte, 1);
        if (bytes_read <= 0) {
            if (bytes_read < 0 && errno != EAGAIN) {
                PX4_ERR("Read error: %s", strerror(errno));
                return -1;
            }
            continue;
        }
        // Feed the byte to the parser
        if (feed_parser_byte(byte, frame, &pos, &state, &expected_data_len, &data_received)) {
            *length = pos;
            return 0; // Frame complete
        }
        if (pos >= MAX_FRAME_SIZE - 1) {
            PX4_WARN("Frame buffer overflow, resetting parser");
            state = ParseState::WAIT_HEADER;
            pos = 0;
        }
    }
    perf_count(_timeout_errors);
    return -1; // Timeout
}

// New helper function to feed a single byte to the parser (used in both receive_frame and process_incoming_data)
bool WinchPayload::feed_parser_byte(uint8_t byte, uint8_t *frame, uint16_t *pos, ParseState *state,
                                    uint16_t *expected_data_len, uint16_t *data_received)
{
    switch (*state) {
    case ParseState::WAIT_HEADER:
        if (byte == PAYLOAD_FRAME_HEADER) {
            *pos = 0;
            frame[*pos] = byte;
            (*pos)++;
            *state = ParseState::WAIT_FUNCTION;
        }
        break;
    case ParseState::WAIT_FUNCTION:
        frame[*pos] = byte;
        (*pos)++;
        if (byte == PAYLOAD_FUNC_READ_RESPONSE) {
            *state = ParseState::WAIT_ADDRESS;
        } else if (byte == PAYLOAD_FUNC_WRITE_SINGLE) {
            *expected_data_len = 4; // addr(1) + reg(2) + value(1)
            *data_received = 0;
            *state = ParseState::WAIT_DATA;
        } else {
            // Unknown function code, reset
            PX4_DEBUG("Unknown function code: 0x%02X", byte);
            *state = ParseState::WAIT_HEADER;
        }
        break;
    case ParseState::WAIT_ADDRESS:
        frame[*pos] = byte;
        (*pos)++;
        // Next bytes are start address (2) + count (2)
        *expected_data_len = 4;
        *data_received = 0; // Reset for data after address
        *state = ParseState::WAIT_DATA;
        break;
    case ParseState::WAIT_DATA:
        frame[*pos] = byte;
        (*pos)++;
        (*data_received)++;
        // For read response, after getting count, calculate expected data length
        if (frame[1] == PAYLOAD_FUNC_READ_RESPONSE && *data_received == 4) {
            uint16_t reg_count = (frame[*pos - 2] << 8) | frame[*pos - 1]; // count h/l just received
            *expected_data_len += (reg_count * 2);
        }
        if (*data_received >= *expected_data_len) {
            *state = ParseState::WAIT_CRC;
            *data_received = 0;
        }
        break;
    case ParseState::WAIT_CRC:
        frame[*pos] = byte;
        (*pos)++;
        (*data_received)++;
        if (*data_received >= 2) {
            return true; // Frame complete
        }
        break;
    }
    // Handle byte stuffing: the state machine stores stuffed bytes; unstuffing happens later
    return false;
}

int WinchPayload::parse_status_response(const uint8_t *data, uint16_t length)
{
    if (length < 10) {
        PX4_DEBUG("Frame too short: %d bytes", length);
        return -1;
    }
    // Remove byte stuffing
    uint8_t unstuffed[MAX_FRAME_SIZE];
    uint16_t unstuffed_len;
    if (remove_byte_stuffing(data, length, unstuffed, &unstuffed_len) < 0) {
        PX4_DEBUG("Failed to remove byte stuffing");
        return -1;
    }
    // Validate frame structure
    if (unstuffed[0] != PAYLOAD_FRAME_HEADER) {
        PX4_DEBUG("Invalid header: 0x%02X", unstuffed[0]);
        return -1;
    }
    if (unstuffed[1] != PAYLOAD_FUNC_READ_RESPONSE) {
        PX4_DEBUG("Not a read response: 0x%02X", unstuffed[1]);
        return -1;
    }
    // Verify CRC
    if (unstuffed_len < 4) {
        return -1;
    }
    uint16_t received_crc = unstuffed[unstuffed_len - 2] | (unstuffed[unstuffed_len - 1] << 8);
    uint16_t calculated_crc = calculate_crc16(&unstuffed[0], unstuffed_len - 2);
    if (received_crc != calculated_crc) {
        perf_count(_crc_errors);
        _crc_error_count++;
        PX4_DEBUG("CRC error: recv=0x%04X calc=0x%04X", received_crc, calculated_crc);
        if (_crc_error_count > MAX_CRC_ERRORS) {
            PX4_WARN("Too many CRC errors, triggering reconnection");
            handle_comm_error();
        }
        return -1;
    }
    // Reset CRC error count on successful parse
    _crc_error_count = 0;
    // Parse register data
    uint16_t start_addr = (unstuffed[3] << 8) | unstuffed[4];
    uint16_t reg_count = (unstuffed[5] << 8) | unstuffed[6];
    // Validate data length
    uint16_t expected_len = 9 + (reg_count * 2); // header(1) + func(1) + addr(1) + start(2) + count(2) + data + crc(2)
    if (unstuffed_len < expected_len) {
        PX4_DEBUG("Data length mismatch: got %d, expected %d", unstuffed_len, expected_len);
        return -1;
    }
    const uint8_t *reg_data = &unstuffed[7];
    // Update status
    _winch_status.timestamp = hrt_absolute_time();
    _winch_status.device_address = _device_address;
    for (uint16_t i = 0; i < reg_count && i < REG_STATUS_COUNT; i++) {
        uint16_t reg_addr = start_addr + i;
        int16_t value = (reg_data[i * 2] << 8) | reg_data[i * 2 + 1];
        switch (reg_addr) {
        case REG_WINCH_STATE:
            _winch_status.winch_state = (uint8_t)(value & 0xFF);
            break;
        case REG_MOTOR_STATE:
            _winch_status.motor_state = (uint8_t)(value & 0xFF);
            break;
        case REG_HOOK_STATE:
            _winch_status.hook_state = (uint8_t)(value & 0xFF);
            break;
        case REG_ROPE_LENGTH:
            _winch_status.rope_length = value;
            break;
        case REG_LOAD_WEIGHT:
            _winch_status.load_weight = value;
            break;
        case REG_RELEASE_VOLTAGE:
            _winch_status.release_module_voltage = value;
            break;
        default:
            break;
        }
    }
    // Update communication status
    _winch_status.comm_link_status = 1;
    _winch_status.system_healthy = (_winch_status.winch_state < 8) ? 1 : 0;
    // Check for fault states
    if (_winch_status.motor_state >= 8) {
        PX4_WARN("Motor fault detected: state=%d", _winch_status.motor_state);
    }
    if (_winch_status.winch_state >= 10) {
        PX4_WARN("Winch fault detected: state=%d", _winch_status.winch_state);
    }
    // Publish and update timing
    _winch_status_pub.publish(_winch_status);
    _last_status_time = hrt_absolute_time();
    _last_successful_comm = hrt_absolute_time();
    _consecutive_errors = 0;
    return 0;
}

void WinchPayload::handle_comm_error()
{
    _consecutive_errors++;
    perf_count(_comms_errors);
    PX4_DEBUG("Communication error %d/%d", _consecutive_errors, MAX_CONSECUTIVE_ERRORS);
    if (_consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
        PX4_WARN("Too many consecutive errors, marking disconnected");
        _connection_state = ConnectionState::DISCONNECTED;
        _winch_status.comm_link_status = 0;
        _winch_status.timestamp = hrt_absolute_time();
        _winch_status_pub.publish(_winch_status);
    }
}

void WinchPayload::check_connection_health()
{
    hrt_abstime now = hrt_absolute_time();
    switch (_connection_state) {
    case ConnectionState::CONNECTED:
        // Check for communication timeout
        if (now - _last_successful_comm > COMM_LOSS_TIMEOUT_US) {
            PX4_WARN("Communication timeout, connection lost");
            _connection_state = ConnectionState::DISCONNECTED;
            _winch_status.comm_link_status = 0;
            _winch_status.timestamp = now;
            _winch_status_pub.publish(_winch_status);
        }
        break;
    case ConnectionState::DISCONNECTED:
        // Attempt reconnection periodically
        if (now - _last_reconnect_attempt > (RECONNECT_DELAY_MS * 1000ULL)) {
            _last_reconnect_attempt = now;
            PX4_INFO("Attempting reconnection...");
            if (attempt_connection()) {
                PX4_INFO("Reconnection successful");
            } else {
                PX4_WARN("Reconnection failed, will retry");
            }
        }
        break;
    case ConnectionState::RECONNECTING:
        // Handled in attempt_connection()
        break;
    }
}

void WinchPayload::process_incoming_data()
{
    if (_serial_fd < 0 || _connection_state != ConnectionState::CONNECTED) {
        return;
    }
    // Read available data
    uint8_t buffer[RX_BUFFER_SIZE];
    int bytes_read = read(_serial_fd, buffer, sizeof(buffer));
    if (bytes_read <= 0) {
        return;
    }
    // Feed bytes to the member parser (for fixed frame mode)
    for (int i = 0; i < bytes_read; i++) {
        if (feed_parser_byte(buffer[i], _parse_buffer, &_parse_pos, &_parse_state,
                             &_expected_data_len, &_data_received)) {
            // Frame complete, process it (assuming status response in fixed mode)
            if (parse_status_response(_parse_buffer, _parse_pos) == 0) {
                // Success, reset parser
                _parse_state = ParseState::WAIT_HEADER;
                _parse_pos = 0;
            } else {
                // Parse failed, reset
                _parse_state = ParseState::WAIT_HEADER;
                _parse_pos = 0;
            }
        }
    }
}

void WinchPayload::process_winch_control()
{
    winch_control_s control;
    if (_winch_control_sub.update(&control)) {
        if (_connection_state != ConnectionState::CONNECTED) {
            PX4_WARN("Cannot send command, winch disconnected");
            return;
        }
        PX4_INFO("Received winch command: %d", control.command);
        int result = 0;
        switch (control.command) {
        case 0: // Stop
            result = send_stop_command();
            break;
        case 1: // Descent
            result = send_descent_command();
            break;
        case 2: // Ascent
            result = send_ascent_command();
            break;
        case 3: // Emergency rope cut
            result = send_rope_cut_command();
            break;
        default:
            PX4_WARN("Unknown command: %d", control.command);
            break;
        }
        if (result < 0) {
            PX4_ERR("Command %d failed", control.command);
        }
        // Handle hook control
        if (control.release_hook == 1) {
            send_hook_open_command();
        } else if (control.release_hook == 0) {
            send_hook_close_command();
        }
    }
}

int WinchPayload::send_stop_command()
{
    return send_command_with_retry(REG_CMD_STOP, 0xFF, 3);
}

int WinchPayload::send_descent_command()
{
    return send_command_with_retry(REG_CMD_DESCENT, 0xFF, 3);
}

int WinchPayload::send_ascent_command()
{
    return send_command_with_retry(REG_CMD_ASCENT, 0xFF, 3);
}

int WinchPayload::send_hook_open_command()
{
    return send_command_with_retry(REG_CMD_HOOK_OPEN, 0xFF, 3);
}

int WinchPayload::send_hook_close_command()
{
    return send_command_with_retry(REG_CMD_HOOK_CLOSE, 0xFF, 3);
}

int WinchPayload::send_rope_cut_command()
{
    PX4_WARN("!!! EMERGENCY ROPE CUT COMMAND !!!");
    // Critical command - use more retries
    return send_command_with_retry(REG_CMD_ROPE_CUT, 0xFF, 5);
}

int WinchPayload::enable_fixed_frame_mode(bool enable)
{
    return send_command_with_retry(REG_CMD_FIXED_FRAME_MODE, enable ? 0x01 : 0x00, 3);
}

int WinchPayload::request_status()
{
    uint8_t frame[TX_BUFFER_SIZE];
    uint16_t frame_len;
    if (build_read_frame(REG_STATUS_START, REG_STATUS_COUNT, frame, &frame_len) < 0) {
        return -1;
    }
    return send_frame(frame, frame_len);
}

int WinchPayload::build_write_frame(uint16_t reg_addr, uint8_t value,
                                    uint8_t *frame, uint16_t *frame_len)
{
    uint8_t raw_frame[16]; // Small fixed size for write
    uint16_t pos = 0;
    raw_frame[pos++] = PAYLOAD_FRAME_HEADER;
    raw_frame[pos++] = PAYLOAD_FUNC_WRITE_SINGLE;
    raw_frame[pos++] = _device_address;
    raw_frame[pos++] = (reg_addr >> 8) & 0xFF;
    raw_frame[pos++] = reg_addr & 0xFF;
    raw_frame[pos++] = value;
    uint16_t crc = calculate_crc16(&raw_frame[0], pos);
    raw_frame[pos++] = crc & 0xFF;
    raw_frame[pos++] = (crc >> 8) & 0xFF;
    return apply_byte_stuffing(raw_frame, pos, frame, frame_len);
}

int WinchPayload::build_read_frame(uint16_t start_addr, uint16_t count,
                                   uint8_t *frame, uint16_t *frame_len)
{
    uint8_t raw_frame[16]; // Small fixed size for read
    uint16_t pos = 0;
    raw_frame[pos++] = PAYLOAD_FRAME_HEADER;
    raw_frame[pos++] = PAYLOAD_FUNC_READ_REQUEST;
    raw_frame[pos++] = _device_address;
    raw_frame[pos++] = (start_addr >> 8) & 0xFF;
    raw_frame[pos++] = start_addr & 0xFF;
    raw_frame[pos++] = (count >> 8) & 0xFF;
    raw_frame[pos++] = count & 0xFF;
    uint16_t crc = calculate_crc16(&raw_frame[0], pos);
    raw_frame[pos++] = crc & 0xFF;
    raw_frame[pos++] = (crc >> 8) & 0xFF;
    return apply_byte_stuffing(raw_frame, pos, frame, frame_len);
}

void WinchPayload::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }
    perf_begin(_loop_perf);
    // Check connection health and attempt reconnection if needed
    check_connection_health();
    // Process incoming winch control commands
    process_winch_control();
    // Process incoming data from winch (for fixed mode)
    process_incoming_data();
    // In polling mode, request status periodically
    if (!_fixed_frame_mode && _connection_state == ConnectionState::CONNECTED) {
        hrt_abstime now = hrt_absolute_time();
        if (now - _last_command_time > 100000ULL) { // 100ms
            request_status();
        }
    }
    perf_end(_loop_perf);
}

int WinchPayload::print_status()
{
    PX4_INFO("=== Payload Winch Driver Status ===");
    PX4_INFO("Serial port: %s (fd=%d)", _port, _serial_fd);
    PX4_INFO("Device address: 0x%02X", _device_address);
    PX4_INFO("Connection: %s",
             _connection_state == ConnectionState::CONNECTED ? "CONNECTED" :
             _connection_state == ConnectionState::DISCONNECTED ? "DISCONNECTED" : "RECONNECTING");
    PX4_INFO("Fixed frame mode: %s", _fixed_frame_mode ? "enabled" : "disabled");
    PX4_INFO("Last status: %.2f s ago",
             (double)(hrt_absolute_time() - _last_status_time) / 1e6);
    PX4_INFO("Consecutive errors: %d/%d", _consecutive_errors, MAX_CONSECUTIVE_ERRORS);
    PX4_INFO("CRC errors: %d", _crc_error_count);
    PX4_INFO("");
    PX4_INFO("--- Winch Status ---");
    PX4_INFO("State: %d (%s)", _winch_status.winch_state,
             get_winch_state_name(_winch_status.winch_state));
    PX4_INFO("Motor: %d (%s)", _winch_status.motor_state,
             get_motor_state_name(_winch_status.motor_state));
    PX4_INFO("Hook: %d (%s)", _winch_status.hook_state,
             get_hook_state_name(_winch_status.hook_state));
    PX4_INFO("Rope length: %.2f m", (double)_winch_status.rope_length * 0.01);
    PX4_INFO("Load weight: %.1f kg", (double)_winch_status.load_weight * 0.1);
    PX4_INFO("Release voltage: %.2f V", (double)_winch_status.release_module_voltage * 0.01);
    PX4_INFO("Comm status: %d", _winch_status.comm_link_status);
    PX4_INFO("System healthy: %d", _winch_status.system_healthy);
    PX4_INFO("");
    perf_print_counter(_loop_perf);
    perf_print_counter(_comms_errors);
    perf_print_counter(_crc_errors);
    perf_print_counter(_timeout_errors);
    perf_print_counter(_reconnect_count);
    return 0;
}

const char *WinchPayload::get_winch_state_name(uint8_t state)
{
    static const char *names[] = {
        "Hovering", // 0
        "Fuse Out", // 1
        "Upper Limit", // 2
        "Bottom Stop", // 3
        "Ascending", // 4
        "Descending", // 5
        "Unknown", // 6
        "Unknown", // 7
        "Light Load", // 8
        "Lower Limit", // 9
        "Hover Overcurrent", // 10
        "Rise Overcurrent", // 11
        "Entangled", // 12
        "Motor Fault" // 13
    };
    if (state < sizeof(names) / sizeof(names[0])) {
        return names[state];
    }
    return "Unknown";
}

const char *WinchPayload::get_motor_state_name(uint8_t state)
{
    switch (state) {
    case 0: return "Stopped";
    case 1: return "Reverse";
    case 2: return "Forward";
    case 8: return "Overload";
    case 10: return "Low Voltage";
    case 11: return "Over Temperature";
    case 12: return "Not Connected";
    case 13: return "Fault";
    default: return "Unknown";
    }
}

const char *WinchPayload::get_hook_state_name(uint8_t state)
{
    switch (state) {
    case 0: return "Closed";
    case 1: return "Open";
    case 2: return "Not Connected";
    case 3: return "Low Voltage";
    default: return "Unknown";
    }
}

// ... rest of task_spawn, custom_command, print_usage remain the same (assuming they are unchanged from original)
