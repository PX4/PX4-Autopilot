"""
Tests for all parse_* functions in fc_doc_generator.py.
"""
import fc_doc_generator as fcdg


# ---------------------------------------------------------------------------
# get_chip_family
# ---------------------------------------------------------------------------

class TestGetChipFamily:
    def test_stm32h7(self, board_stm32h7_all_dshot):
        assert fcdg.get_chip_family(board_stm32h7_all_dshot) == "stm32h7"

    def test_stm32h7_io(self, board_stm32h7_mixed_io):
        assert fcdg.get_chip_family(board_stm32h7_mixed_io) == "stm32h7"

    def test_stm32f4(self, board_stm32f4):
        assert fcdg.get_chip_family(board_stm32f4) == "stm32f4"

    def test_imxrt(self, board_imxrt):
        assert fcdg.get_chip_family(board_imxrt) == "imxrt"

    def test_missing_defconfig(self, tmp_path):
        assert fcdg.get_chip_family(tmp_path) == "unknown"

    def test_ppm_shared(self, board_ppm_shared):
        assert fcdg.get_chip_family(board_ppm_shared) == "stm32h7"


# ---------------------------------------------------------------------------
# parse_board_config
# ---------------------------------------------------------------------------

class TestParseBoardConfig:
    def test_8ch_no_io(self, board_stm32h7_all_dshot):
        result = fcdg.parse_board_config(board_stm32h7_all_dshot)
        assert result["total_outputs"] == 8
        assert result["has_io_board"] is False
        assert result["io_outputs"] == 0

    def test_9ch_with_io(self, board_stm32h7_mixed_io):
        result = fcdg.parse_board_config(board_stm32h7_mixed_io)
        assert result["total_outputs"] == 9
        assert result["has_io_board"] is True
        assert result["io_outputs"] == 8

    def test_6ch_no_io(self, board_stm32f4):
        result = fcdg.parse_board_config(board_stm32f4)
        assert result["total_outputs"] == 6
        assert result["has_io_board"] is False

    def test_4ch_ppm(self, board_ppm_shared):
        result = fcdg.parse_board_config(board_ppm_shared)
        assert result["total_outputs"] == 4

    def test_missing_file(self, tmp_path):
        assert fcdg.parse_board_config(tmp_path) == {}

    def test_imxrt_8ch(self, board_imxrt):
        result = fcdg.parse_board_config(board_imxrt)
        assert result["total_outputs"] == 8
        assert result["has_io_board"] is False


# ---------------------------------------------------------------------------
# parse_timer_config
# ---------------------------------------------------------------------------

class TestParseTimerConfig:
    def test_stm32h7_timer_count(self, board_stm32h7_all_dshot):
        result = fcdg.parse_timer_config(board_stm32h7_all_dshot)
        assert len(result["timers"]) == 4

    def test_stm32h7_all_timers_dshot(self, board_stm32h7_all_dshot):
        result = fcdg.parse_timer_config(board_stm32h7_all_dshot)
        assert all(t["dshot"] for t in result["timers"])

    def test_stm32h7_channel_count(self, board_stm32h7_all_dshot):
        result = fcdg.parse_timer_config(board_stm32h7_all_dshot)
        assert len(result["channels"]) == 8

    def test_stm32h7_channel_output_indices(self, board_stm32h7_all_dshot):
        result = fcdg.parse_timer_config(board_stm32h7_all_dshot)
        indices = [ch["output_index"] for ch in result["channels"]]
        # output_index is 1-based and sequential
        assert indices == list(range(1, len(indices) + 1))

    def test_stm32h7_mixed_dma(self, board_stm32h7_mixed_io):
        result = fcdg.parse_timer_config(board_stm32h7_mixed_io)
        timer_map = {t["timer"]: t["dshot"] for t in result["timers"]}
        assert timer_map["Timer5"] is True
        assert timer_map["Timer4"] is True
        assert timer_map["Timer12"] is False

    def test_stm32h7_mixed_channel_count(self, board_stm32h7_mixed_io):
        result = fcdg.parse_timer_config(board_stm32h7_mixed_io)
        assert len(result["channels"]) == 9

    def test_imxrt_derives_timers_from_channels(self, board_imxrt):
        result = fcdg.parse_timer_config(board_imxrt)
        # iMXRT: io_timers[] is empty, timers are derived from PWM prefix in channels
        timer_names = {t["timer"] for t in result["timers"]}
        assert "PWM2" in timer_names
        assert "PWM4" in timer_names

    def test_imxrt_all_channels_dshot(self, board_imxrt):
        result = fcdg.parse_timer_config(board_imxrt)
        assert all(ch["is_dshot_channel"] for ch in result["channels"])

    def test_imxrt_channel_count(self, board_imxrt):
        result = fcdg.parse_timer_config(board_imxrt)
        assert len(result["channels"]) == 8

    def test_capture_channels_excluded(self, board_stm32h7_capture_channels):
        # initIOTimerChannelCapture entries are input-capture pins, not PWM outputs.
        # Only the 8 regular initIOTimerChannel entries (Timer5/Timer4) should appear.
        result = fcdg.parse_timer_config(board_stm32h7_capture_channels)
        assert len(result["channels"]) == 8
        # output indices must be 1–8 (no gap from skipped capture entries)
        indices = [ch["output_index"] for ch in result["channels"]]
        assert indices == list(range(1, 9))
        # No channel should reference Timer1/Timer8/Timer12 (capture-only timers)
        timers_used = {ch["timer"] for ch in result["channels"]}
        assert timers_used.isdisjoint({"Timer1", "Timer8", "Timer12"})

    def test_stm32f4_no_dshot(self, board_stm32f4):
        result = fcdg.parse_timer_config(board_stm32f4)
        assert all(not t["dshot"] for t in result["timers"])
        assert len(result["timers"]) == 3

    def test_stm32f4_channel_count(self, board_stm32f4):
        result = fcdg.parse_timer_config(board_stm32f4)
        assert len(result["channels"]) == 6

    def test_missing_file(self, tmp_path):
        assert fcdg.parse_timer_config(tmp_path) == {}


class TestParseTimerConfigCaptureChannels:
    """parse_timer_config() must return capture_channels for initIOTimerChannelCapture
    entries, and those entries must not appear in channels."""

    def test_capture_channels_returned(self, board_stm32h7_capture_channels):
        result = fcdg.parse_timer_config(board_stm32h7_capture_channels)
        assert "capture_channels" in result
        assert len(result["capture_channels"]) == 8

    def test_capture_channels_timers(self, board_stm32h7_capture_channels):
        result = fcdg.parse_timer_config(board_stm32h7_capture_channels)
        cap_timers = {c["timer"] for c in result["capture_channels"]}
        assert cap_timers == {"Timer1", "Timer8", "Timer12"}

    def test_capture_channels_not_in_regular_channels(self, board_stm32h7_capture_channels):
        result = fcdg.parse_timer_config(board_stm32h7_capture_channels)
        reg_pairs = {(c["timer"], c["channel"]) for c in result["channels"]}
        cap_pairs = {(c["timer"], c["channel"]) for c in result["capture_channels"]}
        assert reg_pairs.isdisjoint(cap_pairs)

    def test_f7_capture_channel_stored(self, board_stm32f7_no_dshot):
        # stm32f7_no_dshot has one initIOTimerChannelCapture on Timer5/Channel4
        result = fcdg.parse_timer_config(board_stm32f7_no_dshot)
        caps = result.get("capture_channels", [])
        assert len(caps) == 1
        assert caps[0]["timer"] == "Timer5"
        assert caps[0]["channel"] == "Channel4"
        assert caps[0]["channel_index"] == 3  # Channel4 → index 3

    def test_no_capture_channels_when_none_present(self, board_stm32h7_all_dshot):
        # stm32h7_all_dshot has no initIOTimerChannelCapture entries
        result = fcdg.parse_timer_config(board_stm32h7_all_dshot)
        assert result.get("capture_channels", []) == []


# ---------------------------------------------------------------------------
# parse_serial_config
# ---------------------------------------------------------------------------

class TestParseSerialConfig:
    def test_stm32h7_port_count(self, board_stm32h7_all_dshot):
        result = fcdg.parse_serial_config(board_stm32h7_all_dshot)
        # USART1, USART2, USART3, UART4, USART6, UART7 = 6 active
        assert len(result["serial_ports"]) == 6

    def test_stm32h7_labels(self, board_stm32h7_all_dshot):
        result = fcdg.parse_serial_config(board_stm32h7_all_dshot)
        label_map = {p["device"]: p["label"] for p in result["serial_ports"]}
        assert label_map["/dev/ttyS0"] == "TELEM1"
        assert label_map["/dev/ttyS1"] == "TELEM2"
        assert label_map["/dev/ttyS2"] == "Debug Console"  # USART3 is console, no BOARD_SERIAL label
        assert label_map["/dev/ttyS3"] == "GPS1"
        assert label_map["/dev/ttyS4"] == "RC"

    def test_stm32h7_uart_order(self, board_stm32h7_all_dshot):
        result = fcdg.parse_serial_config(board_stm32h7_all_dshot)
        uarts = [p["uart"] for p in result["serial_ports"]]
        # Must follow stm32h7 UART_HW_ORDER
        assert uarts[0] == "USART1"
        assert uarts[2] == "USART3"

    def test_io_board_px4io_rc_combined_label(self, board_stm32h7_mixed_io):
        result = fcdg.parse_serial_config(board_stm32h7_mixed_io)
        # PX4IO_SERIAL_DEVICE="/dev/ttyS5" and CONFIG_BOARD_SERIAL_RC="/dev/ttyS5"
        # → should be labelled "PX4IO/RC"
        label_map = {p["device"]: p["label"] for p in result["serial_ports"]}
        assert label_map["/dev/ttyS5"] == "PX4IO/RC"

    def test_tel_normalized_to_telem(self, board_stm32h7_all_dshot):
        result = fcdg.parse_serial_config(board_stm32h7_all_dshot)
        labels = [p["label"] for p in result["serial_ports"]]
        assert "TELEM1" in labels
        assert "TELEM2" in labels
        # Raw "TEL1" / "TEL2" must not appear
        assert "TEL1" not in labels
        assert "TEL2" not in labels

    def test_imxrt_lpuart_order(self, board_imxrt):
        result = fcdg.parse_serial_config(board_imxrt)
        uarts = [p["uart"] for p in result["serial_ports"]]
        # LPUART2 comes before LPUART3 etc. (LPUART1 not enabled → skipped)
        assert uarts[0] == "LPUART2"
        assert "LPUART1" not in uarts

    def test_unknown_family_returns_empty(self, tmp_path):
        # Board with no defconfig → unknown family → no ports
        result = fcdg.parse_serial_config(tmp_path)
        assert result == {"serial_ports": []}

    def test_flow_control_detected_from_board_h(self, board_stm32h7_all_dshot):
        result = fcdg.parse_serial_config(board_stm32h7_all_dshot)
        fc_map = {p["uart"]: p["flow_control"] for p in result["serial_ports"]}
        # USART1 has both #define GPIO_USART1_RTS and _CTS → True
        assert fc_map["USART1"] is True
        # USART2 has only #define GPIO_USART2_RTS (no CTS) → False
        assert fc_map["USART2"] is False
        # UART4 has GPIO_UART4_RTS in a comment only, not a #define → False
        assert fc_map["UART4"] is False

    def test_flow_control_false_when_no_board_h(self, tmp_path):
        # Board with defconfig but no board.h → all flow_control=False
        defconfig_dir = tmp_path / "nuttx-config" / "nsh"
        defconfig_dir.mkdir(parents=True)
        (defconfig_dir / "defconfig").write_text(
            "CONFIG_ARCH_CHIP_STM32H7=y\nCONFIG_STM32H7_USART1=y\n"
        )
        result = fcdg.parse_serial_config(tmp_path)
        for p in result["serial_ports"]:
            assert p["flow_control"] is False


# ---------------------------------------------------------------------------
# parse_rc_config
# ---------------------------------------------------------------------------

class TestParseRcConfig:
    def test_has_rc_input(self, board_stm32h7_all_dshot):
        result = fcdg.parse_rc_config(board_stm32h7_all_dshot)
        assert result["has_rc_input"] is True
        assert result["has_common_rc"] is False

    def test_rc_serial_device(self, board_stm32h7_all_dshot):
        result = fcdg.parse_rc_config(board_stm32h7_all_dshot)
        assert result["rc_serial_device"] == "/dev/ttyS4"

    def test_no_ppm_pin(self, board_stm32h7_all_dshot):
        result = fcdg.parse_rc_config(board_stm32h7_all_dshot)
        assert result["has_ppm_pin"] is False

    def test_has_common_rc(self, board_stm32h7_mixed_io):
        result = fcdg.parse_rc_config(board_stm32h7_mixed_io)
        assert result["has_common_rc"] is True
        assert result["has_rc_input"] is False

    def test_ppm_pin_present_mixed_io(self, board_stm32h7_mixed_io):
        result = fcdg.parse_rc_config(board_stm32h7_mixed_io)
        assert result["has_ppm_pin"] is True

    def test_ppm_shared_with_rc_serial(self, board_ppm_shared):
        result = fcdg.parse_rc_config(board_ppm_shared)
        assert result["has_ppm_pin"] is True
        assert result["ppm_shared_with_rc_serial"] is True

    def test_imxrt_common_rc(self, board_imxrt):
        result = fcdg.parse_rc_config(board_imxrt)
        assert result["has_common_rc"] is True

    def test_no_rc_data(self, tmp_path):
        result = fcdg.parse_rc_config(tmp_path)
        assert result["has_rc_input"] is False
        assert result["has_common_rc"] is False
        assert result["has_ppm_pin"] is False


# ---------------------------------------------------------------------------
# parse_gps_config
# ---------------------------------------------------------------------------

class TestParseGpsConfig:
    def test_imxrt_pps_capture(self, board_imxrt):
        result = fcdg.parse_gps_config(board_imxrt)
        assert result["has_pps_capture"] is True

    def test_imxrt_safety_switch_and_led(self, board_imxrt):
        result = fcdg.parse_gps_config(board_imxrt)
        assert result["has_safety_switch"] is True
        assert result["has_safety_led"] is True

    def test_imxrt_has_buzzer(self, board_imxrt):
        result = fcdg.parse_gps_config(board_imxrt)
        assert result["has_buzzer"] is True

    def test_no_safety_stm32h7(self, board_stm32h7_all_dshot):
        result = fcdg.parse_gps_config(board_stm32h7_all_dshot)
        assert result["has_safety_switch"] is False
        assert result["has_safety_led"] is False
        assert result["has_pps_capture"] is False
        assert result["has_buzzer"] is False

    def test_no_gps_data(self, tmp_path):
        result = fcdg.parse_gps_config(tmp_path)
        assert result["has_pps_capture"] is False
        assert result["has_safety_switch"] is False
        assert result["has_buzzer"] is False


# ---------------------------------------------------------------------------
# parse_power_config
# ---------------------------------------------------------------------------

class TestParsePowerConfig:
    def test_single_brick_stm32h7(self, board_stm32h7_all_dshot):
        result = fcdg.parse_power_config(board_stm32h7_all_dshot)
        assert result["num_power_inputs"] == 1
        assert result["has_redundant_power"] is False
        assert result["has_dual_battery_monitoring"] is False
        assert result["power_monitor_type"] == "analog"

    def test_dual_brick_stm32h7_mixed_io(self, board_stm32h7_mixed_io):
        result = fcdg.parse_power_config(board_stm32h7_mixed_io)
        assert result["num_power_inputs"] == 2
        assert result["has_redundant_power"] is True
        assert result["has_dual_battery_monitoring"] is True
        assert result["power_monitor_type"] == "ltc44xx"

    def test_digital_brick_imxrt(self, board_imxrt):
        result = fcdg.parse_power_config(board_imxrt)
        assert result["num_power_inputs"] == 1
        assert result["has_redundant_power"] is False
        # px4board INA238 driver takes precedence
        assert result["power_monitor_type"] == "ina238"

    def test_analog_monitor_stm32f4(self, board_stm32f4):
        result = fcdg.parse_power_config(board_stm32f4)
        assert result["num_power_inputs"] == 1
        assert result["power_monitor_type"] == "analog"

    def test_missing_board_config_returns_defaults(self, tmp_path):
        result = fcdg.parse_power_config(tmp_path)
        assert result["num_power_inputs"] == 1
        assert result["has_redundant_power"] is False
        assert result["has_dual_battery_monitoring"] is False
        assert result["power_monitor_type"] is None

    def test_dronecan_pattern_a(self, board_stm32h7_mixed_io):
        # Pattern A: GPIO_nPOWER_IN_CAN present in board_config.h
        result = fcdg.parse_power_config(board_stm32h7_mixed_io)
        assert result["has_dronecan_power_input"] is True

    def test_dronecan_pattern_b(self, board_imxrt):
        # Pattern B: BOARD_NUMBER_DIGITAL_BRICKS + CONFIG_DRIVERS_UAVCAN=y
        result = fcdg.parse_power_config(board_imxrt)
        assert result["has_dronecan_power_input"] is True

    def test_no_dronecan_stm32f4(self, board_stm32f4):
        result = fcdg.parse_power_config(board_stm32f4)
        assert result["has_dronecan_power_input"] is False

    def test_no_dronecan_stm32h7_single(self, board_stm32h7_all_dshot):
        result = fcdg.parse_power_config(board_stm32h7_all_dshot)
        assert result["has_dronecan_power_input"] is False


# ---------------------------------------------------------------------------
# parse_sd_card_config
# ---------------------------------------------------------------------------

class TestParseSDCardConfig:
    def test_has_sd_stm32h7(self, board_stm32h7_all_dshot):
        # defconfig has CONFIG_MMCSD_SDIO=y
        result = fcdg.parse_sd_card_config(board_stm32h7_all_dshot)
        assert result["has_sd_card"] is True

    def test_has_sd_mixed_io(self, board_stm32h7_mixed_io):
        result = fcdg.parse_sd_card_config(board_stm32h7_mixed_io)
        assert result["has_sd_card"] is True

    def test_has_sd_imxrt(self, board_imxrt):
        result = fcdg.parse_sd_card_config(board_imxrt)
        assert result["has_sd_card"] is True

    def test_no_sd_stm32f4(self, board_stm32f4):
        result = fcdg.parse_sd_card_config(board_stm32f4)
        assert result["has_sd_card"] is False

    def test_no_sd_ppm_shared(self, board_ppm_shared):
        result = fcdg.parse_sd_card_config(board_ppm_shared)
        assert result["has_sd_card"] is False

    def test_secondary_sd_bench(self, tmp_path):
        # No defconfig, but default.px4board has SD bench tool
        (tmp_path / 'default.px4board').write_text(
            'CONFIG_SYSTEMCMDS_SD_BENCH=y\n'
        )
        result = fcdg.parse_sd_card_config(tmp_path)
        assert result["has_sd_card"] is True

    def test_secondary_sd_stress(self, tmp_path):
        (tmp_path / 'default.px4board').write_text(
            'CONFIG_SYSTEMCMDS_SD_STRESS=y\n'
        )
        result = fcdg.parse_sd_card_config(tmp_path)
        assert result["has_sd_card"] is True

    def test_missing_returns_false(self, tmp_path):
        result = fcdg.parse_sd_card_config(tmp_path)
        assert result["has_sd_card"] is False


# ---------------------------------------------------------------------------
# parse_rc_board_sensors
# ---------------------------------------------------------------------------

class TestParseRcBoardSensors:
    def test_basic_sensor_detection(self, board_stm32h7_all_dshot):
        result = fcdg.parse_rc_board_sensors(board_stm32h7_all_dshot)
        assert result['imu'] == ['ICM-42688P']
        assert result['baro'] == ['MS5611']
        assert result['mag'] == ['IST8310']
        assert result['osd'] == []

    def test_deduplicates_repeated_driver(self, board_stm32h7_all_dshot):
        """ist8310 started twice (internal + external) → appears exactly once."""
        result = fcdg.parse_rc_board_sensors(board_stm32h7_all_dshot)
        assert result['mag'].count('IST8310') == 1

    def test_ignores_non_sensor_commands(self, board_stm32h7_all_dshot):
        """board_adc and other non-sensor commands are silently ignored."""
        result = fcdg.parse_rc_board_sensors(board_stm32h7_all_dshot)
        flat = str(result)
        assert 'board_adc' not in flat
        assert 'start' not in flat

    def test_missing_file_returns_empty(self, tmp_path):
        """No rc.board_sensors → empty result, no error."""
        result = fcdg.parse_rc_board_sensors(tmp_path)
        assert result == {'imu': [], 'baro': [], 'mag': [], 'osd': []}


# ---------------------------------------------------------------------------
# parse_rc_board_sensor_bus
# ---------------------------------------------------------------------------

class TestParseRcBoardSensorBus:
    def test_spi_flag_detected(self, board_stm32h7_all_dshot):
        result = fcdg.parse_rc_board_sensor_bus(board_stm32h7_all_dshot)
        imu = result['imu']
        assert len(imu) == 1
        assert imu[0]['name'] == 'ICM-42688P'
        assert imu[0]['bus_type'] == 'SPI'
        assert imu[0]['bus_num'] is None
        assert imu[0]['external'] is False

    def test_i2c_internal_flag_detected(self, board_stm32h7_all_dshot):
        result = fcdg.parse_rc_board_sensor_bus(board_stm32h7_all_dshot)
        # ist8310 -I start → internal I2C
        internal = [e for e in result['mag'] if not e['external']]
        assert len(internal) == 1
        assert internal[0]['bus_type'] == 'I2C'
        assert internal[0]['bus_num'] is None
        assert internal[0]['external'] is False

    def test_external_flag_and_bus_num_detected(self, board_stm32h7_all_dshot):
        result = fcdg.parse_rc_board_sensor_bus(board_stm32h7_all_dshot)
        # ist8310 -X -b 1 start → external I2C bus 1
        external = [e for e in result['mag'] if e['external']]
        assert len(external) == 1
        assert external[0]['bus_type'] == 'I2C'
        assert external[0]['bus_num'] == 1
        assert external[0]['external'] is True

    def test_same_chip_different_bus_kept_separate(self, board_stm32h7_all_dshot):
        """IST8310 internal + external → 2 separate entries."""
        result = fcdg.parse_rc_board_sensor_bus(board_stm32h7_all_dshot)
        assert len(result['mag']) == 2

    def test_missing_file_returns_empty(self, tmp_path):
        result = fcdg.parse_rc_board_sensor_bus(tmp_path)
        assert result == {'imu': [], 'baro': [], 'mag': [], 'osd': [], 'power_monitor': []}

    def test_skips_variant_block(self, board_stm32h7_variant):
        """Sensors inside if ver hwtypecmp blocks must not appear in sensor_bus_info."""
        result = fcdg.parse_rc_board_sensor_bus(board_stm32h7_variant)
        imu_names = [e['name'] for e in result['imu']]
        # bmi088 and icm20602 are inside variant blocks → must be absent
        assert 'BMI088' not in imu_names
        assert 'ICM-20602' not in imu_names
        # icm42688p is unconditional → must be present
        assert 'ICM-42688P' in imu_names

    def test_external_port_label_detected_from_comment(self, board_stm32h7_all_dshot):
        """External sensor preceded by 'GPS1/I2C1' comment gets port_label='GPS1'."""
        result = fcdg.parse_rc_board_sensor_bus(board_stm32h7_all_dshot)
        external = [e for e in result['mag'] if e['external']]
        assert len(external) == 1
        assert external[0]['port_label'] == 'GPS1'

    def test_internal_sensor_port_label_none(self, board_stm32h7_all_dshot):
        """Internal sensor preceded by 'Internal compass' comment gets port_label=None."""
        result = fcdg.parse_rc_board_sensor_bus(board_stm32h7_all_dshot)
        internal = [e for e in result['mag'] if not e['external']]
        assert len(internal) == 1
        assert internal[0]['port_label'] is None

    def test_spi_sensor_port_label_none(self, board_stm32h7_all_dshot):
        """SPI sensor with no GPS/I2C comment gets port_label=None."""
        result = fcdg.parse_rc_board_sensor_bus(board_stm32h7_all_dshot)
        assert result['imu'][0]['port_label'] is None


# ---------------------------------------------------------------------------
# parse_i2c_bus_config
# ---------------------------------------------------------------------------

class TestParseI2cBusConfig:
    def test_missing_file_returns_empty_dict(self, tmp_path):
        """No src/i2c.cpp → empty dict."""
        assert fcdg.parse_i2c_bus_config(tmp_path) == {}

    def test_external_buses_parsed(self, tmp_path):
        (tmp_path / 'src').mkdir(parents=True)
        (tmp_path / 'src' / 'i2c.cpp').write_text(
            'constexpr px4_i2c_bus_t buses[] = {\n'
            '    initI2CBusExternal(1),\n'
            '    initI2CBusExternal(2),\n'
            '};\n'
        )
        result = fcdg.parse_i2c_bus_config(tmp_path)
        assert result['external'] == [1, 2]
        assert result['internal'] == []

    def test_internal_buses_parsed(self, tmp_path):
        (tmp_path / 'src').mkdir(parents=True)
        (tmp_path / 'src' / 'i2c.cpp').write_text(
            'constexpr px4_i2c_bus_t buses[] = {\n'
            '    initI2CBusInternal(3),\n'
            '};\n'
        )
        result = fcdg.parse_i2c_bus_config(tmp_path)
        assert result['external'] == []
        assert result['internal'] == [3]

    def test_mixed_buses_parsed_and_sorted(self, tmp_path):
        (tmp_path / 'src').mkdir(parents=True)
        (tmp_path / 'src' / 'i2c.cpp').write_text(
            'constexpr px4_i2c_bus_t buses[] = {\n'
            '    initI2CBusExternal(1),\n'
            '    initI2CBusExternal(2),\n'
            '    initI2CBusInternal(3),\n'
            '    initI2CBusExternal(4),\n'
            '};\n'
        )
        result = fcdg.parse_i2c_bus_config(tmp_path)
        assert result['external'] == [1, 2, 4]
        assert result['internal'] == [3]

    def test_whitespace_in_args(self, tmp_path):
        (tmp_path / 'src').mkdir(parents=True)
        (tmp_path / 'src' / 'i2c.cpp').write_text(
            'initI2CBusExternal( 1 );\ninitI2CBusInternal( 2 );\n'
        )
        result = fcdg.parse_i2c_bus_config(tmp_path)
        assert 1 in result['external']
        assert 2 in result['internal']

    def test_empty_file_returns_empty_dict(self, tmp_path):
        (tmp_path / 'src').mkdir(parents=True)
        (tmp_path / 'src' / 'i2c.cpp').write_text('// no bus entries\n')
        assert fcdg.parse_i2c_bus_config(tmp_path) == {}


# ---------------------------------------------------------------------------
# power_monitor category in parse_rc_board_sensor_bus
# ---------------------------------------------------------------------------

class TestPowerMonitorSensorBus:
    def test_ina226_classified_as_power_monitor(self, tmp_path):
        (tmp_path / 'init').mkdir(parents=True)
        (tmp_path / 'init' / 'rc.board_sensors').write_text(
            'ina226 -X -b 2 -t 1 -k start\n'
        )
        result = fcdg.parse_rc_board_sensor_bus(tmp_path)
        assert len(result['power_monitor']) == 1
        pm = result['power_monitor'][0]
        assert pm['name'] == 'INA226'
        assert pm['bus_type'] == 'I2C'
        assert pm['bus_num'] == 2
        assert pm['external'] is True

    def test_ina228_classified_as_power_monitor(self, tmp_path):
        (tmp_path / 'init').mkdir(parents=True)
        (tmp_path / 'init' / 'rc.board_sensors').write_text(
            'ina228 -X -b 1 start\n'
        )
        result = fcdg.parse_rc_board_sensor_bus(tmp_path)
        assert result['power_monitor'][0]['name'] == 'INA228'


# ---------------------------------------------------------------------------
# _extract_port_label_from_comment
# ---------------------------------------------------------------------------

class TestExtractPortLabelFromComment:
    def test_gps_port_extracted(self):
        assert fcdg._extract_port_label_from_comment(
            'External compass on GPS1/I2C1: standard puck') == 'GPS1'

    def test_gps2_extracted(self):
        assert fcdg._extract_port_label_from_comment('compass on GPS2') == 'GPS2'

    def test_telem_extracted_when_no_gps(self):
        assert fcdg._extract_port_label_from_comment('sensor on TELEM1') == 'TELEM1'

    def test_i2c_numbered_extracted_when_no_gps_or_telem(self):
        assert fcdg._extract_port_label_from_comment('I2C2 ist8310 magnetometer') == 'I2C2'

    def test_i2c_without_digit_returns_none(self):
        # "Internal magnetometer on I2C" — no digit suffix
        assert fcdg._extract_port_label_from_comment('Internal magnetometer on I2C') is None

    def test_none_input_returns_none(self):
        assert fcdg._extract_port_label_from_comment(None) is None

    def test_empty_comment_returns_none(self):
        assert fcdg._extract_port_label_from_comment('') is None

    def test_gps_takes_priority_over_i2c(self):
        # Comment mentions both GPS1 and I2C1 — GPS should win
        assert fcdg._extract_port_label_from_comment(
            'External compass on GPS1/I2C1 (the 3rd external bus)') == 'GPS1'


# ---------------------------------------------------------------------------
# parse_sensor_variant_blocks
# ---------------------------------------------------------------------------

class TestParseSensorVariantBlocks:
    def test_detects_variant_block(self, board_stm32h7_variant):
        result = fcdg.parse_sensor_variant_blocks(board_stm32h7_variant)
        assert result['has_variants'] is True

    def test_unconditional_sensors_separated(self, board_stm32h7_variant):
        result = fcdg.parse_sensor_variant_blocks(board_stm32h7_variant)
        uncond = result['unconditional']
        imu_names = [e['name'] for e in uncond['imu']]
        baro_names = [e['name'] for e in uncond['baro']]
        mag_names = [e['name'] for e in uncond['mag']]
        assert 'ICM-42688P' in imu_names
        assert 'MS5611' in baro_names
        assert 'IST8310' in mag_names
        # variant-only sensors must NOT be in unconditional
        assert 'BMI088' not in imu_names
        assert 'ICM-20602' not in imu_names

    def test_variant_code_sensors_captured(self, board_stm32h7_variant):
        result = fcdg.parse_sensor_variant_blocks(board_stm32h7_variant)
        hw1 = result['variants'].get('HW000001', {})
        imu_names = [e['name'] for e in hw1.get('imu', [])]
        assert 'BMI088' in imu_names

    def test_else_block_stored_as_other(self, board_stm32h7_variant):
        result = fcdg.parse_sensor_variant_blocks(board_stm32h7_variant)
        other = result['variants'].get('__other__', {})
        imu_names = [e['name'] for e in other.get('imu', [])]
        assert 'ICM-20602' in imu_names

    def test_no_variants_returns_has_variants_false(self, board_stm32h7_all_dshot):
        result = fcdg.parse_sensor_variant_blocks(board_stm32h7_all_dshot)
        assert result['has_variants'] is False
        assert result['variants'] == {}

    def test_no_variants_unconditional_has_all_sensors(self, board_stm32h7_all_dshot):
        result = fcdg.parse_sensor_variant_blocks(board_stm32h7_all_dshot)
        uncond = result['unconditional']
        assert len(uncond['imu']) == 1
        assert uncond['imu'][0]['name'] == 'ICM-42688P'

    def test_missing_file_returns_has_variants_false(self, tmp_path):
        result = fcdg.parse_sensor_variant_blocks(tmp_path)
        assert result['has_variants'] is False
        assert result['unconditional'] == {'imu': [], 'baro': [], 'mag': [], 'osd': [], 'power_monitor': []}
        assert result['variants'] == {}

    def test_graceful_fail_sensors_moved_to_variant(self, board_stm32h7_graceful_fail):
        """Sensors that appear unconditionally AND in a variant block must be removed from
        unconditional (PX4 graceful-fail pattern — driver start fails on hardware without chip)."""
        result = fcdg.parse_sensor_variant_blocks(board_stm32h7_graceful_fail)
        uncond_imu = [e['name'] for e in result['unconditional']['imu']]
        # BMI088 is in unconditional block AND VD000000 → must be moved to VD000000 only
        assert 'BMI088' not in uncond_imu
        # ICM-20689 is unconditional only → stays unconditional
        assert 'ICM-20689' in uncond_imu
        # ICM-20602 is VD000001 only → in variants, not unconditional
        assert 'ICM-20602' not in uncond_imu
        # Variants must still hold the sensor
        vd0_imu = [e['name'] for e in result['variants'].get('VD000000', {}).get('imu', [])]
        assert 'BMI088' in vd0_imu
        vd1_imu = [e['name'] for e in result['variants'].get('VD000001', {}).get('imu', [])]
        assert 'ICM-20602' in vd1_imu
