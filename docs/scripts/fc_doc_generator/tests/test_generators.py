"""
Snapshot tests for all section generator functions.

Run ``pytest --update-snapshots`` to regenerate snapshots after intentional
changes to generator output.
"""
import fc_doc_generator as fcdg

BOARD_KEY = "test/fixture"


# ---------------------------------------------------------------------------
# Shared entry builders
# ---------------------------------------------------------------------------

def _group(n, timer, outputs, dshot=True, bdshot_outputs=None, bdshot_output_only=None):
    dshot_outputs = outputs if dshot else []
    non_dshot = [] if dshot else outputs
    return {
        "group": n,
        "timer": timer,
        "outputs": outputs,
        "dshot": dshot,
        "dshot_outputs": dshot_outputs,
        "non_dshot_outputs": non_dshot,
        "bdshot": bool(bdshot_outputs),
        "bdshot_outputs": bdshot_outputs or [],
        "bdshot_output_only": bdshot_output_only or [],
    }


def _entry_stm32h7_all_dshot():
    return {
        "has_io_board": False, "total_outputs": 8, "io_outputs": 0,
        "serial_ports": [],
        "has_rc_input": False, "has_common_rc": False, "rc_serial_device": None,
        "has_ppm_pin": False, "ppm_shared_with_rc_serial": False,
        "has_pps_capture": False, "has_safety_switch": False, "has_safety_led": False,
        "has_buzzer": False,
        "num_power_inputs": 1, "has_redundant_power": False,
        "has_dual_battery_monitoring": False, "has_dronecan_power_input": False,
        "power_monitor_type": "analog", "has_sd_card": True,
        "rc_ports_wizard": None, "gps_ports_wizard": None, "power_ports_wizard": None,
        "groups": [
            _group(1, "Timer3", [1, 2], bdshot_outputs=[1, 2]),
            _group(2, "Timer2", [3, 4], bdshot_outputs=[3, 4]),
            _group(3, "Timer5", [5, 6], bdshot_outputs=[5, 6]),
            _group(4, "Timer8", [7, 8], bdshot_outputs=[7, 8]),
        ],
    }


def _entry_stm32h7_mixed_io():
    return {
        "has_io_board": True, "total_outputs": 9, "io_outputs": 8,
        "serial_ports": [
            {"uart": "USART1", "device": "/dev/ttyS0", "label": "GPS1",          "flow_control": False},
            {"uart": "USART2", "device": "/dev/ttyS1", "label": "TELEM3",        "flow_control": True},
            {"uart": "USART3", "device": "/dev/ttyS2", "label": "Debug Console", "flow_control": False},
            {"uart": "UART4",  "device": "/dev/ttyS3", "label": "GPS2",          "flow_control": True},
            {"uart": "UART5",  "device": "/dev/ttyS4", "label": "TELEM2",        "flow_control": False},
            {"uart": "USART6", "device": "/dev/ttyS5", "label": "PX4IO/RC",      "flow_control": False},
            {"uart": "UART7",  "device": "/dev/ttyS6", "label": "TELEM1",        "flow_control": False},
            {"uart": "UART8",  "device": "/dev/ttyS7", "label": "",              "flow_control": False},
        ],
        "has_rc_input": False, "has_common_rc": True, "rc_serial_device": "/dev/ttyS5",
        "has_ppm_pin": True, "ppm_shared_with_rc_serial": False,
        "has_pps_capture": False, "has_safety_switch": True, "has_safety_led": False,
        "has_buzzer": False,
        "num_power_inputs": 2, "has_redundant_power": True,
        "has_dual_battery_monitoring": True, "has_dronecan_power_input": True,
        "power_monitor_type": "ltc44xx", "has_sd_card": True,
        "rc_ports_wizard": None, "gps_ports_wizard": None, "power_ports_wizard": None,
        "groups": [
            _group(1, "Timer5", [1, 2, 3, 4], bdshot_outputs=[1, 2, 3, 4]),
            _group(2, "Timer4", [5, 6], bdshot_outputs=[5, 6]),
            _group(3, "Timer12", [7, 8], dshot=False),
            _group(4, "Timer1", [9], bdshot_outputs=[9]),
        ],
    }


def _entry_imxrt():
    return {
        "has_io_board": False, "total_outputs": 8, "io_outputs": 0,
        "serial_ports": [
            {"uart": "LPUART2", "device": "/dev/ttyS0", "label": "Debug Console", "flow_control": False},
            {"uart": "LPUART3", "device": "/dev/ttyS1", "label": "GPS1",          "flow_control": False},
            {"uart": "LPUART4", "device": "/dev/ttyS2", "label": "TELEM1",        "flow_control": False},
            {"uart": "LPUART5", "device": "/dev/ttyS3", "label": "TELEM2",        "flow_control": False},
            {"uart": "LPUART6", "device": "/dev/ttyS4", "label": "TELEM3",        "flow_control": False},
            {"uart": "LPUART7", "device": "/dev/ttyS5", "label": "RC",            "flow_control": False},
            {"uart": "LPUART8", "device": "/dev/ttyS6", "label": "",              "flow_control": False},
        ],
        "has_rc_input": False, "has_common_rc": True, "rc_serial_device": "/dev/ttyS5",
        "has_ppm_pin": False, "ppm_shared_with_rc_serial": False,
        "has_pps_capture": True, "has_safety_switch": True, "has_safety_led": True,
        "has_buzzer": True,
        "num_power_inputs": 1, "has_redundant_power": False,
        "has_dual_battery_monitoring": False, "has_dronecan_power_input": True,
        "power_monitor_type": "ina238", "has_sd_card": True,
        "rc_ports_wizard": None, "gps_ports_wizard": None, "power_ports_wizard": None,
        "groups": [
            _group(1, "PWM2", [1, 2, 3, 4], bdshot_outputs=[1, 2, 3, 4]),
            _group(2, "PWM4", [5, 6, 7, 8], bdshot_outputs=[5, 6, 7, 8]),
        ],
    }


def _entry_stm32h7_capture_channels():
    """8 outputs (Timer5/Timer4 only); Timer1/Timer8/Timer12 are capture-only and excluded."""
    return {
        "has_io_board": False, "total_outputs": 8, "io_outputs": 0,
        "serial_ports": [],
        "has_rc_input": False, "has_common_rc": False, "rc_serial_device": None,
        "has_ppm_pin": False, "ppm_shared_with_rc_serial": False,
        "has_pps_capture": False, "has_safety_switch": False, "has_safety_led": False,
        "has_buzzer": False,
        "num_power_inputs": 1, "has_redundant_power": False,
        "has_dual_battery_monitoring": False, "has_dronecan_power_input": False,
        "power_monitor_type": "analog", "has_sd_card": True,
        "rc_ports_wizard": None, "gps_ports_wizard": None, "power_ports_wizard": None,
        "groups": [
            _group(1, "Timer5", [1, 2, 3, 4], bdshot_outputs=[1, 2, 3, 4]),
            _group(2, "Timer4", [5, 6, 7, 8], bdshot_outputs=[5, 6, 7], bdshot_output_only=[8]),
        ],
    }


def _entry_stm32f4_no_dshot():
    return {
        "has_io_board": False, "total_outputs": 6, "io_outputs": 0,
        "serial_ports": [
            {"uart": "USART1", "device": "/dev/ttyS0", "label": "Debug Console", "flow_control": False},
            {"uart": "USART2", "device": "/dev/ttyS1", "label": "",               "flow_control": False},
            {"uart": "USART3", "device": "/dev/ttyS2", "label": "",               "flow_control": False},
            {"uart": "UART4",  "device": "/dev/ttyS3", "label": "GPS1",           "flow_control": False},
            {"uart": "UART5",  "device": "/dev/ttyS4", "label": "TELEM1",         "flow_control": False},
            {"uart": "USART6", "device": "/dev/ttyS5", "label": "RC",             "flow_control": False},
        ],
        "has_rc_input": True, "has_common_rc": False, "rc_serial_device": "/dev/ttyS5",
        "has_ppm_pin": False, "ppm_shared_with_rc_serial": False,
        "has_pps_capture": False, "has_safety_switch": False, "has_safety_led": False,
        "has_buzzer": False,
        "num_power_inputs": 1, "has_redundant_power": False,
        "has_dual_battery_monitoring": False, "has_dronecan_power_input": False,
        "power_monitor_type": "analog", "has_sd_card": False,
        "rc_ports_wizard": None, "gps_ports_wizard": None, "power_ports_wizard": None,
        "groups": [
            _group(1, "Timer1", [1, 2], dshot=False),
            _group(2, "Timer4", [3, 4], dshot=False),
            _group(3, "Timer5", [5, 6], dshot=False),
        ],
    }


# ---------------------------------------------------------------------------
# PWM section snapshots
# ---------------------------------------------------------------------------

class TestGeneratePwmSection:
    def test_stm32h7_all_dshot(self, snapshot):
        result = fcdg.generate_pwm_section(BOARD_KEY, _entry_stm32h7_all_dshot())
        snapshot("pwm_stm32h7_all_dshot.md", result)

    def test_stm32h7_mixed_io(self, snapshot):
        result = fcdg.generate_pwm_section(BOARD_KEY, _entry_stm32h7_mixed_io())
        snapshot("pwm_stm32h7_mixed_io.md", result)

    def test_imxrt_all_dshot(self, snapshot):
        result = fcdg.generate_pwm_section(BOARD_KEY, _entry_imxrt())
        snapshot("pwm_imxrt_all_dshot.md", result)

    def test_stm32h7_capture_channels(self, snapshot):
        result = fcdg.generate_pwm_section(BOARD_KEY, _entry_stm32h7_capture_channels())
        snapshot("pwm_stm32h7_capture_channels.md", result)

    def test_stm32f4_no_dshot(self, snapshot):
        result = fcdg.generate_pwm_section(BOARD_KEY, _entry_stm32f4_no_dshot())
        snapshot("pwm_stm32f4_no_dshot.md", result)

    def test_no_groups(self, snapshot):
        entry = {"has_io_board": False, "total_outputs": 0, "io_outputs": 0, "groups": []}
        result = fcdg.generate_pwm_section(BOARD_KEY, entry)
        snapshot("pwm_no_groups.md", result)

    # Basic structural assertions (not snapshot-dependent)
    def test_all_dshot_heading(self):
        result = fcdg.generate_pwm_section(BOARD_KEY, _entry_stm32h7_all_dshot())
        assert "## PWM Outputs" in result

    def test_io_board_labels_main_aux(self):
        result = fcdg.generate_pwm_section(BOARD_KEY, _entry_stm32h7_mixed_io())
        assert "FMU PWM outputs (AUX)" in result
        assert "IO PWM outputs (MAIN)" in result

    def test_no_io_board_labels_main(self):
        result = fcdg.generate_pwm_section(BOARD_KEY, _entry_stm32h7_all_dshot())
        assert "FMU PWM outputs (MAIN)" in result
        assert "IO PWM" not in result

    def test_no_groups_fallback_message(self):
        entry = {"has_io_board": False, "total_outputs": 0, "io_outputs": 0, "groups": []}
        result = fcdg.generate_pwm_section(BOARD_KEY, entry)
        assert "could not be determined" in result


# ---------------------------------------------------------------------------
# Radio Control section snapshots
# ---------------------------------------------------------------------------

class TestGenerateRadioControlSection:
    def _entry_rc_input(self):
        return {
            **_entry_stm32h7_all_dshot(),
            "has_rc_input": True,
            "rc_serial_device": "/dev/ttyS4",
            "serial_ports": [
                {"uart": "USART1", "device": "/dev/ttyS0", "label": "TELEM1"},
                {"uart": "USART2", "device": "/dev/ttyS1", "label": "TELEM2"},
                {"uart": "USART3", "device": "/dev/ttyS2", "label": "Debug Console"},
                {"uart": "UART4", "device": "/dev/ttyS3", "label": "GPS1"},
                {"uart": "USART6", "device": "/dev/ttyS4", "label": "RC"},
                {"uart": "UART7", "device": "/dev/ttyS5", "label": ""},
            ],
        }

    def test_has_rc_input(self, snapshot):
        result = fcdg.generate_radio_control_section(BOARD_KEY, self._entry_rc_input())
        snapshot("rc_has_rc_input.md", result)

    def test_has_common_rc(self, snapshot):
        result = fcdg.generate_radio_control_section(BOARD_KEY, _entry_stm32h7_mixed_io())
        snapshot("rc_has_common_rc.md", result)

    def test_ppm_shared(self, snapshot):
        entry = {
            **self._entry_rc_input(),
            "has_ppm_pin": True,
            "ppm_shared_with_rc_serial": True,
        }
        result = fcdg.generate_radio_control_section(BOARD_KEY, entry)
        snapshot("rc_ppm_shared.md", result)

    def test_ppm_dedicated(self, snapshot):
        entry = {
            **self._entry_rc_input(),
            "has_ppm_pin": True,
            "ppm_shared_with_rc_serial": False,
        }
        result = fcdg.generate_radio_control_section(BOARD_KEY, entry)
        snapshot("rc_ppm_dedicated.md", result)

    def test_no_rc_data(self, snapshot):
        entry = {
            **_entry_stm32h7_all_dshot(),
            "has_rc_input": False, "has_common_rc": False, "has_io_board": False,
            "serial_ports": [],
        }
        result = fcdg.generate_radio_control_section(BOARD_KEY, entry)
        snapshot("rc_no_rc_data.md", result)

    def test_heading_present(self):
        result = fcdg.generate_radio_control_section(BOARD_KEY, self._entry_rc_input())
        assert "## Radio Control" in result

    def test_source_data_comment_present(self):
        result = fcdg.generate_radio_control_section(BOARD_KEY, self._entry_rc_input())
        assert "<!-- rc-source-data" in result

    def test_pwm_encoder_note_present(self):
        result = fcdg.generate_radio_control_section(BOARD_KEY, self._entry_rc_input())
        assert "PPM encoder" in result

    def test_no_ppm_placeholder_for_io_board(self):
        # IO board boards handle PPM via the IO RC port — no separate TODO entry
        result = fcdg.generate_radio_control_section(BOARD_KEY, _entry_stm32h7_mixed_io())
        assert "TODO: PPM port label" not in result

    def test_ppm_placeholder_present_when_no_io_board(self):
        # Dedicated PPM GPIO on FMU-only board → placeholder port listed
        entry = {
            **self._entry_rc_input(),
            "has_ppm_pin": True,
            "ppm_shared_with_rc_serial": False,
            "has_io_board": False,
        }
        result = fcdg.generate_radio_control_section(BOARD_KEY, entry)
        assert "TODO: PPM port label" in result


# ---------------------------------------------------------------------------
# GPS & Compass section snapshots
# ---------------------------------------------------------------------------

class TestGenerateGpsSection:
    def _entry_gps_basic(self):
        return {
            **_entry_stm32h7_all_dshot(),
            "serial_ports": [
                {"uart": "USART1", "device": "/dev/ttyS0", "label": "TELEM1"},
                {"uart": "UART4", "device": "/dev/ttyS3", "label": "GPS1"},
            ],
            "has_safety_switch": False, "has_pps_capture": False,
        }

    def test_basic_gps(self, snapshot):
        result = fcdg.generate_gps_section(BOARD_KEY, self._entry_gps_basic())
        snapshot("gps_basic.md", result)

    def test_with_safety_switch(self, snapshot):
        entry = {**self._entry_gps_basic(), "has_safety_switch": True}
        result = fcdg.generate_gps_section(BOARD_KEY, entry)
        snapshot("gps_with_safety.md", result)

    def test_two_gps_ports(self, snapshot):
        entry = {
            **_entry_stm32h7_mixed_io(),
            "serial_ports": [
                {"uart": "USART1", "device": "/dev/ttyS0", "label": "GPS1"},
                {"uart": "UART4", "device": "/dev/ttyS3", "label": "GPS2"},
            ],
        }
        result = fcdg.generate_gps_section(BOARD_KEY, entry)
        snapshot("gps_two_ports.md", result)

    def test_wizard_override(self, snapshot):
        entry = {
            **self._entry_gps_basic(),
            "gps_ports_wizard": [
                {"label": "GPS1", "device": "/dev/ttyS0", "uart": "USART1"},
            ],
        }
        result = fcdg.generate_gps_section(BOARD_KEY, entry)
        snapshot("gps_wizard_override.md", result)

    def test_heading_present(self):
        result = fcdg.generate_gps_section(BOARD_KEY, self._entry_gps_basic())
        assert "## GPS" in result

    def test_source_data_comment_present(self):
        result = fcdg.generate_gps_section(BOARD_KEY, self._entry_gps_basic())
        assert "<!-- gps-source-data" in result

    def test_full_connector_auto_detected_for_primary_gps(self):
        # All three GPS-connector features present → primary GPS gets 10-pin description
        entry = {**_entry_imxrt(), "serial_ports": [
            {"uart": "LPUART3", "device": "/dev/ttyS1", "label": "GPS1"},
            {"uart": "LPUART4", "device": "/dev/ttyS2", "label": "GPS2"},
        ]}
        result = fcdg.generate_gps_section(BOARD_KEY, entry)
        assert "10-pin JST GH" in result
        assert "safety switch" in result

    def test_full_connector_not_on_secondary_gps(self):
        # GPS2 does not inherit the full-connector description — it shows TODO
        entry = {**_entry_imxrt(), "serial_ports": [
            {"uart": "LPUART3", "device": "/dev/ttyS1", "label": "GPS1"},
            {"uart": "LPUART4", "device": "/dev/ttyS2", "label": "GPS2"},
        ]}
        result = fcdg.generate_gps_section(BOARD_KEY, entry)
        # GPS1 → full 10-pin; GPS2 → unknown (TODO)
        assert "10-pin JST GH" in result
        assert "TODO: connector type" in result

    def test_basic_connector_when_no_safety_features(self):
        # No safety switch/LED/buzzer → TODO connector type (not auto-detected)
        result = fcdg.generate_gps_section(BOARD_KEY, self._entry_gps_basic())
        assert "TODO: connector type" in result


# ---------------------------------------------------------------------------
# Serial section snapshots
# ---------------------------------------------------------------------------

class TestGenerateSerialSection:
    def test_stm32h7_serial(self, snapshot):
        result = fcdg.generate_serial_section(BOARD_KEY, _entry_stm32h7_all_dshot())
        snapshot("serial_stm32h7.md", result)

    def test_imxrt_serial(self, snapshot):
        result = fcdg.generate_serial_section(BOARD_KEY, _entry_imxrt())
        snapshot("serial_imxrt.md", result)

    def test_empty_ports(self, snapshot):
        entry = {**_entry_stm32h7_all_dshot(), "serial_ports": []}
        result = fcdg.generate_serial_section(BOARD_KEY, entry)
        snapshot("serial_empty.md", result)

    def test_serial_with_flow_control(self, snapshot):
        result = fcdg.generate_serial_section(BOARD_KEY, _entry_stm32h7_mixed_io())
        snapshot("serial_with_flow_control.md", result)

    def test_heading_present(self):
        result = fcdg.generate_serial_section(BOARD_KEY, _entry_stm32h7_all_dshot())
        assert "## Serial" in result


# ---------------------------------------------------------------------------
# Power section snapshots and structural tests
# ---------------------------------------------------------------------------

class TestGeneratePowerSection:
    def _entry_single(self):
        return {**_entry_stm32h7_all_dshot()}

    def _entry_dual(self):
        return {**_entry_stm32h7_mixed_io()}

    def _entry_ina238(self):
        return {**_entry_imxrt()}

    def test_single_analog_snapshot(self, snapshot):
        result = fcdg.generate_power_section(BOARD_KEY, self._entry_single())
        snapshot("power_single_analog.md", result)

    def test_dual_redundant_snapshot(self, snapshot):
        result = fcdg.generate_power_section(BOARD_KEY, self._entry_dual())
        snapshot("power_dual_redundant.md", result)

    def test_digital_ina238_snapshot(self, snapshot):
        result = fcdg.generate_power_section(BOARD_KEY, self._entry_ina238())
        snapshot("power_digital_ina238.md", result)

    def test_wizard_override_snapshot(self, snapshot):
        entry = {
            **self._entry_dual(),
            "power_ports_wizard": [
                {"label": "POWER 1", "connector_type": "6-pin Molex CLIK-Mate"},
                {"label": "POWER 2", "connector_type": "6-pin Molex CLIK-Mate"},
            ],
        }
        result = fcdg.generate_power_section(BOARD_KEY, entry)
        snapshot("power_wizard_override.md", result)

    def test_heading_present(self):
        result = fcdg.generate_power_section(BOARD_KEY, self._entry_single())
        assert "## Power {#power}" in result

    def test_servo_rail_warning_always_present(self):
        result = fcdg.generate_power_section(BOARD_KEY, self._entry_single())
        assert "separately powered" in result
        assert "VTOL" in result

    def test_battery_config_link_always_present(self):
        result = fcdg.generate_power_section(BOARD_KEY, self._entry_single())
        assert "battery.md" in result

    def test_todo_present_without_wizard(self):
        result = fcdg.generate_power_section(BOARD_KEY, self._entry_single())
        assert "TODO: POWER port label" in result

    def test_todo_absent_with_wizard(self):
        entry = {
            **self._entry_single(),
            "power_ports_wizard": [
                {"label": "POWER", "connector_type": "6-pin JST GH"},
            ],
        }
        result = fcdg.generate_power_section(BOARD_KEY, entry)
        assert "TODO:" not in result

    def test_redundancy_text_present_for_dual(self):
        # The redundancy sentence appears in the prose before the JSON comment
        prose = fcdg.generate_power_section(BOARD_KEY, self._entry_dual()).split("<!-- power-source-data")[0]
        assert "redundant" in prose.lower()

    def test_redundancy_text_absent_for_single(self):
        # No redundancy sentence in the prose (JSON comment may still contain the field name)
        prose = fcdg.generate_power_section(BOARD_KEY, self._entry_single()).split("<!-- power-source-data")[0]
        assert "redundant" not in prose.lower()

    def test_source_data_comment_present(self):
        result = fcdg.generate_power_section(BOARD_KEY, self._entry_single())
        assert "<!-- power-source-data" in result

    def test_dronecan_port_snapshot(self, snapshot):
        entry = {
            **self._entry_single(),
            "power_ports_wizard": [
                {"label": "POWER", "connector_type": "6-pin JST GH",
                 "monitor_type": "dronecan"},
            ],
        }
        result = fcdg.generate_power_section(BOARD_KEY, entry)
        snapshot("power_dronecan_port.md", result)

    def test_mixed_analog_dronecan_snapshot(self, snapshot):
        entry = {
            **self._entry_dual(),
            "power_ports_wizard": [
                {"label": "POWER 1", "connector_type": "6-pin Molex CLIK-Mate",
                 "monitor_type": "analog"},
                {"label": "POWER C1", "connector_type": "6-pin JST GH",
                 "monitor_type": "dronecan"},
            ],
        }
        result = fcdg.generate_power_section(BOARD_KEY, entry)
        snapshot("power_mixed_analog_dronecan.md", result)

    def test_dronecan_port_contains_uavcan_enable(self):
        entry = {
            **self._entry_single(),
            "power_ports_wizard": [
                {"label": "POWER", "connector_type": "6-pin JST GH",
                 "monitor_type": "dronecan"},
            ],
        }
        result = fcdg.generate_power_section(BOARD_KEY, entry)
        assert "UAVCAN_ENABLE" in result

    def test_dronecan_port_links_to_power_module_index(self):
        entry = {
            **self._entry_single(),
            "power_ports_wizard": [
                {"label": "POWER", "connector_type": "6-pin JST GH",
                 "monitor_type": "dronecan"},
            ],
        }
        result = fcdg.generate_power_section(BOARD_KEY, entry)
        assert "power_module/index.md" in result

    def test_analog_port_does_not_contain_uavcan(self):
        entry = {
            **self._entry_single(),
            "power_ports_wizard": [
                {"label": "POWER", "connector_type": "6-pin JST GH",
                 "monitor_type": "analog"},
            ],
        }
        result = fcdg.generate_power_section(BOARD_KEY, entry)
        # prose section only (JSON comment excluded)
        prose = result.split("<!-- power-source-data")[0]
        assert "UAVCAN_ENABLE" not in prose

    def test_mixed_intro_does_not_claim_pure_redundancy(self):
        entry = {
            **self._entry_dual(),
            "power_ports_wizard": [
                {"label": "POWER 1", "connector_type": "6-pin Molex CLIK-Mate",
                 "monitor_type": "analog"},
                {"label": "POWER C1", "connector_type": "6-pin JST GH",
                 "monitor_type": "dronecan"},
            ],
        }
        prose = fcdg.generate_power_section(BOARD_KEY, entry).split("<!-- power-source-data")[0]
        # Mixed board should not claim "redundant power inputs" in the intro
        assert "redundant power inputs" not in prose.lower()


# ---------------------------------------------------------------------------
# Telemetry section snapshots and structural tests
# ---------------------------------------------------------------------------

class TestGenerateTelemetrySection:
    def _entry_single_telem(self):
        return {
            **_entry_stm32f4_no_dshot(),
            "serial_ports": [
                {"uart": "UART4", "device": "/dev/ttyS3", "label": "GPS1"},
                {"uart": "UART5", "device": "/dev/ttyS4", "label": "TELEM1"},
                {"uart": "USART6", "device": "/dev/ttyS5", "label": "RC"},
            ],
        }

    def _entry_multi_telem(self):
        return {
            **_entry_stm32h7_all_dshot(),
            "serial_ports": [
                {"uart": "USART1", "device": "/dev/ttyS0", "label": "TELEM1"},
                {"uart": "USART2", "device": "/dev/ttyS1", "label": "TELEM2"},
                {"uart": "UART4", "device": "/dev/ttyS3", "label": "GPS1"},
                {"uart": "USART6", "device": "/dev/ttyS4", "label": "RC"},
                {"uart": "UART7", "device": "/dev/ttyS5", "label": "TELEM3"},
            ],
        }

    def test_single_telem_snapshot(self, snapshot):
        result = fcdg.generate_telemetry_section(BOARD_KEY, self._entry_single_telem())
        snapshot("telemetry_single.md", result)

    def test_multi_telem_snapshot(self, snapshot):
        result = fcdg.generate_telemetry_section(BOARD_KEY, self._entry_multi_telem())
        snapshot("telemetry_multi.md", result)

    def test_heading_present(self):
        result = fcdg.generate_telemetry_section(BOARD_KEY, self._entry_single_telem())
        assert "## Telemetry Radios (Optional) {#telemetry}" in result

    def test_no_telem_ports_todo_placeholder(self):
        entry = {**_entry_stm32h7_all_dshot(), "serial_ports": []}
        result = fcdg.generate_telemetry_section(BOARD_KEY, entry)
        assert "TODO: TELEM port label" in result

    def test_single_says_telem1(self):
        result = fcdg.generate_telemetry_section(BOARD_KEY, self._entry_single_telem())
        assert "TELEM1" in result

    def test_multi_says_all_ports(self):
        result = fcdg.generate_telemetry_section(BOARD_KEY, self._entry_multi_telem())
        assert "TELEM1" in result
        assert "TELEM2" in result
        assert "TELEM3" in result

    def test_multi_says_telem1_default(self):
        # For multi-port boards TELEM1 is still called out as the no-config default
        result = fcdg.generate_telemetry_section(BOARD_KEY, self._entry_multi_telem())
        prose = result.split("<!-- telemetry-source-data")[0]
        assert "TELEM1" in prose
        assert "no further configuration" in prose

    def test_source_comment_present(self):
        result = fcdg.generate_telemetry_section(BOARD_KEY, self._entry_single_telem())
        assert "<!-- telemetry-source-data" in result


# ---------------------------------------------------------------------------
# SD Card section snapshots and structural tests
# ---------------------------------------------------------------------------

class TestGenerateSDCardSection:
    def _entry_with_sd(self):
        return {**_entry_stm32h7_all_dshot()}   # has_sd_card=True

    def _entry_no_sd(self):
        return {**_entry_stm32f4_no_dshot()}   # has_sd_card=False

    def test_with_sd_snapshot(self, snapshot):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_with_sd())
        snapshot("sd_card_present.md", result)

    def test_no_sd_snapshot(self, snapshot):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_no_sd())
        snapshot("sd_card_absent.md", result)

    def test_heading_present_with_sd(self):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_with_sd())
        assert "## SD Card" in result

    def test_heading_present_no_sd(self):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_no_sd())
        assert "## SD Card" in result

    def test_with_sd_has_tip_block(self):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_with_sd())
        assert ":::tip" in result

    def test_with_sd_links_to_basic_concepts(self):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_with_sd())
        assert "px4_basic_concepts.md#sd-cards" in result

    def test_no_sd_says_no_slot(self):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_no_sd())
        assert "does not have an SD card slot" in result

    def test_with_sd_no_slot_message(self):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_with_sd())
        assert "does not have an SD card slot" not in result

    def test_source_comment_present(self):
        result = fcdg.generate_sd_card_section(BOARD_KEY, self._entry_with_sd())
        assert "<!-- sd-source-data" in result


# ---------------------------------------------------------------------------
# generate_full_template — hero image asset path
# ---------------------------------------------------------------------------

class TestGenerateFullTemplate:
    def test_image_uses_doc_name_folder(self):
        """When doc_name is passed, the hero image path uses that folder."""
        result = fcdg.generate_full_template(
            "holybro/durandal-v1", {},
            manufacturer="Holybro", product="Durandal",
            doc_name="holybro_durandal",
        )
        assert "assets/flight_controller/holybro_durandal/holybro_durandal_hero.jpg" in result
        assert "TODO/TODO" not in result

    def test_image_fallback_derives_from_manufacturer_product(self):
        """Without doc_name, falls back to _doc_filename(manufacturer, product)."""
        result = fcdg.generate_full_template(
            "holybro/durandal-v1", {},
            manufacturer="Holybro", product="Durandal",
        )
        assert "assets/flight_controller/holybro_durandal/holybro_durandal_hero.jpg" in result
        assert "TODO/TODO" not in result

    def test_image_fallback_todo_when_no_manufacturer(self):
        """Without manufacturer or doc_name, asset folder falls back to TODO."""
        result = fcdg.generate_full_template("holybro/durandal-v1", {})
        assert "assets/flight_controller/TODO/TODO_hero.jpg" in result


# ---------------------------------------------------------------------------
# Specifications section snapshots
# ---------------------------------------------------------------------------

def _spec_base_entry():
    """Minimal entry for specifications tests (no sensors, no buses)."""
    return {
        "chip_model": "STM32H753",
        "has_io_board": False, "io_outputs": 0,
        "groups": [],
        "serial_ports": [],
        "num_i2c_buses": 0, "num_spi_buses": 0, "num_can_buses": 0,
        "has_usb": False,
        "has_rc_input": False, "has_common_rc": False, "rc_serial_device": None,
        "has_ppm_pin": False, "ppm_shared_with_rc_serial": False,
        "num_power_inputs": 1, "has_redundant_power": False,
        "has_dual_battery_monitoring": False, "has_dronecan_power_input": False,
        "power_monitor_type": "analog",
        "has_ethernet": False, "has_sd_card": False, "has_heater": False,
        "sensor_imu_drivers": [], "sensor_baro_drivers": [],
        "sensor_mag_drivers": [], "sensor_osd_drivers": [],
        "sensor_bus_info": None,
        "overview_wizard": None,
    }


def _entry_i2c_detailed():
    """Entry with mixed internal/external I2C sensors and a wizard label for the external bus."""
    e = _spec_base_entry()
    e.update({
        "num_i2c_buses": 4,
        "sensor_bus_info": {
            "imu":  [],
            "baro": [{"name": "BMP388",  "bus_type": "I2C", "bus_num": None, "external": False}],
            "mag":  [
                {"name": "IST8310", "bus_type": "I2C", "bus_num": 1, "external": True},
                {"name": "RM3100",  "bus_type": "I2C", "bus_num": None, "external": False},
            ],
            "osd":  [],
        },
        "i2c_buses_wizard": [{"bus_num": 1, "label": "GPS1"}],
    })
    return e


class TestGenerateSpecificationsSection:
    def test_spi_sensors_with_subbullets(self, snapshot):
        """SPI-flagged sensors produce bus annotations and SPI sub-bullets."""
        entry = _spec_base_entry()
        entry.update({
            "num_spi_buses": 4,
            "sensor_imu_drivers": ["ICM-42688P"],
            "sensor_baro_drivers": ["MS5611"],
            "sensor_mag_drivers": [],
            "sensor_bus_info": {
                "imu":  [{"name": "ICM-42688P", "bus_type": "SPI", "bus_num": None, "external": False}],
                "baro": [{"name": "MS5611",     "bus_type": "SPI", "bus_num": None, "external": False}],
                "mag":  [],
                "osd":  [],
            },
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_spi_sensors.md", result)

    def test_i2c_sensors_internal_and_external(self, snapshot):
        """I2C sensors with internal + external instances produce bus annotations and I2C sub-bullets."""
        entry = _spec_base_entry()
        entry.update({
            "num_i2c_buses": 4,
            "sensor_mag_drivers": ["IST8310"],
            "sensor_bus_info": {
                "imu":  [],
                "baro": [],
                "mag": [
                    {"name": "IST8310", "bus_type": "I2C", "bus_num": None, "external": False},
                    {"name": "IST8310", "bus_type": "I2C", "bus_num": 1,    "external": True},
                ],
                "osd":  [],
            },
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_i2c_sensors.md", result)

    def test_no_bus_info_fallback(self, snapshot):
        """Without sensor_bus_info, sensor names show without bus annotation."""
        entry = _spec_base_entry()
        entry.update({
            "num_spi_buses": 3,
            "sensor_imu_drivers": ["ICM-42688P"],
            "sensor_baro_drivers": ["MS5611"],
            "sensor_mag_drivers": ["IST8310"],
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_no_bus_info.md", result)

    def test_wizard_override_no_bus_annotation(self, snapshot):
        """Wizard-supplied sensor values bypass bus annotation entirely."""
        entry = _spec_base_entry()
        entry.update({
            "num_spi_buses": 3,
            "sensor_imu_drivers": ["ICM-42688P"],
            "sensor_bus_info": {
                "imu":  [{"name": "ICM-42688P", "bus_type": "SPI", "bus_num": None, "external": False}],
                "baro": [], "mag": [], "osd": [],
            },
            "overview_wizard": {"imu": "ICM-42688P"},
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_wizard_override.md", result)

    def test_duplicate_sensors_counted(self, snapshot):
        """Duplicate sensor names in wizard list collapse to Name (N) format."""
        entry = _spec_base_entry()
        entry.update({
            "num_spi_buses": 2,
            "sensor_baro_drivers": ["MS5611"],
            "overview_wizard": {
                "baro": ["MS5611", "MS5611"],
            },
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_sensor_counted.md", result)

    def test_unused_driver_todo_inline(self, snapshot):
        """Drivers detected but not in wizard list get an inline TODO comment."""
        entry = _spec_base_entry()
        entry.update({
            "num_spi_buses": 3,
            "sensor_imu_drivers": ["ICM-20649", "ICM-20689", "BMI088", "ICM-20948", "ICM-42688P"],
            "sensor_mag_drivers": ["RM3100", "IST8310"],
            "overview_wizard": {
                "imu": ["ICM-20649", "ICM-20689", "BMI088"],
                "mag": ["RM3100"],
            },
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_unused_driver_todo.md", result)

    def test_electrical_full(self, snapshot):
        """Full electrical data: operating voltage, USB power, high-voltage servo rail, triple redundancy."""
        entry = _spec_base_entry()
        entry.update({
            "num_power_inputs": 2,
            "has_redundant_power": True,
            "power_ports_wizard": [
                {"label": "POWER1", "connector_type": "6-pin Molex CLIK-Mate"},
                {"label": "POWER2", "connector_type": "6-pin Molex CLIK-Mate"},
            ],
            "overview_wizard": {
                "min_voltage": "7.0",
                "max_voltage": "55.0",
                "usb_powers_fc": True,
                "usb_pwr_min_v": "4.75",
                "usb_pwr_max_v": "5.25",
                "has_servo_rail": True,
                "servo_rail_max_v": "36",
                "width_mm": "46",
                "length_mm": "64",
                "height_mm": "22",
                "weight_g": 75.0,
            },
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_electrical_full.md", result)

    def test_electrical_normal_servo_rail(self, snapshot):
        """Servo rail ≤12 V gets 'Up to' label (not High-voltage)."""
        entry = _spec_base_entry()
        entry.update({
            "num_power_inputs": 2,
            "has_redundant_power": True,
            "overview_wizard": {
                "min_voltage": "4.3",
                "max_voltage": "5.4",
                "usb_powers_fc": True,
                "usb_pwr_min_v": "4.75",
                "usb_pwr_max_v": "5.25",
                "has_servo_rail": True,
                "servo_rail_max_v": "10.5",
            },
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_electrical_normal_servo.md", result)

    def test_usb_multi_labels(self, snapshot):
        """Multiple USB ports with distinct labels show Label (ConnectorType) pairs."""
        entry = _spec_base_entry()
        entry.update({
            "has_usb": True,
            "overview_wizard": {
                "usb_connectors": ["USB-C", "Micro-USB"],
                "usb_labels": ["USB", "Debug USB"],
            },
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_usb_multi_labels.md", result)

    def test_usb_single_default_label(self, snapshot):
        """Single USB port with default label 'USB' shows connector type only."""
        entry = _spec_base_entry()
        entry.update({
            "has_usb": True,
            "overview_wizard": {
                "usb_connectors": ["USB-C"],
                "usb_labels": ["USB"],
            },
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_usb_single_label.md", result)

    def test_i2c_external_with_label(self, snapshot):
        """External I2C bus with wizard label shows label + sensor; internal summarised."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_detailed())
        snapshot("spec_i2c_external_label.md", result)

    def test_i2c_external_no_label(self, snapshot):
        """External I2C bus without wizard label shows TODO placeholder."""
        e = _entry_i2c_detailed()
        del e['i2c_buses_wizard']
        result = fcdg.generate_specifications_section(BOARD_KEY, e)
        snapshot("spec_i2c_external_no_label.md", result)

    def test_i2c_count_line_split(self):
        """Count line shows internal/external split when external buses are detected."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_detailed())
        assert '**I2C ports**: 4 (3 internal, 1 external)' in result

    def test_i2c_external_sub_bullet(self):
        """External bus sub-bullet shows wizard label, bus number, and sensor purpose."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_detailed())
        assert 'GPS1 (I2C1, external): IST8310 (magnetometer)' in result

    def test_i2c_gps_connector_note(self):
        """External I2C bus labelled GPS1 appends '— on GPS connector' note."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_detailed())
        assert 'GPS1 (I2C1, external): IST8310 (magnetometer) — on GPS connector' in result

    def test_i2c_internal_summary_line(self):
        """Internal sensors appear on a single summary sub-bullet."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_detailed())
        assert '    - Internal: BMP388 (barometer), RM3100 (magnetometer)' in result

    def test_i2c_port_label_auto_fill_from_source(self):
        """port_label in sensor_bus_info auto-fills label when no wizard label is set."""
        e = _entry_i2c_detailed()
        del e['i2c_buses_wizard']
        # Add port_label to the external sensor entry
        e['sensor_bus_info']['mag'][0]['port_label'] = 'GPS1'
        result = fcdg.generate_specifications_section(BOARD_KEY, e)
        # Should use auto-detected label instead of TODO placeholder
        assert 'GPS1 (I2C1, external): IST8310 (magnetometer) — on GPS connector' in result
        assert 'TODO: label for I2C bus 1' not in result


# ---------------------------------------------------------------------------
# Specifications section — I2C bus config (detailed) path
# ---------------------------------------------------------------------------

def _entry_i2c_bus_config():
    """Entry with i2c_bus_config: buses 1/2/4 external, bus 3 internal."""
    e = _spec_base_entry()
    e.update({
        "num_i2c_buses": 4,
        "i2c_bus_config": {"external": [1, 2, 4], "internal": [3]},
        "sensor_bus_info": {
            "imu":  [],
            "baro": [{"name": "BMP388", "bus_type": "I2C", "bus_num": None,
                      "external": False, "port_label": None}],
            "mag":  [{"name": "IST8310", "bus_type": "I2C", "bus_num": 1,
                      "external": True, "port_label": None}],
            "osd":  [],
            "power_monitor": [{"name": "INA226", "bus_type": "I2C", "bus_num": 2,
                                "external": True, "port_label": None}],
        },
        "i2c_buses_wizard": [
            {"bus_num": 1, "label": "GPS1"},
            {"bus_num": 2, "label": "POWER"},
        ],
    })
    return e


class TestI2cBusConfigPath:
    def test_count_line_external_internal_split(self):
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_bus_config())
        assert '**I2C ports**: 4 (3 external, 1 internal)' in result

    def test_free_external_bus_marked(self):
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_bus_config())
        assert 'I2C4 (external): free (no sensor detected)' in result

    def test_external_bus_with_sensor_and_gps_note(self):
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_bus_config())
        assert 'I2C1 (external, GPS1): IST8310 (magnetometer) — on GPS connector' in result

    def test_external_bus_with_power_monitor(self):
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_bus_config())
        assert 'I2C2 (external, POWER): INA226 (power monitor)' in result

    def test_internal_bus_with_internal_sensor(self):
        """Single internal bus gets the internal sensor list retrofitted onto it."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_bus_config())
        assert 'I2C3 (internal): BMP388 (barometer)' in result

    def test_fallback_path_unchanged(self):
        """Entry without i2c_bus_config uses existing format."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_detailed())
        assert '**I2C ports**: 4 (3 internal, 1 external)' in result
        assert 'GPS1 (I2C1, external): IST8310 (magnetometer) — on GPS connector' in result

    def test_snapshot(self, snapshot):
        """Full snapshot of i2c_bus_config present path."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _entry_i2c_bus_config())
        snapshot("spec_i2c_bus_config.md", result)


# ---------------------------------------------------------------------------
# Specifications section — IO board RC input
# ---------------------------------------------------------------------------

class TestSpecIoBoardRc:
    def test_io_board_with_ppm(self, snapshot):
        """IO board with PPM pin shows full IO protocol list including PPM."""
        entry = _spec_base_entry()
        entry.update({
            "has_io_board": True,
            "io_outputs": 8,
            "has_rc_input": True,
            "has_ppm_pin": True,
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_io_board_rc_with_ppm.md", result)

    def test_io_board_without_ppm(self, snapshot):
        """IO board without PPM pin shows full IO protocol list without PPM."""
        entry = _spec_base_entry()
        entry.update({
            "has_io_board": True,
            "io_outputs": 8,
            "has_rc_input": True,
            "has_ppm_pin": False,
        })
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_io_board_rc_no_ppm.md", result)


# ---------------------------------------------------------------------------
# Specifications section — variant tests
# ---------------------------------------------------------------------------

def _spec_variant_entry():
    """Entry with sensor_variant_info matching the stm32h7_variant fixture."""
    e = _spec_base_entry()
    e.update({
        "num_spi_buses": 5,
        "num_i2c_buses": 4,
        "sensor_imu_drivers": ["ICM-42688P", "BMI088", "ICM-20602"],
        "sensor_baro_drivers": ["MS5611"],
        "sensor_mag_drivers": ["IST8310"],
        "sensor_bus_info": {
            "imu":  [
                {"name": "ICM-42688P", "bus_type": "SPI", "bus_num": None, "external": False},
                {"name": "BMI088",     "bus_type": "SPI", "bus_num": None, "external": False},
                {"name": "ICM-20602",  "bus_type": "SPI", "bus_num": None, "external": False},
            ],
            "baro": [{"name": "MS5611",  "bus_type": "SPI", "bus_num": None, "external": False}],
            "mag":  [
                {"name": "IST8310", "bus_type": "I2C", "bus_num": None, "external": False},
                {"name": "IST8310", "bus_type": "I2C", "bus_num": 1,    "external": True},
            ],
            "osd":  [],
        },
        "sensor_variant_info": {
            "has_variants": True,
            "unconditional": {
                "imu":  [{"name": "ICM-42688P", "bus_type": "SPI", "bus_num": None, "external": False}],
                "baro": [{"name": "MS5611",     "bus_type": "SPI", "bus_num": None, "external": False}],
                "mag":  [
                    {"name": "IST8310", "bus_type": "I2C", "bus_num": None, "external": False},
                    {"name": "IST8310", "bus_type": "I2C", "bus_num": 1,    "external": True},
                ],
                "osd":  [],
            },
            "variants": {
                "HW000001": {
                    "imu":  [{"name": "BMI088",    "bus_type": "SPI", "bus_num": None, "external": False}],
                    "baro": [], "mag": [], "osd": [],
                },
                "__other__": {
                    "imu":  [{"name": "ICM-20602", "bus_type": "SPI", "bus_num": None, "external": False}],
                    "baro": [], "mag": [], "osd": [],
                },
            },
        },
    })
    return e


class TestGenerateSpecSectionVariants:
    def test_variant_base_sensors_shown_without_todo(self, snapshot):
        """With variant data, unconditional sensors show without TODO."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _spec_variant_entry())
        snapshot("spec_with_variants.md", result)

    def test_variant_unique_sensors_annotated(self):
        """Variant-unique sensors appear annotated, not in unconditional base."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _spec_variant_entry())
        assert "ICM-42688P (SPI)" in result                       # unconditional
        assert "BMI088 (SPI, variant HW000001)" in result         # variant-unique
        assert "ICM-20602 (SPI, other variants)" in result        # __other__
        assert "TODO: confirm which is installed" not in result   # no TODO on sensor lines

    def test_variant_spi_subbullets_unconditional_only(self):
        """SPI sub-bullets use unconditional sensors only, not variant-specific."""
        result = fcdg.generate_specifications_section(BOARD_KEY, _spec_variant_entry())
        lines = result.splitlines()
        spi_idx = next(i for i, l in enumerate(lines) if '**SPI buses**' in l)
        subbullets = [l for l in lines[spi_idx+1:] if l.startswith('    -')]
        sub_names = [l.strip('- ').split(' ')[0] for l in subbullets]
        assert 'ICM-42688P' in sub_names    # unconditional IMU
        assert 'MS5611' in sub_names        # unconditional baro
        assert 'BMI088' not in sub_names    # variant-only
        assert 'ICM-20602' not in sub_names # variant-only

    def test_variant_with_wizard_labels(self, snapshot):
        """Variant labels from wizard are used in place of raw codes."""
        entry = _spec_variant_entry()
        entry["overview_wizard"] = {
            "sensor_variant_labels": {"HW000001": "Rev 1.0", "__other__": "Rev 1.1"}
        }
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        snapshot("spec_with_variant_labels.md", result)

    def test_variant_labels_shown_in_output(self):
        """With labels set, annotations use human-readable revision names."""
        entry = _spec_variant_entry()
        entry["overview_wizard"] = {
            "sensor_variant_labels": {"HW000001": "Rev 1.0", "__other__": "Rev 1.1"}
        }
        result = fcdg.generate_specifications_section(BOARD_KEY, entry)
        assert "hardware revision Rev 1.0" in result
        assert "hardware revision Rev 1.1" in result
