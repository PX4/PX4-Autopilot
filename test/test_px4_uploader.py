import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "Tools"))

import px4_uploader


def _fw(board_id: int, path: str) -> SimpleNamespace:
    return SimpleNamespace(board_id=board_id, path=path)


class TestPx4UploaderModes(unittest.TestCase):
    """Tests use a test board ID mapping (1069->1070) injected via setUp."""

    def setUp(self) -> None:
        self._orig_map = px4_uploader.Uploader.SECONDARY_BOARD_ID_MAP.copy()
        px4_uploader.Uploader.SECONDARY_BOARD_ID_MAP = {1069: 1070}

    def tearDown(self) -> None:
        px4_uploader.Uploader.SECONDARY_BOARD_ID_MAP = self._orig_map

    def test_primary_mode_uses_primary_only(self) -> None:
        uploader = px4_uploader.Uploader(
            px4_uploader.UploaderConfig(update_mode="primary")
        )
        plan = uploader._resolve_upload_plan(
            [_fw(1069, "primary.px4"), _fw(1070, "secondary.px4")], 1069
        )
        self.assertTrue(plan.upload_primary)
        self.assertFalse(plan.upload_secondary)
        self.assertEqual(plan.primary_firmware.board_id, 1069)

    def test_both_mode_requires_secondary_image(self) -> None:
        uploader = px4_uploader.Uploader(
            px4_uploader.UploaderConfig(update_mode="both")
        )
        with self.assertRaises(px4_uploader.BoardMismatchError):
            uploader._resolve_upload_plan([_fw(1069, "primary.px4")], 1069)

    def test_secondary_mode_requires_mapped_secondary_on_primary(self) -> None:
        uploader = px4_uploader.Uploader(
            px4_uploader.UploaderConfig(update_mode="secondary")
        )
        with self.assertRaises(px4_uploader.BoardMismatchError):
            uploader._resolve_upload_plan([_fw(1069, "primary.px4")], 1069)

    def test_secondary_mode_on_secondary_board_is_direct(self) -> None:
        uploader = px4_uploader.Uploader(
            px4_uploader.UploaderConfig(update_mode="secondary")
        )
        plan = uploader._resolve_upload_plan([_fw(1070, "secondary.px4")], 1070)
        self.assertTrue(plan.upload_primary)
        self.assertFalse(plan.upload_secondary)
        self.assertEqual(plan.primary_firmware.board_id, 1070)

    @patch.object(px4_uploader, "SerialTransport")
    @patch.object(px4_uploader, "BootloaderProtocol")
    def test_upload_to_port_both_calls_primary_then_secondary(
        self,
        protocol_cls: MagicMock,
        transport_cls: MagicMock,
    ) -> None:
        transport = MagicMock()
        transport.port_name = "/dev/fake"
        protocol = MagicMock()
        protocol.board_type = 1069
        transport_cls.return_value = transport
        protocol_cls.return_value = protocol

        uploader = px4_uploader.Uploader(px4_uploader.UploaderConfig(update_mode="both"))
        uploader._try_identify = MagicMock(return_value=True)
        primary_fw = _fw(1069, "primary.px4")
        secondary_fw = _fw(1070, "secondary.px4")
        uploader._resolve_upload_plan = MagicMock(
            return_value=px4_uploader.UploadPlan(
                primary_firmware=primary_fw,
                secondary_firmware=secondary_fw,
                upload_primary=True,
                upload_secondary=True,
            )
        )
        uploader._do_upload = MagicMock()
        uploader._upload_secondary_firmware = MagicMock()

        self.assertTrue(uploader._upload_to_port("/dev/fake", [primary_fw, secondary_fw]))
        uploader._do_upload.assert_called_once_with(protocol, primary_fw, reboot=False)
        uploader._upload_secondary_firmware.assert_called_once_with(
            transport, protocol, secondary_fw
        )
        transport.open.assert_called_once()
        transport.close.assert_called_once()

    @patch.object(px4_uploader, "BootloaderProtocol")
    def test_secondary_upload_reboots_primary_after_failure(
        self, protocol_cls: MagicMock
    ) -> None:
        uploader = px4_uploader.Uploader(px4_uploader.UploaderConfig(update_mode="secondary"))
        uploader._do_upload = MagicMock(side_effect=px4_uploader.UploadError("secondary failed"))
        uploader._wait_for_primary_after_serial_forward = MagicMock()

        secondary_protocol = MagicMock()
        secondary_protocol.board_type = 1070
        secondary_protocol.board_rev = 0
        secondary_protocol.bl_rev = 6
        protocol_cls.return_value = secondary_protocol

        transport = MagicMock()
        transport.port_name = "/dev/fake"
        primary_protocol = MagicMock()
        secondary_fw = _fw(1070, "secondary.px4")

        with self.assertRaises(px4_uploader.UploadError):
            uploader._upload_secondary_firmware(transport, primary_protocol, secondary_fw)

        primary_protocol.start_serial_forward.assert_called_once()
        uploader._wait_for_primary_after_serial_forward.assert_called_once_with(
            primary_protocol
        )
        primary_protocol.reboot.assert_called_once()

    def test_auto_mode_selects_both_when_secondary_available(self) -> None:
        uploader = px4_uploader.Uploader(
            px4_uploader.UploaderConfig(update_mode="auto")
        )
        plan = uploader._resolve_upload_plan(
            [_fw(1069, "primary.px4"), _fw(1070, "secondary.px4")], 1069
        )
        self.assertTrue(plan.upload_primary)
        self.assertTrue(plan.upload_secondary)
        self.assertEqual(plan.primary_firmware.board_id, 1069)
        self.assertEqual(plan.secondary_firmware.board_id, 1070)

    def test_auto_mode_falls_back_to_primary_without_secondary(self) -> None:
        uploader = px4_uploader.Uploader(
            px4_uploader.UploaderConfig(update_mode="auto")
        )
        plan = uploader._resolve_upload_plan([_fw(1069, "primary.px4")], 1069)
        self.assertTrue(plan.upload_primary)
        self.assertFalse(plan.upload_secondary)
        self.assertEqual(plan.primary_firmware.board_id, 1069)

    @patch("time.monotonic")
    @patch("time.sleep")
    def test_wait_for_primary_times_out(
        self, mock_sleep: MagicMock, mock_monotonic: MagicMock
    ) -> None:
        mock_monotonic.side_effect = [0.0, 0.1, 0.2, 5.1]
        protocol = MagicMock()
        protocol.sync.side_effect = px4_uploader.ProtocolError("no sync")
        protocol.transport.port_name = "/dev/fake"

        uploader = px4_uploader.Uploader(px4_uploader.UploaderConfig())
        with self.assertRaises(px4_uploader.TimeoutError):
            uploader._wait_for_primary_after_serial_forward(protocol, timeout_s=5.0)


if __name__ == "__main__":
    unittest.main()
