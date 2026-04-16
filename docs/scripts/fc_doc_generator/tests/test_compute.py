"""
Tests for compute_groups() and compute_bdshot() in fc_doc_generator.py.
"""
import fc_doc_generator as fcdg


def _parse(board_path):
    """Helper: run both timer parsers and return (timers, channels)."""
    tc = fcdg.parse_timer_config(board_path)
    return tc.get("timers", []), tc.get("channels", [])


# ---------------------------------------------------------------------------
# compute_groups
# ---------------------------------------------------------------------------

class TestComputeGroups:
    def test_stm32h7_all_dshot_group_count(self, board_stm32h7_all_dshot):
        timers, channels = _parse(board_stm32h7_all_dshot)
        groups = fcdg.compute_groups(timers, channels)
        assert len(groups) == 4

    def test_stm32h7_all_dshot_outputs_per_group(self, board_stm32h7_all_dshot):
        timers, channels = _parse(board_stm32h7_all_dshot)
        groups = fcdg.compute_groups(timers, channels)
        assert all(len(g["outputs"]) == 2 for g in groups)

    def test_stm32h7_all_dshot_all_dshot(self, board_stm32h7_all_dshot):
        timers, channels = _parse(board_stm32h7_all_dshot)
        groups = fcdg.compute_groups(timers, channels)
        assert all(g["dshot"] for g in groups)

    def test_stm32h7_all_dshot_output_indices(self, board_stm32h7_all_dshot):
        timers, channels = _parse(board_stm32h7_all_dshot)
        groups = fcdg.compute_groups(timers, channels)
        all_outputs = sorted(o for g in groups for o in g["outputs"])
        assert all_outputs == list(range(1, 9))

    def test_mixed_io_group_count(self, board_stm32h7_mixed_io):
        timers, channels = _parse(board_stm32h7_mixed_io)
        groups = fcdg.compute_groups(timers, channels)
        assert len(groups) == 3

    def test_mixed_io_timer12_not_dshot(self, board_stm32h7_mixed_io):
        timers, channels = _parse(board_stm32h7_mixed_io)
        groups = fcdg.compute_groups(timers, channels)
        timer12_group = next(g for g in groups if "Timer12" in g["timer"])
        assert timer12_group["dshot"] is False
        assert timer12_group["non_dshot_outputs"] == timer12_group["outputs"]

    def test_mixed_io_dshot_timers(self, board_stm32h7_mixed_io):
        timers, channels = _parse(board_stm32h7_mixed_io)
        groups = fcdg.compute_groups(timers, channels)
        dshot_groups = [g for g in groups if g["dshot"]]
        assert len(dshot_groups) == 2

    def test_imxrt_groups_by_pwm_prefix(self, board_imxrt):
        timers, channels = _parse(board_imxrt)
        groups = fcdg.compute_groups(timers, channels)
        timer_names = {g["timer"] for g in groups}
        assert "PWM2" in timer_names
        assert "PWM4" in timer_names

    def test_imxrt_total_outputs(self, board_imxrt):
        timers, channels = _parse(board_imxrt)
        groups = fcdg.compute_groups(timers, channels)
        all_outputs = sorted(o for g in groups for o in g["outputs"])
        assert all_outputs == list(range(1, 9))

    def test_empty_input(self):
        groups = fcdg.compute_groups([], [])
        assert groups == []

    def test_single_channel(self):
        timers = [{"timer": "Timer1", "dshot": True}]
        channels = [{"output_index": 1, "timer": "Timer1", "channel": "Channel1",
                     "channel_index": 0, "is_dshot_channel": True}]
        groups = fcdg.compute_groups(timers, channels)
        assert len(groups) == 1
        assert groups[0]["outputs"] == [1]
        assert groups[0]["dshot"] is True


# ---------------------------------------------------------------------------
# compute_bdshot
# ---------------------------------------------------------------------------

class TestComputeBdshot:
    def test_stm32h7_all_dshot_full_bdshot(self, board_stm32h7_all_dshot):
        timers, channels = _parse(board_stm32h7_all_dshot)
        groups = fcdg.compute_groups(timers, channels)
        groups = fcdg.compute_bdshot(groups, channels, "stm32h7")
        for g in groups:
            assert g["bdshot"] is True
            assert set(g["bdshot_outputs"]) == set(g["outputs"])
            assert g["bdshot_output_only"] == []

    def test_stm32h7_timer4_ch4_bdshot_output_only(self):
        # Timer4, channel indices 0,1,2 → full bdshot; index 3 (CH4) → output_only
        # output_index is 1-based (matching real parse_timer_config behaviour)
        timers = [{"timer": "Timer4", "dshot": True}]
        channels = [
            {"output_index": 1, "timer": "Timer4", "channel": "Channel1", "channel_index": 0, "is_dshot_channel": True},
            {"output_index": 2, "timer": "Timer4", "channel": "Channel2", "channel_index": 1, "is_dshot_channel": True},
            {"output_index": 3, "timer": "Timer4", "channel": "Channel3", "channel_index": 2, "is_dshot_channel": True},
            {"output_index": 4, "timer": "Timer4", "channel": "Channel4", "channel_index": 3, "is_dshot_channel": True},
        ]
        groups = fcdg.compute_groups(timers, channels)
        groups = fcdg.compute_bdshot(groups, channels, "stm32h7")
        g = groups[0]
        assert g["bdshot"] is True
        assert 1 in g["bdshot_outputs"]  # output 1 = channel_index 0 → full
        assert 4 in g["bdshot_output_only"]  # output 4 = channel_index 3 → output-only

    def test_stm32f4_no_bdshot(self, board_stm32f4):
        timers, channels = _parse(board_stm32f4)
        groups = fcdg.compute_groups(timers, channels)
        groups = fcdg.compute_bdshot(groups, channels, "stm32f4")
        assert all(not g["bdshot"] for g in groups)
        assert all(g["bdshot_outputs"] == [] for g in groups)

    def test_imxrt_all_bdshot(self, board_imxrt):
        timers, channels = _parse(board_imxrt)
        groups = fcdg.compute_groups(timers, channels)
        groups = fcdg.compute_bdshot(groups, channels, "imxrt")
        dshot_groups = [g for g in groups if g["dshot"]]
        for g in dshot_groups:
            assert g["bdshot"] is True
            assert set(g["bdshot_outputs"]) == set(g["dshot_outputs"])
            assert g["bdshot_output_only"] == []

    def test_no_dshot_group_has_no_bdshot(self, board_stm32h7_mixed_io):
        timers, channels = _parse(board_stm32h7_mixed_io)
        groups = fcdg.compute_groups(timers, channels)
        groups = fcdg.compute_bdshot(groups, channels, "stm32h7")
        no_dshot = [g for g in groups if not g["dshot"]]
        for g in no_dshot:
            assert g["bdshot"] is False
            assert g["bdshot_outputs"] == []
            assert g["bdshot_output_only"] == []

    def test_unknown_family_no_full_bdshot(self):
        # For unknown family, timer is not in cap_map → falls to else branch:
        # bdshot_outputs=[], bdshot_output_only=group["outputs"] (output-only DShot)
        timers = [{"timer": "Timer1", "dshot": True}]
        channels = [{"output_index": 1, "timer": "Timer1", "channel": "Channel1",
                     "channel_index": 0, "is_dshot_channel": True}]
        groups = fcdg.compute_groups(timers, channels)
        groups = fcdg.compute_bdshot(groups, channels, "unknown")
        assert all(g["bdshot_outputs"] == [] for g in groups)

    def test_capture_channels_not_in_groups(self, board_stm32h7_capture_channels):
        # initIOTimerChannelCapture entries are excluded from channels by the parser,
        # so Timer1/Timer8/Timer12 (capture-only timers) produce no groups at all.
        timers, channels = _parse(board_stm32h7_capture_channels)
        groups = fcdg.compute_groups(timers, channels)
        group_timers = {g["timer"] for g in groups}
        assert group_timers.isdisjoint({"Timer1", "Timer8", "Timer12"})


class TestComputeGroupsCapture:
    def test_capture_group_count(self, board_stm32h7_capture_channels):
        # Capture channels excluded → only Timer5 and Timer4 remain
        timers, channels = _parse(board_stm32h7_capture_channels)
        groups = fcdg.compute_groups(timers, channels)
        assert len(groups) == 2

    def test_capture_total_outputs(self, board_stm32h7_capture_channels):
        # 8 regular outputs (Timer5: 1–4, Timer4: 5–8); capture entries skipped
        timers, channels = _parse(board_stm32h7_capture_channels)
        groups = fcdg.compute_groups(timers, channels)
        all_outputs = sorted(o for g in groups for o in g["outputs"])
        assert all_outputs == list(range(1, 9))

    def test_capture_dshot_only_on_dma_timers(self, board_stm32h7_capture_channels):
        timers, channels = _parse(board_stm32h7_capture_channels)
        groups = fcdg.compute_groups(timers, channels)
        # Both remaining groups (Timer5, Timer4) have DMA → dshot=True
        dshot_groups = {g["timer"] for g in groups if g["dshot"]}
        assert dshot_groups == {"Timer5", "Timer4"}
        assert all(g["dshot"] for g in groups)

    def test_capture_timer_group_memberships(self, board_stm32h7_capture_channels):
        timers, channels = _parse(board_stm32h7_capture_channels)
        groups = fcdg.compute_groups(timers, channels)
        by_timer = {g["timer"]: g["outputs"] for g in groups}
        assert by_timer["Timer5"] == [1, 2, 3, 4]
        assert by_timer["Timer4"] == [5, 6, 7, 8]
        # Capture-only timers must not appear as groups
        assert "Timer1" not in by_timer
        assert "Timer8" not in by_timer
        assert "Timer12" not in by_timer
