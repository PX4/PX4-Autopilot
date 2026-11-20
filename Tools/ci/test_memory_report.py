"""Regression tests for honest comparisons of different RAM measurements."""

import copy
import unittest

from memory_report import compare_snapshots, render_details_summary, render_markdown


def snapshot():
    return {
        "schema_version": 1, "target": "test-arm", "revision": "abc", "toolchain": "test-gcc",
        "manifest": [
            {"name": "Module", "label": "Module", "allocation": "Startup allocation"},
            {"name": "Member", "label": "Member", "allocation": "Embedded", "included_in": "Module"},
        ],
        "static": {"status": "ok", "reason": None, "flash": 1000, "ram": 700},
        "objects": {"status": "ok", "reason": None, "types": {
            "Module": {"status": "ok", "bytes": 100, "reason": None},
            "Member": {"status": "ok", "bytes": 20, "reason": None},
        }},
        "stack": {"status": "ok", "reason": None, "frames": [
            {"source": "a.cpp", "function": "void f()", "bytes": 96, "kind": "static", "line": 1},
        ], "ambiguities": [], "coverage": {"files": 1, "functions": 1}},
    }


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.before = snapshot()
        self.after = copy.deepcopy(self.before)
        self.after["revision"] = "def"

    def report(self):
        return compare_snapshots(self.before, self.after)

    def test_identical_data_has_no_change(self):
        self.assertFalse(self.report()["changed"])
        self.assertEqual(self.report()["stack"]["rows"], [])

    def test_unchanged_unknowns_are_not_pr_changes(self):
        self.before["stack"]["frames"][0]["kind"] = "dynamic_unbounded"
        self.before["stack"]["ambiguities"] = [{
            "source": "header.h", "function": "inline f()", "reason": "Conflicting records",
            "variants": [{"bytes": 8, "kind": "static", "line": 1},
                         {"bytes": 16, "kind": "static", "line": 1}]}]
        self.after = copy.deepcopy(self.before)
        self.after["stack"]["frames"][0]["line"] = 50
        self.after["stack"]["ambiguities"][0]["variants"].reverse()
        self.after["stack"]["ambiguities"][0]["variants"][0]["line"] = 99
        report = self.report()
        self.assertFalse(report["changed"])
        self.assertEqual(report["stack"]["rows"], [])
        self.assertNotIn("inline f()", render_markdown(report))
        self.assertIn("0 stack changes", render_details_summary(report))

    def test_changed_ambiguous_observations_remain_visible(self):
        self.before["stack"]["ambiguities"] = [{
            "source": "header.h", "function": "inline changed()", "reason": "Conflicting records",
            "variants": [{"bytes": 8, "kind": "static"}, {"bytes": 16, "kind": "static"}]}]
        self.after = copy.deepcopy(self.before)
        self.after["stack"]["ambiguities"][0]["variants"][1]["bytes"] = 32
        report = self.report()
        self.assertTrue(report["changed"])
        self.assertEqual(len(report["stack"]["rows"]), 1)
        self.assertTrue(report["stack"]["rows"][0]["measurement_changed"])
        self.assertIn("inline changed()", render_markdown(report))
        self.assertIn("⚠️ unknown bounds", render_details_summary(report))

    def test_member_growth_is_visible_without_static_ram_growth(self):
        for entry in self.after["objects"]["types"].values():
            entry["bytes"] += 512
        report = self.report()
        self.assertTrue(report["changed"])
        self.assertEqual(report["static"]["ram"]["delta"], 0)
        self.assertEqual([row["delta"] for row in report["objects"]["rows"]], [512, 512])
        self.assertEqual(report["objects"]["rows"][1]["included_in"], "Module")
        self.assertNotIn("total", report["objects"])
        self.assertIn("do not add these rows", render_markdown(report))

    def test_vla_removal_is_not_a_numeric_stack_saving(self):
        self.before["stack"]["frames"][0]["kind"] = "dynamic_unbounded"
        self.after["stack"]["frames"][0]["bytes"] = 80
        row = self.report()["stack"]["rows"][0]
        self.assertEqual(row["status"], "changed")
        self.assertIsNone(row["delta"])
        text = render_markdown(self.report())
        self.assertIn("96 B + dynamic ⚠️", text)
        self.assertNotIn("-16 B", text)

    def test_bounded_dynamic_frame_can_be_compared(self):
        self.before["stack"]["frames"][0].update(kind="dynamic_bounded", bytes=128)
        self.assertEqual(self.report()["stack"]["rows"][0]["delta"], -32)

    def test_line_shift_does_not_invent_added_function(self):
        self.after["stack"]["frames"][0]["line"] = 300
        self.assertFalse(self.report()["changed"])

    def test_missing_type_is_not_zero(self):
        self.after["objects"]["types"]["Module"] = {
            "status": "not_found", "bytes": None, "reason": "No type definition"}
        row = self.report()["objects"]["rows"][0]
        self.assertEqual(row["status"], "unavailable")
        self.assertIsNone(row["after"])
        self.assertIsNone(row["delta"])
        self.assertIn("No type definition", render_markdown(self.report()))

    def test_missing_stack_data_is_not_a_list_of_removals(self):
        self.after["stack"].update(status="unavailable", reason="No .su files", frames=[])
        report = self.report()
        self.assertTrue(report["changed"])
        self.assertEqual(report["stack"]["rows"], [])
        self.assertIn("No .su files", render_markdown(report))

    def test_conflicting_frames_are_explicitly_ambiguous(self):
        self.after["stack"]["frames"] = []
        self.after["stack"]["ambiguities"] = [{"source": "a.cpp", "function": "void f()", "variants": []}]
        row = self.report()["stack"]["rows"][0]
        self.assertEqual(row["status"], "ambiguous")
        self.assertIsNone(row["delta"])

    def test_added_frame_has_no_zero_baseline(self):
        self.after["stack"]["frames"].append({"source": "a.cpp", "function": "void g()", "bytes": 64, "kind": "static"})
        row = self.report()["stack"]["rows"][0]
        self.assertEqual(row["status"], "added")
        self.assertIsNone(row["delta"])

    def test_different_targets_toolchains_or_manifests_are_rejected(self):
        for key in ("target", "toolchain", "manifest", "schema_version"):
            with self.subTest(key=key):
                other = copy.deepcopy(self.after)
                other[key] = "different"
                with self.assertRaisesRegex(ValueError, key):
                    compare_snapshots(self.before, other)

    def test_static_failure_is_not_zero_usage(self):
        self.after["static"] = {"status": "unavailable", "reason": "Bad ELF"}
        self.assertIsNone(self.report()["static"]["ram"])
        self.assertTrue(self.report()["changed"])

    def test_markdown_escapes_names_and_limits_rows(self):
        self.after["stack"]["frames"] = [
            {"source": "<source>|.cpp", "function": f"f{i}`\n()", "bytes": i, "kind": "static"}
            for i in range(20)]
        text = render_markdown(self.report(), limit=2)
        self.assertIn("Showing 2 of 21", text)
        self.assertNotIn("<source>", text)
        self.assertIn("&#124;", text)
        self.assertIn("&#96;", text)

    def test_new_unbounded_frame_is_visible_before_large_known_deltas(self):
        self.after["stack"]["frames"][0]["bytes"] = 4096
        self.after["stack"]["frames"].append({
            "source": "a.cpp", "function": "void vla()", "bytes": 32, "kind": "dynamic_unbounded"})
        text = render_markdown(self.report(), limit=1)
        self.assertIn("void vla()", text)
        self.assertIn("32 B + dynamic ⚠️", text)
        self.assertNotIn("void f()", text)

    def test_delta_warning_boundaries_match_existing_bot(self):
        for delta, icon in ((100, ""), (101, "🟡"), (1000, "🟡"), (1001, "🔴"), (-100, ""), (-101, "🟢")):
            with self.subTest(delta=delta):
                # Values are deliberately large enough to keep decreases valid.
                self.before["stack"]["frames"][0]["bytes"] = 4096
                self.after["stack"]["frames"][0]["bytes"] = 4096 + delta
                report = self.report()
                expected = f"{icon + ' ' if icon else ''}{delta:+,} B"
                self.assertIn(expected, render_markdown(report))
                heading = render_details_summary(report)
                self.assertEqual([mark for mark in ('🟡', '🔴', '🟢') if mark in heading], [icon] if icon else [])

    def test_summary_does_not_hide_growth_or_double_count_members(self):
        report = self.report()
        report["objects"]["rows"][0].update(delta=512, status="changed")
        report["objects"]["rows"][1].update(delta=512, status="changed")
        heading = render_details_summary(report)
        self.assertIn("🟡 1 module object changed (largest +512 B)", heading)
        report["objects"]["rows"].append({"status": "changed", "delta": -4096})
        heading = render_details_summary(report)
        self.assertIn("🟡 2 module objects changed (largest +512 B)", heading)
        self.assertNotIn("🟢", heading)

    def test_comment_shortens_long_names_but_keeps_full_evidence(self):
        name = "template_function<" + "LongType," * 1000 + ">()"
        self.after["stack"]["frames"] = [{"source": "long.cpp", "function": name, "bytes": 64, "kind": "static"}]
        report = self.report()
        self.assertEqual(report["stack"]["rows"][1]["function"], name)
        text = render_markdown(report)
        self.assertNotIn(name, text)
        self.assertIn("…", text)
        self.assertLess(len(text.encode()), 3000)


if __name__ == "__main__":
    unittest.main()
