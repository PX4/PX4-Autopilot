"""Check the report's evidence labels, ownership and safe HTML rendering."""

from copy import deepcopy
from html.parser import HTMLParser
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from memory_report_html import render_html


class ParsedReport(HTMLParser):
    """Small DOM view sufficient to inspect rendered cells and attributes."""

    VOID = {"area", "base", "br", "col", "embed", "hr", "img", "input", "link", "meta", "param", "source", "track", "wbr"}

    def __init__(self, html):
        super().__init__(convert_charrefs=True)
        self.nodes = []
        self.stack = []
        self.feed(html)

    def handle_starttag(self, tag, attrs):
        node = {"tag": tag, "attrs": dict(attrs), "text": "", "parent": self.stack[-1] if self.stack else None}
        self.nodes.append(node)
        if tag not in self.VOID:
            self.stack.append(node)

    def handle_endtag(self, tag):
        for index in range(len(self.stack) - 1, -1, -1):
            if self.stack[index]["tag"] == tag:
                del self.stack[index:]
                break

    def handle_data(self, data):
        for node in self.stack:
            node["text"] += data

    def find(self, tag=None, **attrs):
        return [node for node in self.nodes if (tag is None or node["tag"] == tag)
                and all(node["attrs"].get(key) == value for key, value in attrs.items())]


def example_report():
    return {
        "target": "example_board",
        "provenance": {
            "before": "a" * 40, "after": "b" * 40,
            "toolchain": "example compiler", "label": "Synthetic test fixture; not firmware measurements",
        },
        "static": {
            "status": "ok", "flash": {"before": 1000, "after": 984, "delta": -16},
            "ram": {"before": 100, "after": 100, "delta": 0},
        },
        "objects": {
            "status": "ok", "rows": [
                {"name": "Module", "label": "Example module", "allocation": "Heap at startup",
                 "included_in": None, "before": 200, "after": 712, "delta": 512, "status": "changed"},
                {"name": "Planner", "label": "Example planner", "allocation": "Embedded",
                 "included_in": "Module", "before": 100, "after": 612, "delta": 512, "status": "changed"},
            ],
        },
        "stack": {
            "status": "ok", "rows": [
                {"source": "source.cpp", "function": "grow()", "before": {"bytes": 32, "kind": "static"},
                 "after": {"bytes": 64, "kind": "static"}, "delta": 32, "status": "changed"},
                {"source": "source.cpp", "function": "same()", "before": {"bytes": 32, "kind": "static"},
                 "after": {"bytes": 32, "kind": "static"}, "delta": 0, "status": "unchanged"},
                {"source": "source.cpp", "function": "kind_changed()", "before": {"bytes": 32, "kind": "dynamic_bounded"},
                 "after": {"bytes": 32, "kind": "static"}, "delta": 0, "status": "changed"},
                {"source": "source.cpp", "function": "vla()", "before": {"bytes": 96, "kind": "dynamic_unbounded"},
                 "after": {"bytes": 80, "kind": "static"}, "delta": None, "status": "changed"},
            ], "coverage": {"before": {"files": 1}, "after": {"files": 1}},
        },
        "limitations": ["Synthetic fixture only."],
    }


class MemoryReportHtmlTest(unittest.TestCase):
    def setUp(self):
        self.report = example_report()

    def test_provenance_is_visible_and_modules_precede_function_list(self):
        document = ParsedReport(render_html(self.report))
        self.assertEqual(len(document.find("html", lang="en")), 1)
        metadata = [node["text"] for node in document.find("dd")]
        self.assertEqual(metadata, ["example_board", "a" * 40, "b" * 40, "example compiler"])
        self.assertIn(self.report["provenance"]["label"], document.find("header")[0]["text"])
        self.assertEqual([node["attrs"]["id"] for node in document.find("section")],
                         ["static", "objects", "stack", "interpretation"])
        self.assertIn("GitHub PR bot comment is the primary report", document.find("footer")[0]["text"])

    def test_untrusted_input_is_text_not_markup(self):
        payload = '\"><script>window.injection=true</script><img src="https://example.invalid/">&'
        self.report["target"] = payload
        self.report["provenance"]["label"] = payload
        self.report["stack"]["rows"][0]["function"] = payload
        self.report["objects"]["rows"][1]["included_in"] = payload
        self.report["limitations"] = [payload]
        document = ParsedReport(render_html(self.report))
        self.assertEqual(len(document.find("script")), 1)
        self.assertEqual(document.find("img"), [])
        self.assertIn(payload, document.find("header")[0]["text"])
        self.assertEqual(len(document.find("tr", **{"data-search": payload + " source.cpp"})), 1)
        self.assertIn(payload, document.find("section", id="objects")[0]["text"])

    def test_unbounded_stack_never_becomes_a_numeric_saving(self):
        # Defend the UI even if a producer incorrectly supplied a raw-byte delta.
        self.report["stack"]["rows"][-1]["delta"] = -16
        document = ParsedReport(render_html(self.report))
        row = document.find("tr", **{"data-search": "vla() source.cpp"})[0]
        self.assertEqual(row["attrs"]["data-status"], "warning")
        self.assertIn("Unknown upper bound", row["text"])
        self.assertIn("96 B", row["text"])
        self.assertIn("Recorded portion", row["text"])
        self.assertNotIn("-16 B", row["text"])

    def test_missing_measurements_are_unavailable_not_zero(self):
        document = ParsedReport(render_html({}))
        cards = document.find("a", **{"class": "card"})
        self.assertEqual(len(cards), 4)
        for card in cards:
            self.assertIn("Unavailable", card["text"])
            self.assertNotIn("+0", card["text"])
        report = deepcopy(self.report)
        report["objects"]["rows"] = [{"name": "Module", "status": "unavailable", "reason": "No DWARF definition"}]
        card = ParsedReport(render_html(report)).find("a", href="#objects")[0]
        self.assertIn("Unavailable", card["text"])
        self.assertNotIn("0 changed", card["text"])

    def test_embedded_ownership_is_explicit_and_not_added(self):
        document = ParsedReport(render_html(self.report))
        card = document.find("a", href="#objects")[0]
        self.assertIn("1 changed", card["text"])
        self.assertNotIn("2 changed", card["text"])
        objects = document.find("section", id="objects")[0]
        self.assertIn("Included in Module; do not add again.", objects["text"])
        self.assertIn("These rows are not summed.", objects["text"])
        self.assertNotIn("1,024 B", objects["text"])
        self.assertIn("Not runtime heap peak", objects["text"])

    def test_only_changes_are_rendered_including_kind_changes_and_unknowns(self):
        document = ParsedReport(render_html(self.report))
        select = document.find("select")[0]
        options = [node for node in document.find("option") if node["parent"] is select]
        self.assertEqual(options[0]["attrs"]["value"], "changes")
        self.assertEqual(options[0]["text"], "Changed functions")
        self.assertNotIn("all", [node["attrs"]["value"] for node in options])
        expected = ["grow() source.cpp", "kind_changed() source.cpp", "vla() source.cpp"]
        for search in expected:
            self.assertEqual(len(document.find("tr", **{"data-search": search})), 1)
        self.assertEqual(document.find("tr", **{"data-search": "same() source.cpp"}), [])
        self.assertIn("select.value === 'changes'", document.find("script")[0]["text"])

    def test_old_ambiguous_and_unchanged_unbounded_records_are_omitted(self):
        ambiguous = deepcopy(self.report["stack"]["rows"][0])
        ambiguous.update(status="ambiguous", delta=None, reason="Conflicting compiler records")
        unbounded = deepcopy(self.report["stack"]["rows"][-1])
        unbounded.update(status="unchanged", after=deepcopy(unbounded["before"]))
        self.report["stack"]["rows"] = [ambiguous, unbounded]
        document = ParsedReport(render_html(self.report))
        card = document.find("a", href="#stack")[0]
        self.assertIn("0 changed", card["text"])
        self.assertNotIn("unknown dynamic bound", card["text"])
        self.assertEqual(document.find("table", id="stack-table"), [])
        self.assertIn("No function stack-frame changes", document.find("section", id="stack")[0]["text"])

    def test_changed_ambiguous_record_requires_evidence_and_has_no_numeric_delta(self):
        row = deepcopy(self.report["stack"]["rows"][0])
        row.update(status="ambiguous", measurement_changed=True, reason="Conflicting compiler records")
        self.report["stack"]["rows"] = [row]
        document = ParsedReport(render_html(self.report))
        rendered = document.find("tr", **{"data-search": "grow() source.cpp"})[0]
        self.assertIn("Not comparable", rendered["text"])
        self.assertNotIn("+32 B", rendered["text"])
        self.assertIn("1 changed", document.find("a", href="#stack")[0]["text"])
        self.assertIn("1 ambiguous record", document.find("a", href="#stack")[0]["text"])

    def test_actual_changes_are_not_buried_under_ambiguous_header_records(self):
        ambiguous = deepcopy(self.report["stack"]["rows"][0])
        ambiguous.update(function="ambiguous()", source="a_header.h", status="ambiguous", delta=None,
                         measurement_changed=True)
        self.report["stack"]["rows"].insert(0, ambiguous)
        original_order = [row["function"] for row in self.report["stack"]["rows"]]
        document = ParsedReport(render_html(self.report))
        rows = [node for node in document.find("tr") if "data-search" in node["attrs"]]
        self.assertEqual([row["attrs"]["data-search"] for row in rows], [
            "vla() source.cpp", "grow() source.cpp", "kind_changed() source.cpp",
            "ambiguous() a_header.h",
        ])
        self.assertEqual([row["function"] for row in self.report["stack"]["rows"]], original_order)

    def test_threshold_colours_match_bot_without_changing_direction_filters(self):
        for delta, tone, direction in ((1001, "danger", "up"), (1000, "warning", "up"),
                                       (101, "warning", "up"), (100, "neutral", "up"),
                                       (-100, "neutral", "down"), (-101, "down", "down")):
            with self.subTest(delta=delta):
                row = {"source": "example.cpp", "function": "f()", "status": "changed",
                       "before": {"bytes": 2048, "kind": "static"},
                       "after": {"bytes": 2048 + delta, "kind": "static"}, "delta": delta}
                self.report["stack"]["rows"] = [row]
                document = ParsedReport(render_html(self.report))
                rendered = document.find("tr", **{"data-search": "f() example.cpp"})[0]
                self.assertEqual(rendered["attrs"]["data-status"], direction)
                badges = document.find("span", **{"class": "badge " + tone})
                self.assertTrue(any(f"{delta:+,} B" in badge["text"] for badge in badges))

    def test_standalone_output_has_no_external_assets(self):
        document = ParsedReport(render_html(self.report))
        self.assertEqual(document.find("link"), [])
        self.assertEqual(len(document.find("style")), 1)
        self.assertIn("@media print", document.find("style")[0]["text"])
        self.assertTrue(all("src" not in node["attrs"] for node in document.nodes))

    def test_cli_reads_json_and_writes_html_outside_checkout(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output = root / "report.json", root / "report.html"
            source.write_text(json.dumps(self.report), encoding="utf-8")
            subprocess.run([sys.executable, str(Path(__file__).with_name("memory_report_html.py").resolve()),
                            "--input", str(source), "--output", str(output)], cwd=root, check=True)
            document = ParsedReport(output.read_text(encoding="utf-8"))
            self.assertEqual(document.find("title")[0]["text"], "PX4 memory impact — example_board")


if __name__ == "__main__":
    unittest.main()
