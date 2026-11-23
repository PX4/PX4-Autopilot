"""Check GCC qualifier semantics, ambiguous records, and real compiler output."""

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from stack_frames import collect_stack_frames


class StackFramesTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def write_records(self, records, filename="module.cpp.su"):
        path = self.root / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(records, encoding="utf-8")

    def test_qualifiers_and_cpp_signatures(self):
        signature = "void ns::Planner::visit(T&) [with T = matrix::Vector<float, 2>]"
        self.write_records(
            f"/checkout/src/planner.cpp:100:2:{signature}\t96\tstatic\n"
            "/checkout/src/planner.cpp:150:2:void bounded()\t128\tdynamic,bounded\n"
            "/checkout/src/planner.cpp:200:2:void variable(unsigned int)\t32\tdynamic\n"
        )
        result = collect_stack_frames(self.root, "/checkout")
        self.assertEqual(result["status"], "ok")
        frames = {frame["function"]: frame for frame in result["frames"]}
        self.assertEqual(frames[signature], {
            "source": "src/planner.cpp", "function": signature,
            "bytes": 96, "kind": "static", "line": 100,
        })
        self.assertEqual(frames["void bounded()"]["kind"], "dynamic_bounded")
        self.assertEqual(frames["void bounded()"]["bytes"], 128)
        self.assertEqual(frames["void variable(unsigned int)"]["kind"], "dynamic_unbounded")
        self.assertEqual(result["coverage"]["dynamic_unbounded"], 1)

    def test_duplicate_identity_does_not_include_line(self):
        self.write_records("src/a.cpp:2:1:int f()\t16\tstatic\n")
        self.write_records("src/a.cpp:22:1:int f()\t16\tstatic\n", "other/a.su")
        result = collect_stack_frames(self.root)
        self.assertEqual(len(result["frames"]), 1)
        self.assertEqual(result["frames"][0]["line"], 2)
        self.assertEqual(result["coverage"]["records"], 2)
        self.assertEqual(result["coverage"]["functions"], 1)

    def test_conflicting_records_are_explicitly_unknown(self):
        self.write_records(
            "src/a.hpp:2:1:int f()\t16\tstatic\n"
            "src/a.hpp:2:1:int f()\t32\tstatic\n"
            "src/a.hpp:2:1:int f()\t32\tdynamic\n"
            "src/b.cpp:2:1:int f()\t16\tstatic\n"
        )
        result = collect_stack_frames(self.root)
        self.assertEqual(result["status"], "ok")
        self.assertEqual([frame["source"] for frame in result["frames"]], ["src/b.cpp"])
        self.assertEqual(result["coverage"]["ambiguous_functions"], 1)
        self.assertEqual(result["coverage"]["functions"], 2)
        variants = result["ambiguities"][0]["variants"]
        self.assertEqual({(frame["bytes"], frame["kind"]) for frame in variants}, {
            (16, "static"), (32, "static"), (32, "dynamic_unbounded"),
        })

    def test_relative_and_external_paths_are_not_guessed(self):
        self.write_records(
            "../../src/module.cpp:2:1:int relative()\t16\tstatic\n"
            "/toolchain/include/header.hpp:3:1:int external()\t8\tstatic\n"
        )
        result = collect_stack_frames(self.root, "/checkout")
        self.assertEqual({frame["source"] for frame in result["frames"]}, {
            "../../src/module.cpp", "/toolchain/include/header.hpp",
        })

    def test_incomplete_gcc_names_never_become_comparable(self):
        # Real GCC 13 release builds can lose names of variadic functions and
        # optimized clones. Equal byte counts do not make those identities safe.
        self.write_records(
            "src/log.cpp:206:15:)\t12\tstatic\n"
            "src/log.cpp:215:15:)\t12\tstatic\n"
            "src/filter.cpp:100:1:0(const Sample&)\t80\tstatic\n"
        )
        result = collect_stack_frames(self.root)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["frames"], [])
        self.assertEqual(result["coverage"]["ambiguous_functions"], 2)
        self.assertTrue(all("incomplete function name" in row["reason"]
                            for row in result["ambiguities"]))

    def test_missing_or_empty_records_are_unavailable(self):
        self.assertEqual(collect_stack_frames(self.root / "missing")["status"], "unavailable")
        self.assertEqual(collect_stack_frames(self.root)["status"], "unavailable")
        self.write_records("\n")
        result = collect_stack_frames(self.root)
        self.assertEqual(result["status"], "unavailable")
        self.assertEqual(result["coverage"]["files"], 1)

    def test_invalid_records_invalidate_the_collection(self):
        for record in (
            "src/a.cpp:2:1:int f() 16 static",
            "int f()\t16\tstatic",
            "src/a.cpp:2:1:int f()\t-1\tstatic",
            "src/a.cpp:2:1:int f()\tNaN\tstatic",
            "src/a.cpp:2:1:int f()\t16\tbounded",
            "src/a.cpp:2:1:int f()\t16\tstatic,dynamic",
            "src/a.cpp:2:1:int f()\t16\tdynamic,dynamic",
        ):
            with self.subTest(record=record):
                self.write_records("src/ok.cpp:1:1:int ok()\t8\tstatic\n" + record)
                result = collect_stack_frames(self.root)
                self.assertEqual(result["status"], "unavailable")
                self.assertEqual(result["frames"], [])
                self.assertEqual(result["coverage"]["invalid_records"], 1)
                self.assertEqual(result["errors"][0]["line"], 2)

    def test_cli_reports_failure_as_json_and_nonzero_exit(self):
        self.write_records("invalid record\n")
        process = subprocess.run([
            sys.executable, str(Path(__file__).with_name("stack_frames.py")),
            "--root", str(self.root),
        ], check=False, capture_output=True, text=True)
        self.assertNotEqual(process.returncode, 0)
        self.assertEqual(json.loads(process.stdout)["status"], "unavailable")

    @unittest.skipUnless(shutil.which("gcc"), "native GCC is needed for the compiler fixture")
    def test_real_gcc_static_and_variable_frames(self):
        source = self.root / "fixture.c"
        source.write_text("""
__attribute__((noinline)) unsigned fixed(void)
{
    volatile unsigned char buffer[256];
    buffer[0] = 3;
    buffer[255] = 5;
    return buffer[0] + buffer[255];
}
__attribute__((noinline)) unsigned variable(unsigned size)
{
    volatile unsigned char buffer[size];
    buffer[0] = 3;
    buffer[size - 1] = 5;
    return buffer[0] + buffer[size - 1];
}
""", encoding="utf-8")
        subprocess.run([
            "gcc", "-O2", "-fstack-usage", "-c", str(source),
            "-o", str(source.with_suffix(".o")),
        ], check=True, capture_output=True, text=True)
        result = collect_stack_frames(self.root, self.root)
        self.assertEqual(result["status"], "ok")
        frames = {frame["function"]: frame for frame in result["frames"]}
        self.assertEqual(frames["fixed"]["kind"], "static")
        self.assertGreater(frames["fixed"]["bytes"], 0)
        self.assertEqual(frames["variable"]["kind"], "dynamic_unbounded")
        self.assertEqual(frames["variable"]["source"], "fixture.c")


if __name__ == "__main__":
    unittest.main()
