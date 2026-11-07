"""Check DWARF object sizes against compiled C++ fixtures, without flight hardware."""

import gzip
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from module_object_sizes import collect_object_sizes


@unittest.skipUnless(shutil.which("c++"), "A native C++ compiler is required")
class ModuleObjectSizesTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def build(self, sources, *, debug=True, extra_flags=()):
        paths = []
        for name, contents in sources.items():
            path = self.root / name
            path.write_text(contents)
            paths.append(str(path))
        elf = self.root / "fixture.elf"
        subprocess.run(["c++", "-std=c++14", "-O0", *(["-g"] if debug else []),
                        *extra_flags, *paths, "-o", str(elf)], check=True, capture_output=True)
        return elf

    def test_target_sizes_include_inline_members_but_not_pointed_to_storage(self):
        elf = self.build({"fixture.cpp": """
#include <cstdio>
namespace flight {
struct Planner { unsigned char scratch[512]; };
struct Module {
    Planner planner;
    unsigned char *external;
    struct Nested { int value; };
    Nested nested;
};
}
namespace other { struct Module { unsigned char scratch[17]; }; }
flight::Module module;
other::Module other_module;
unsigned char external[8192];
int main() {
    module.external = external;
    std::printf("%zu %zu %zu %zu\\n", sizeof(flight::Module), sizeof(flight::Planner),
                sizeof(flight::Module::Nested), sizeof(other::Module));
}
"""})
        names = ["flight::Module", "flight::Planner", "flight::Module::Nested", "other::Module"]
        expected = [int(size) for size in subprocess.check_output([str(elf)], text=True).split()]
        report = collect_object_sizes(elf, names)
        self.assertEqual(report["status"], "ok")
        for name, size in zip(names, expected):
            self.assertEqual(report["types"][name], {"status": "ok", "bytes": size, "reason": None})
        self.assertEqual(report["types"]["flight::Planner"]["bytes"], 512)
        self.assertLess(report["types"]["flight::Module"]["bytes"], 8192)

    def test_duplicate_definitions_across_compilation_units_are_not_added(self):
        elf = self.build({
            "one.cpp": "struct Module { char buffer[512]; }; Module one; int main() { return 0; }",
            "two.cpp": "struct Module { char buffer[512]; }; Module two;",
        })
        report = collect_object_sizes(elf, ["Module", "Module"])
        self.assertEqual(report["types"], {"Module": {"status": "ok", "bytes": 512, "reason": None}})

    def test_conflicting_definitions_are_ambiguous(self):
        # Deliberately inconsistent TUs simulate stale/mixed build information.
        # The linked fixture is never executed.
        elf = self.build({
            "one.cpp": "struct Module { char buffer[16]; }; Module one; int main() { return 0; }",
            "two.cpp": "struct Module { char buffer[32]; }; Module two;",
        })
        entry = collect_object_sizes(elf, ["Module"])["types"]["Module"]
        self.assertEqual(entry["status"], "ambiguous")
        self.assertIsNone(entry["bytes"])
        self.assertIn("16, 32", entry["reason"])

    def test_forward_declaration_does_not_override_a_definition(self):
        elf = self.build({
            "one.cpp": "struct Module; Module *pointer; int main() { return 0; }",
            "two.cpp": "struct Module { char buffer[512]; }; Module object;",
        })
        self.assertEqual(collect_object_sizes(elf, ["Module"])["types"]["Module"]["bytes"], 512)

    def test_missing_and_incomplete_types_have_no_size(self):
        elf = self.build({"fixture.cpp": "struct Forward; Forward *pointer; int main() { return 0; }"})
        report = collect_object_sizes(elf, ["Missing", "Forward"])
        self.assertEqual(report["status"], "ok")
        for entry in report["types"].values():
            self.assertEqual(entry["status"], "not_found")
            self.assertIsNone(entry["bytes"])

    def test_local_type_is_not_mistaken_for_a_global_module(self):
        elf = self.build({"fixture.cpp": """
int main() { struct Module { char data[512]; }; Module local{}; return local.data[0]; }
"""})
        entry = collect_object_sizes(elf, ["Module"])["types"]["Module"]
        self.assertEqual(entry["status"], "not_found")
        self.assertIsNone(entry["bytes"])

    def test_out_of_line_nested_definition_keeps_its_declared_scope(self):
        elf = self.build({"fixture.cpp": """
namespace flight { struct Module { struct Nested; }; }
struct flight::Module::Nested { char data[64]; };
flight::Module::Nested nested;
int main() { return 0; }
"""})
        entry = collect_object_sizes(elf, ["flight::Module::Nested"])["types"]["flight::Module::Nested"]
        self.assertEqual(entry, {"status": "ok", "bytes": 64, "reason": None})

    def test_separate_dwarf_type_units(self):
        elf = self.build({"fixture.cpp": "struct Module { char buffer[512]; }; Module object; int main() { return 0; }"},
                         extra_flags=("-gdwarf-4", "-fdebug-types-section"))
        self.assertEqual(collect_object_sizes(elf, ["Module"])["types"]["Module"]["bytes"], 512)

    def test_no_debug_information_is_unavailable_not_zero(self):
        elf = self.build({"fixture.cpp": "int main() { return 0; }"}, debug=False)
        report = collect_object_sizes(elf, ["Module"])
        self.assertEqual(report["status"], "unavailable")
        self.assertIn(".debug_info", report["reason"])
        self.assertIsNone(report["types"]["Module"]["bytes"])

    def test_gzip_and_cli_preserve_json_schema(self):
        elf = self.build({"fixture.cpp": "struct Module { char buffer[512]; }; Module object; int main() { return 0; }"})
        compressed = elf.with_suffix(".elf.gz")
        with gzip.open(compressed, "wb") as output:
            output.write(elf.read_bytes())
        expected = collect_object_sizes(elf, ["Module"])
        self.assertEqual(collect_object_sizes(compressed, ["Module"]), expected)
        cli = subprocess.check_output([sys.executable, str(Path(__file__).with_name("module_object_sizes.py")),
                                       "--elf", str(compressed), "--types", "Module"], text=True)
        self.assertEqual(json.loads(cli), expected)

    def test_invalid_or_missing_elf_has_no_numeric_size(self):
        invalid = self.root / "invalid.elf"
        invalid.write_text("This is not an ELF")
        for path in (invalid, self.root / "missing.elf"):
            with self.subTest(path=path):
                report = collect_object_sizes(path, ["Module"])
                self.assertEqual(report["status"], "unavailable")
                self.assertIsNone(report["types"]["Module"]["bytes"])


if __name__ == "__main__":
    unittest.main()
