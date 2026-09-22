"""Exercise memory accounting against ARM ELFs produced by GNU binutils."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from firmware_size import format_change, memory_usage, summarize


class FirmwareSizeTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.linker = self.root / "firmware.ld"
        self.linker.write_text("""
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08008000, LENGTH = 2016K
    ITCM (rwx) : ORIGIN = 0, LENGTH = 16K
    RAM (rw) : ORIGIN = 0x20020000, LENGTH = 368K
    SRAM4 (rw) : ORIGIN = 0x38000000, LENGTH = 64K
}
SECTIONS
{
    .text : { *(.text) } > FLASH
    .ramfunc : { *(.ramfunc) } > ITCM AT > FLASH
    .data : { *(.data) } > RAM AT > FLASH
    .bss (NOLOAD) : { *(.bss) } > RAM
    .noinit (NOLOAD) : { *(.noinit) } > SRAM4
    .debug_info 0 : { *(.debug_info) }
}
""")

    def build(self, name="firmware", *, text=32, data=16, bss=32,
              ramfunc=16, reserved=16, debug=16, linker_flags=()):
        source = self.root / f"{name}.s"
        obj = source.with_suffix(".o")
        elf = source.with_suffix(".elf")
        source.write_text(f"""
.section .text,"ax"
.space {text}, 1
.section .ramfunc,"ax"
.space {ramfunc}, 2
.section .data,"aw"
.space {data}, 3
.section .bss,"aw",%nobits
.space {bss}
.section .noinit,"aw",%nobits
.space {reserved}
.section .debug_info,"",%progbits
.space {debug}, 4
""")
        subprocess.run(["arm-none-eabi-as", str(source), "-o", str(obj)], check=True)
        subprocess.run([
            "arm-none-eabi-ld", *linker_flags, "-T", str(self.linker), str(obj), "-o", str(elf)
        ], check=True)
        return elf

    def usage(self, **kwargs):
        elf = self.build(**kwargs)
        usage = memory_usage(elf)
        binary = elf.with_suffix(".bin")
        subprocess.run([
            "arm-none-eabi-objcopy", "-O", "binary", str(elf), str(binary)
        ], check=True)
        self.assertEqual(usage["flash"], binary.stat().st_size)
        return usage

    def test_bss_and_reserved_buffers_only_use_ram(self):
        before = self.usage()
        self.assertEqual(before, {"flash": 64, "ram": 80})
        for change in ({"bss": 96}, {"reserved": 80}):
            with self.subTest(change=change):
                after = self.usage(**change)
                self.assertEqual(after, {"flash": 64, "ram": 144})

    def test_initialized_data_and_ram_code_use_both(self):
        for change in ({"data": 32}, {"ramfunc": 32}):
            with self.subTest(change=change):
                self.assertEqual(self.usage(**change), {"flash": 80, "ram": 96})

    def test_debug_information_uses_neither(self):
        self.assertEqual(self.usage(), self.usage(debug=4096))

    def test_flash_includes_alignment_between_load_segments(self):
        self.linker.write_text(self.linker.read_text().replace(
            ".ramfunc :", ".ramfunc : ALIGN(32)"
        ))
        self.assertEqual(
            self.usage(text=33), {"flash": 96, "ram": 80}
        )

    def test_elf_header_in_first_load_segment(self):
        # A page size larger than the flash origin's alignment makes ld map the
        # ELF header into the first LOAD segment, below the flash origin.
        self.assertEqual(
            self.usage(linker_flags=("-z", "max-page-size=0x10000")),
            {"flash": 64, "ram": 80},
        )

    def test_opposing_changes_do_not_cancel_comment(self):
        result = summarize({"flash": 1024, "ram": 1024}, {"flash": 1088, "ram": 960})
        self.assertTrue(result["changed"])
        self.assertEqual(result["flash"], "+64 B (+6.25%)")
        self.assertEqual(result["ram"], "-64 B (-6.25%)")
        self.assertFalse(summarize({"flash": 1, "ram": 0}, {"flash": 1, "ram": 0})["changed"])
        self.assertEqual(format_change(0, 4488), "🔴 +4,488 B (n/a)")

    def test_small_deltas_are_not_reported(self):
        before = {"flash": 1000, "ram": 1000}
        self.assertFalse(summarize(before, {"flash": 992, "ram": 1029})["changed"])
        self.assertTrue(summarize(before, {"flash": 970, "ram": 1000})["changed"])

    def test_change_indicator(self):
        for delta, expected in ((1001, "🔴 "), (1000, "🟡 "), (101, "🟡 "), (100, ""),
                                (-100, ""), (-101, "🟢 "), (-5000, "🟢 ")):
            with self.subTest(delta=delta):
                self.assertTrue(format_change(10000, 10000 + delta).startswith(
                    f"{expected}{delta:+,} B"))

    def test_cli_from_outside_checkout(self):
        before = self.build("before")
        after = self.build("after", bss=96)
        output = subprocess.check_output([
            sys.executable, str(Path(__file__).with_name("firmware_size.py").resolve()),
            "--before", str(before), "--after", str(after),
        ], cwd=self.root, text=True)
        self.assertEqual(json.loads(output), {
            "flash": "+0 B (+0.00%)", "ram": "+64 B (+80.00%)", "changed": True,
        })


if __name__ == "__main__":
    unittest.main()
