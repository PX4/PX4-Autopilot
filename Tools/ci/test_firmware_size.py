"""Exercise memory accounting against ARM ELFs produced by GNU binutils."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from firmware_size import format_change, memory_capacity, memory_usage, summarize

CAPACITY = {"flash": 1008 * 1024, "ram": 256 * 1024}

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
        self.assertEqual(usage.pop("image_start"), 0x08008000)
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

    def test_capacity_from_linker_regions(self):
        # The flash alias is neither the image's region nor RAM.
        self.linker.write_text(self.linker.read_text().replace(
            "MEMORY\n{", "MEMORY\n{\n    FLASH_ITCM (rx) : ORIGIN = 0x00208000, LENGTH = 2016K /* alias */"
        ))
        self.assertEqual(memory_capacity(self.linker, 0x08008000),
                         {"flash": 2016 * 1024, "ram": (16 + 368 + 64) * 1024})

    def test_capacity_fails_loudly(self):
        with self.assertRaisesRegex(ValueError, "no single region"):
            memory_capacity(self.linker, 0x09000000)
        self.linker.write_text(self.linker.read_text().replace("LENGTH = 2016K", "LENGTH = 2M - 32K"))
        with self.assertRaisesRegex(ValueError, "cannot read"):
            memory_capacity(self.linker, 0x08008000)

    def test_opposing_changes_do_not_cancel_comment(self):
        result = summarize({"flash": 1024, "ram": 1024}, {"flash": 1088, "ram": 960}, CAPACITY)
        self.assertTrue(result["changed"])
        self.assertEqual(result["flash"], "+64 B (+6.25%)")
        self.assertEqual(result["ram"], "-64 B (-6.25%)")
        self.assertFalse(summarize({"flash": 1, "ram": 0}, {"flash": 1, "ram": 0}, CAPACITY)["changed"])
        self.assertEqual(format_change(0, 4488), "🔴 +4,488 B (n/a)")

    def test_usage_is_the_change_against_capacity(self):
        result = summarize({"flash": 1024, "ram": 1024}, {"flash": 1032192, "ram": 3840}, CAPACITY)
        self.assertEqual(result["flash_used"], "100.00%")
        self.assertEqual(result["ram_used"], "1.46%")

    def test_small_deltas_are_not_reported(self):
        before = {"flash": 1000, "ram": 1000}
        self.assertFalse(summarize(before, {"flash": 992, "ram": 1029}, CAPACITY)["changed"])
        self.assertTrue(summarize(before, {"flash": 970, "ram": 1000}, CAPACITY)["changed"])

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
            "--before", str(before), "--after", str(after), "--linker-script", str(self.linker),
        ], cwd=self.root, text=True)
        self.assertEqual(json.loads(output), {
            "flash": "+0 B (+0.00%)", "flash_used": "0.00%",
            "ram": "+64 B (+80.00%)", "ram_used": "0.03%", "changed": True,
        })


if __name__ == "__main__":
    unittest.main()
