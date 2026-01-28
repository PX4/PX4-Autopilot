#! /usr/bin/env python3

"""
Checks if the functions that should be mapped to ITCM are contained in the built ELF file.

This helps against linker scripts that "rot" as the linker does not warn in case of a function
not existing. Thus, it is possible to forget to update the linker script after a code update.

The tool uses the DWARF debug info and the ELF symbol table section to identify which functions
exist in the built ELF file.

It is expected that the linker scripts that are analyzed by the tool are linker script include
files that only contain the name of the sections (functions) that should be mapped to ITCM in
the following format:
```
*(.text.arm_ack_irq)
*(.text.arm_doirq)
*(.text._ZN4uORB12DeviceMaster19getDeviceNodeLockedEPK12orb_metadatah)
*(.text._ZN3Ekf20controlGravityFusionERKN9estimator9imuSampleE)
[...]
```

A specific entry in the linker script file can be ignored by adding a comment, as shown in the following example:
```
*(.text.arm_ack_irq) /* itcm-check-ignore */
```
"""

import argparse
import re
from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection
from elftools.dwarf.die import DIE
from pathlib import Path
from typing import List, Set


def die_get_funcs_rec(die: DIE, ret: Set[str]):
    """
    Recursively gets the mangled and demangled name of all functions in the given `die`.

    :param die: DIE to be processed. Is gathered recursively after passing a top DIE.
    :param ret: Output set where all function names are added to.
    """
    if die.tag in ("DW_TAG_subprogram", "DW_TAG_inlined_subroutine"):
        link_name_att = die.attributes.get("DW_AT_linkage_name")
        name_att = die.attributes.get("DW_AT_name")
        if link_name_att:
            ret.add(link_name_att.value.decode("utf-8"))
        if name_att:
            ret.add(name_att.value.decode("utf-8"))

    # Recurse into the DIE children
    for child in die.iter_children():
        die_get_funcs_rec(child, ret)


def get_elf_symbols_from_debug(elf_path: Path) -> Set[str]:
    """
    Gets all functions contained in the built ELF file using the DWARF debug info.

    :param elf_path: Path to the ELF file.

    :return: The names of the contained functions.
    """
    ret = set()
    with open(elf_path, 'rb') as f:
        elf = ELFFile(f)
        if not elf.has_dwarf_info():
            print("ELF does not have debug info. Compile with debug info.")
            exit(1)
        dwarf_info = elf.get_dwarf_info()
        for CU in dwarf_info.iter_CUs():
            top_die = CU.get_top_DIE()
            die_get_funcs_rec(top_die, ret)

    return ret


def get_elf_symbols_from_sections(elf_path: Path) -> Set[str]:
    """
    Gets all functions contained in the built ELF file using the symbol table section.

    :param elf_path: Path to the ELF file.

    :return: The names of the contained functions.
    """
    ret = set()
    with open(elf_path, 'rb') as f:
        elf = ELFFile(f)
        for section in elf.iter_sections():
            if isinstance(section, SymbolTableSection):
                for sym in section.iter_symbols():
                    ret.add(sym.name)
    return ret


def is_section_supported(section: str) -> bool:
    """
    Returns whether this type of section can be checked.

    :param section: Name of the section that should be checked for support.

    :return: Whether the type of section is supported.
    """
    not_supported_sections = [".isra", ".part", ".constprop"]
    return not any(not_supported in section for not_supported in not_supported_sections)


def get_input_sections(script_path: Path) -> List[str]:
    """
    Gets all sections (named after the functions) that should be mapped to ITCM according
    to the linker script.

    :param script_path: Path of the linker script.

    :return: The names of the sections
    """
    ret = []
    section_pattern = re.compile(r"^\*\(\.([a-zA-Z0-9_\.]+)\)$")
    ignored_marker = "itcm-check-ignore"
    with open(script_path, 'r') as f:
        for line in f:
            match = section_pattern.match(line)
            if match and ignored_marker not in line:
                section_name = match.group(1).replace("text.", "")
                if is_section_supported(section_name):
                    ret.append(section_name)
    return ret


def check_itcm(elf_path: Path, script_paths: List[Path]):
    """
    Checks if all the functions that should be mapped to ITCM are contained in the built ELF file.

    :param elf_path: Path of the ELF file.
    :param script_paths: Path of all linker scripts that should be checked.
    """
    elf_symbols_from_debug = get_elf_symbols_from_debug(elf_path)
    elf_symbols_from_sections = get_elf_symbols_from_sections(elf_path)
    elf_symbols = elf_symbols_from_debug.union(elf_symbols_from_sections)
    input_sections = []
    for script_path in script_paths:
        script_input_sections = get_input_sections(script_path)
        if script_input_sections:
            input_sections.extend(script_input_sections)
        else:
            print(f"No input sections found in {script_path}, please check if the path is correct.")

    check_passed = True
    for input_section in input_sections:
        if input_section not in elf_symbols:
            check_passed = False
            print(f"Section: {input_section} not found in the ELF file!")

    if check_passed:
        print("ITCM check passed!")
        exit(0)
    else:
        print("ITCM check failed!")
        exit(1)


def main():
    parser = argparse.ArgumentParser(description="Checks if functions marked for ITCM mapping exist in the ELF file.")
    parser.add_argument(
        "--elf-file",
        help="Path of the compiled ELF file",
        type=Path,
        required=True
    )
    parser.add_argument(
        "--script-files",
        help="Paths of the linker script files",
        nargs="+",
        type=Path,
        required=True
    )

    args = parser.parse_args()
    check_itcm(args.elf_file, args.script_files)


if __name__ == '__main__':
    main()
