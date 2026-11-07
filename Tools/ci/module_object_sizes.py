#!/usr/bin/env python3
"""Read target C++ object sizes from an unstripped ELF's DWARF information.

These are per-instance sizeof values, including inline members and padding.
They exclude allocations owned through pointers, allocator overhead, instance
counts and runtime heap peaks. An object's size does not establish whether an
instance is allocated on the heap, stack or in static storage.
"""

import argparse
import gzip
import io
import json
from pathlib import Path


TYPE_TAGS = {"DW_TAG_class_type", "DW_TAG_structure_type", "DW_TAG_union_type"}


def _declaration(die):
    """Out-of-line definitions inherit their name and scope from a declaration."""
    seen = set()
    while True:
        reference = next((name for name in ("DW_AT_specification", "DW_AT_abstract_origin")
                          if name in die.attributes), None)
        if reference is None:
            return die
        if id(die) in seen:
            raise ValueError("Cycle in DWARF declaration references")
        seen.add(id(die))
        die = die.get_DIE_from_attribute(reference)


def _name(die):
    attribute = die.attributes.get("DW_AT_name")
    if attribute is None:
        return None
    value = attribute.value
    return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else value


def _qualified_name(die):
    """Identify named namespace/class types, excluding local/anonymous scopes."""
    parts = []
    while die is not None:
        die = _declaration(die)
        if die.tag in TYPE_TAGS or die.tag == "DW_TAG_namespace":
            name = _name(die)
            if not name:
                return None
            parts.append(name)
        elif die.tag not in {"DW_TAG_compile_unit", "DW_TAG_type_unit", "DW_TAG_partial_unit"}:
            # Function-local types have no stable qualified type name suitable
            # for the registry. Do not mistake one for a global module class.
            return None
        die = die.get_parent()
    return "::".join(reversed(parts))


def _unavailable(type_names, reason):
    return {
        "status": "unavailable", "reason": reason,
        "types": {name: {"status": "not_found", "bytes": None, "reason": reason}
                  for name in type_names},
    }


def collect_object_sizes(elf_path, type_names):
    """Collect complete definitions of the requested fully qualified types.

    Repeated definitions across compilation units are normal and are deduplicated.
    Conflicting sizes are reported as ambiguous, never resolved by picking a DIE.
    A missing type or unavailable debug information has no numeric size.
    """
    type_names = list(dict.fromkeys(type_names))
    try:
        from elftools.common.exceptions import DWARFError, ELFError
        from elftools.dwarf.descriptions import describe_form_class
        from elftools.elf.elffile import ELFFile
    except ImportError:
        return _unavailable(type_names, "pyelftools is required to read DWARF object sizes")

    sizes = {name: set() for name in type_names}
    unsupported = set()
    try:
        path = Path(elf_path)
        if path.suffix == ".gz":
            # ELF readers seek extensively; decompress saved CI artifacts once.
            with gzip.open(path, "rb") as compressed:
                stream = io.BytesIO(compressed.read())
        else:
            stream = path.open("rb")
        with stream:
            elf = ELFFile(stream)
            if not elf.has_dwarf_info(strict=True):
                return _unavailable(type_names, "ELF has no DWARF .debug_info; preserve an unstripped ELF")
            dwarf = elf.get_dwarf_info(follow_links=False)
            for units in (dwarf.iter_CUs(), dwarf.iter_TUs()):
                for unit in units:
                    for die in unit.iter_DIEs():
                        if die.tag not in TYPE_TAGS or "DW_AT_byte_size" not in die.attributes:
                            continue
                        declaration = die.attributes.get("DW_AT_declaration")
                        if declaration is not None and declaration.value:
                            continue
                        name = _qualified_name(die)
                        if name not in sizes:
                            continue
                        attribute = die.attributes["DW_AT_byte_size"]
                        size = attribute.value
                        if describe_form_class(attribute.form) == "constant" and isinstance(size, int) and size >= 0:
                            sizes[name].add(size)
                        else:
                            unsupported.add(name)
                    # pyelftools caches every DIE indefinitely. A full PX4 ELF
                    # otherwise needs several GB of host RAM. Release completed
                    # units' DIE caches; later cross-unit references can reparse
                    # them. Keep the offset index and DIE list in sync.
                    unit._dielist.clear()
                    unit._diemap.clear()
    except (OSError, EOFError, ELFError, DWARFError, ValueError, NotImplementedError) as error:
        return _unavailable(type_names, f"Cannot read ELF/DWARF: {error}")

    types = {}
    for name, candidates in sizes.items():
        if name in unsupported:
            result = {"status": "ambiguous", "bytes": None,
                      "reason": "A definition has a nonconstant or invalid DWARF byte size"}
        elif len(candidates) > 1:
            values = ", ".join(str(value) for value in sorted(candidates))
            result = {"status": "ambiguous", "bytes": None,
                      "reason": f"Conflicting DWARF sizes: {values} bytes"}
        elif candidates:
            result = {"status": "ok", "bytes": next(iter(candidates)), "reason": None}
        else:
            result = {"status": "not_found", "bytes": None,
                      "reason": "No complete named type definition found in DWARF"}
        types[name] = result
    return {"status": "ok", "reason": None, "types": types}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path, help="Unstripped target ELF (or .elf.gz)")
    parser.add_argument("--types", required=True, nargs="+", help="Fully qualified C++ class/struct names")
    args = parser.parse_args()
    result = collect_object_sizes(args.elf, args.types)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
