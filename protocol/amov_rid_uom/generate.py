#!/usr/bin/env python3
"""Generate a project-extended dialect without modifying upstream definitions."""

import argparse
import copy
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def write_if_changed(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_bytes() != data:
        path.write_bytes(data)


def overlay(definitions, dialect, extension, work_dir):
    source = definitions / (dialect + ".xml")
    if not source.is_file():
        raise ValueError("missing upstream dialect: " + str(source))
    if definitions.resolve() == work_dir.resolve():
        raise ValueError("staging directory must differ from upstream definitions")
    extra = ET.parse(extension).getroot()
    ids = {item.attrib["id"] for item in extra.findall("messages/message")}
    names = {item.attrib["name"] for item in extra.findall("messages/message")}
    enum_names = {item.attrib["name"] for item in extra.findall("enums/enum")}
    for xml_path in sorted(definitions.glob("*.xml")):
        upstream = ET.parse(xml_path).getroot()
        for message in upstream.findall("messages/message"):
            if message.attrib["id"] in ids or message.attrib["name"] in names:
                raise ValueError("project message collision in " + str(xml_path))
        for enum in upstream.findall("enums/enum"):
            if enum.attrib["name"] in enum_names:
                raise ValueError("project enum collision in " + str(xml_path))
        write_if_changed(work_dir / xml_path.name, xml_path.read_bytes())
    root = ET.parse(source).getroot()
    for group in ("enums", "messages"):
        destination = root.find(group)
        if destination is None:
            destination = ET.SubElement(root, group)
        for item in extra.find(group):
            destination.append(copy.deepcopy(item))
    result = work_dir / source.name
    write_if_changed(result, ET.tostring(root, encoding="utf-8", xml_declaration=True))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--definitions", required=True, type=Path)
    parser.add_argument("--dialect", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--work-dir", required=True, type=Path)
    parser.add_argument("--extension", type=Path, default=Path(__file__).with_name("amov.xml"))
    parser.add_argument(
        "--pymavlink-root", type=Path, help="Directory containing the pinned pymavlink package"
    )
    parser.add_argument("--language", choices=("C", "Python3"), default="C")
    args = parser.parse_args()
    if args.pymavlink_root:
        sys.path.insert(0, str(args.pymavlink_root.resolve()))
    from pymavlink.generator import mavgen

    xml_path = overlay(args.definitions, args.dialect, args.extension, args.work_dir)
    options = mavgen.Opts(str(args.output), wire_protocol="2.0", language=args.language)
    if not mavgen.mavgen(options, [str(xml_path)]):
        return 1
    # The unpatched upstream generator embeds the build date, which is not a wire field.
    if args.language == "C":
        for version in args.output.glob("*/version.h"):
            data = version.read_text(encoding="utf-8")
            stable = (
                "\n".join(
                    line
                    for line in data.splitlines()
                    if not line.startswith("#define MAVLINK_BUILD_DATE")
                )
                + "\n"
            )
            write_if_changed(version, stable.encode("utf-8"))
    return 0


if __name__ == "__main__":
    if os.environ.get("PYTHONHASHSEED") != "0":
        os.environ["PYTHONHASHSEED"] = "0"
        os.execv(sys.executable, [sys.executable, *sys.argv])
    sys.exit(main())
