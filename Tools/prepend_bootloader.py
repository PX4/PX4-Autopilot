#!/usr/bin/env python3
############################################################################
#
#   Copyright (C) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
"""
Prepend a bootloader to an application image so the pair can be flashed in
one step at the start of flash (e.g. st-flash write out.bin 0x8000000).

The bootloader is padded with 0xff up to the application's load address,
which is taken from the lowest PT_LOAD segment of the application ELF.
"""

import argparse
import struct
import sys

PT_LOAD = 1


def elf_load_address(path):
    with open(path, "rb") as f:
        data = f.read()

    if data[:4] != b"\x7fELF" or data[4] != 1:
        sys.exit(f"{path}: not an ELF32 file")

    little = data[5] == 1
    e = "<" if little else ">"
    (e_phoff,) = struct.unpack_from(e + "I", data, 0x1c)
    (e_phentsize, e_phnum) = struct.unpack_from(e + "HH", data, 0x2a)

    lowest = None

    for i in range(e_phnum):
        (p_type, p_offset, p_vaddr, p_paddr, p_filesz) = struct.unpack_from(
            e + "IIIII", data, e_phoff + i * e_phentsize)

        if p_type == PT_LOAD and p_filesz > 0:
            lowest = p_paddr if lowest is None else min(lowest, p_paddr)

    if lowest is None:
        sys.exit(f"{path}: no loadable segments")

    return lowest


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bootloader", required=True, help="bootloader .bin, placed at --base")
    parser.add_argument("--app", required=True, help="application .bin, placed at its ELF load address")
    parser.add_argument("--elf", required=True, help="application ELF, used for the load address")
    parser.add_argument("--base", type=lambda x: int(x, 0), default=0x08000000,
                        help="flash address the output is flashed to (default 0x08000000)")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    app_address = elf_load_address(args.elf)
    app_offset = app_address - args.base

    with open(args.bootloader, "rb") as f:
        bootloader = f.read()

    with open(args.app, "rb") as f:
        app = f.read()

    if app_offset < 0:
        sys.exit(f"application load address 0x{app_address:08x} is below --base 0x{args.base:08x}")

    if len(bootloader) > app_offset:
        sys.exit(f"bootloader ({len(bootloader)} bytes) overlaps the application at offset 0x{app_offset:x}")

    with open(args.output, "wb") as f:
        f.write(bootloader)
        f.write(b"\xff" * (app_offset - len(bootloader)))
        f.write(app)

    print(f"{args.output}: bootloader {len(bootloader)} bytes @ 0x{args.base:08x}, "
          f"application {len(app)} bytes @ 0x{app_address:08x}")


if __name__ == "__main__":
    main()
