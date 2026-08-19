#!/usr/bin/env python3

############################################################################
#
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

"""Validate strong dynamic imports from an ELF shared object."""

import argparse
import subprocess
import sys
from pathlib import Path
from typing import List, Set


def read_elf_symbols(nm: Path, elf: Path, options: List[str]) -> Set[str]:
    result = subprocess.run(
        [
            str(nm),
            *options,
            str(elf),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    )

    if result.returncode != 0:
        if result.stderr:
            print(result.stderr.rstrip(), file=sys.stderr)

        raise RuntimeError(
            "{} failed with status {}".format(nm, result.returncode)
        )

    return {
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip()
    }


def read_elf_imports(nm: Path, elf: Path) -> Set[str]:
    return read_elf_symbols(
        nm,
        elf,
        [
            "--dynamic",
            "--undefined-only",
            "--no-weak",
            "--just-symbol-name",
        ],
    )


def read_elf_exports(nm: Path, elf: Path) -> Set[str]:
    return read_elf_symbols(
        nm,
        elf,
        [
            "--dynamic",
            "--defined-only",
            "--just-symbol-name",
        ],
    )


def read_symbol_file(path: Path) -> Set[str]:
    symbols = set()

    with path.open(encoding="utf-8") as symbol_file:
        for line in symbol_file:
            symbol = line.partition("#")[0].strip()

            if symbol:
                symbols.add(symbol)

    return symbols


def print_symbols(title: str, symbols: Set[str]) -> None:
    if not symbols:
        return

    print(title, file=sys.stderr)

    for symbol in sorted(symbols):
        print("  {}".format(symbol), file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nm", type=Path, required=True)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument(
        "--provider-symbol-file",
        action="append",
        default=[],
        type=Path,
        metavar="FILE",
        help="File containing one provider symbol per line",
    )
    parser.add_argument(
        "--provider-elf",
        action="append",
        default=[],
        type=Path,
        metavar="ELF",
        help="ELF whose defined dynamic symbols can satisfy imports",
    )
    parser.add_argument(
        "--forbid-prefix",
        action="append",
        default=[],
        help="Reject imports beginning with this prefix",
    )
    args = parser.parse_args()

    if any(not prefix for prefix in args.forbid_prefix):
        parser.error("--forbid-prefix must not be empty")

    validate_providers = bool(args.provider_symbol_file or args.provider_elf)

    if not validate_providers and not args.forbid_prefix:
        parser.error("at least one provider or --forbid-prefix is required")

    try:
        actual = read_elf_imports(args.nm, args.elf)
        provided = set()

        for symbol_file in args.provider_symbol_file:
            provided.update(read_symbol_file(symbol_file))

        for provider_elf in args.provider_elf:
            provided.update(read_elf_exports(args.nm, provider_elf))

    except (OSError, RuntimeError, UnicodeError) as error:
        print("ELF import check failed: {}".format(error), file=sys.stderr)
        return 1

    forbidden_actual = {
        symbol
        for symbol in actual
        if any(symbol.startswith(prefix) for prefix in args.forbid_prefix)
    }
    unavailable_actual = set()

    if validate_providers:
        unavailable_actual = actual - provided - forbidden_actual

    print_symbols("Forbidden strong dynamic imports:", forbidden_actual)
    print_symbols("Unavailable strong dynamic imports:", unavailable_actual)

    if forbidden_actual or unavailable_actual:
        if forbidden_actual:
            print(
                "\nProject-owned imports must be resolved within the ELF.",
                file=sys.stderr,
            )

        if unavailable_actual:
            print(
                "\nUnavailable imports must be exported by a configured provider.",
                file=sys.stderr,
            )

        return 1

    checks = []

    if validate_providers:
        checks.append("all are provided")

    if args.forbid_prefix:
        checks.append("none match {}".format(", ".join(args.forbid_prefix)))

    print(
        "Checked {} strong dynamic imports in {}; {}".format(
            len(actual), args.elf, "; ".join(checks)
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
