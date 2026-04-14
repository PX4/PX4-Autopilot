#!/usr/bin/env python3
"""
Generate an ed25519 keypair for PX4 secure bootloader firmware signing.

Produces two files next to each other:
  <name>.json  - private + public key, used by sign_firmware.py
  <name>.pub   - public key as a C array, included in the bootloader build
                 via CONFIG_PUBLIC_KEY0..3

The .json file contains the private key. Do not publish it. Do not lose it:
without it you cannot sign new firmware images for devices that already
trust its public key.
"""

import argparse
import binascii
import json
import sys
import time
from pathlib import Path

import nacl.encoding
import nacl.signing


def write_public_key_header(signing_key, key_name):
    raw = signing_key.verify_key.encode(encoder=nacl.encoding.RawEncoder)
    lines = []
    for i in range(0, len(raw), 8):
        chunk = ", ".join(f"0x{b:02x}" for b in raw[i:i + 8])
        lines.append(chunk + ",")
    body = "\n".join(lines) + "\n"

    with open(key_name + ".pub", "w") as f:
        f.write("// Public key to verify signed binaries\n")
        f.write(body)


def write_key_json(signing_key, key_name):
    path = Path(key_name + ".json")
    if path.exists():
        print(f"ERROR: {path} already exists. Refusing to overwrite.")
        print("Remove the file and run again if you really want a new key.")
        sys.exit(1)

    keys = {
        "date": time.asctime(),
        "public": signing_key.verify_key.encode(encoder=nacl.encoding.HexEncoder).decode(),
        "private": binascii.hexlify(signing_key._seed).decode(),
    }
    with open(path, "w") as f:
        json.dump(keys, f)


def main():
    parser = argparse.ArgumentParser(
        description="Generate an ed25519 keypair for PX4 firmware signing.",
    )
    parser.add_argument(
        "name",
        help="Output base name. Writes <name>.json and <name>.pub.",
    )
    args = parser.parse_args()

    signing_key = nacl.signing.SigningKey.generate()
    write_key_json(signing_key, args.name)
    write_public_key_header(signing_key, args.name)

    print(f"Wrote {args.name}.json (private + public)")
    print(f"Wrote {args.name}.pub  (public, C array for bootloader build)")


if __name__ == "__main__":
    main()
