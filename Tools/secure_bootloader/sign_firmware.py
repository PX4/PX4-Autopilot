#!/usr/bin/env python3
"""
Sign a PX4 firmware .bin with an ed25519 key for the secure bootloader.

The output binary is the input padded to a 4-byte boundary with 0xff,
followed by a 64-byte signature, followed (optionally) by an R&D
certificate appended verbatim.

The signed image lives in the flash slot the bootloader expects to find
the main app at, so the layout looks like:

    [ vectors / .text / .data ... ][ pad to 4B ][ 64B signature ][ rdct? ]

The bootloader walks the image TOC, finds the BOOT entry's signature_idx
slot, and verifies it against a key stored in the bootloader's keystore.
"""

import argparse
import binascii
import json
import sys

import nacl.encoding
import nacl.signing


def ed25519_sign(private_key_hex, signee_bin):
    signing_key = nacl.signing.SigningKey(private_key_hex, encoder=nacl.encoding.HexEncoder)
    signed = signing_key.sign(signee_bin, encoder=nacl.encoding.RawEncoder)
    public_key = signing_key.verify_key.encode(encoder=nacl.encoding.RawEncoder)
    return signed.signature, public_key


def sign(bin_file_path, key_file_path):
    with open(bin_file_path, mode="rb") as f:
        signee_bin = f.read()

    if len(signee_bin) % 4 != 0:
        signee_bin += bytearray(b"\xff") * (4 - len(signee_bin) % 4)

    try:
        with open(key_file_path, mode="r") as f:
            keys = json.load(f)
    except OSError:
        print(f"ERROR: Key file {key_file_path} not found")
        sys.exit(1)

    signature, public_key = ed25519_sign(keys["private"], signee_bin)
    assert len(signature) == 64

    print(f'Binary "{bin_file_path}" signed.')
    print("Signature:", binascii.hexlify(signature).decode())
    print("Public key:", binascii.hexlify(public_key).decode())

    return signee_bin + signature


def main():
    parser = argparse.ArgumentParser(
        description="Sign a PX4 .bin firmware image with an ed25519 key.",
    )
    parser.add_argument("signee", help="Input .bin to sign")
    parser.add_argument("signed", help="Output signed .bin")
    parser.add_argument(
        "--key",
        default="Tools/test_keys/test_keys.json",
        help="Key file produced by generate_signing_keys.py",
    )
    parser.add_argument(
        "--rdct",
        default=None,
        help="Optional R&D certificate binary, appended after signature",
    )
    args = parser.parse_args()

    if args.key == "Tools/test_keys/test_keys.json":
        print("WARNING: Signing with PX4 test key — do not use in production")

    signed_bytes = sign(args.signee, args.key)

    with open(args.signed, mode="wb") as f:
        f.write(signed_bytes)

    if args.rdct is not None:
        with open(args.rdct, mode="rb") as rdct_in, open(args.signed, mode="ab") as out:
            out.write(rdct_in.read())


if __name__ == "__main__":
    main()
