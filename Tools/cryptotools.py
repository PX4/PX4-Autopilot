#!/usr/bin/env python3

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives import serialization
import struct
import binascii
import json
import time
import argparse
from pathlib import Path
import sys

def ed25519_sign(private_key, signee_bin):
    """
    This function creates the signature. It takes the private key and the binary file
    and returns the tuple (signature, public key)
    """

    # Sign a message with the signing key
    signature = private_key.sign(signee_bin)

    # Obtain the verify key for a given signing key
    public_key = private_key.public_key()
    verify_key = public_key.public_bytes(
        encoding=serialization.Encoding.Raw,
        format=serialization.PublicFormat.Raw
    )

    return signature, verify_key

def sign(bin_file_path, key_file_path=None):
    """
    reads the binary file and the key file.
    If the key file does not exist, it generates a
    new key file.
    """

    with open(bin_file_path,mode='rb') as f:
        signee_bin = f.read()
        # Align to 4 bytes. Signature always starts at
        # 4 byte aligned address, but the signee size
        # might not be aligned
        if len(signee_bin)%4 != 0:
            signee_bin += bytearray(b'\xff')*(4-len(signee_bin)%4)

    try:
        with open(key_file_path,mode='rb') as f:
            private_key = serialization.load_pem_private_key(f.read(), None)

    except:
        print('ERROR: Key file',key_file_path,'not  found')
        sys.exit(1)

    signature, public_key = ed25519_sign(private_key, signee_bin)

    # Do a sanity check. This type of signature is always 64 bytes long
    assert len(signature) == 64

    # Print out the signing information
    print("Binary \"%s\" signed."%bin_file_path)
    print("Signature:",binascii.hexlify(signature))
    print("Public key:",binascii.hexlify(public_key))

    return signee_bin + signature, public_key

if(__name__ == "__main__"):

    parser = argparse.ArgumentParser(description="""CLI tool to calculate and add signature to px4. bin files\n
                                                  if given it takes an existing key file, else it generate new keys""",
                                    epilog="Output: SignedBin.bin and a key.json file")
    parser.add_argument("signee", help=".bin file to add signature", nargs='?', default=None)
    parser.add_argument("signed", help="signed output .bin", nargs='?', default=None)

    parser.add_argument("--key", help="key.json file", default="Tools/test_keys/ed25519_test_key.pem")
    parser.add_argument("--rdct", help="binary R&D certificate file", default=None)
    args = parser.parse_args()

    # Check that both signee and signed exist
    if not args.signee or not args.signed:
        print("ERROR: Must either provide file names for both signee and signed")
        print("       or --genkey [key] to generate a new key pair")
        sys.exit(1)

    # Issue a warning when signing with testing key
    if args.key=='Tools/test_keys/ed25519_test_key.pem':
        print("WARNING: Signing with PX4 test key")

    # Sign the binary
    signed, public_key = sign(args.signee, args.key)

    with open(args.signed, mode='wb') as fs:
        # Write signed binary
        fs.write(signed)

    # Append rdcert if given
    try:
        with open(args.rdct ,mode='rb') as f:
            with open(args.signed, mode='ab') as fs:
                fs.write(f.read())
    except:
        pass
