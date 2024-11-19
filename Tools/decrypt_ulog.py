#!/usr/bin/env python3

import sys

try:
    from Crypto.Cipher import ChaCha20
except ImportError as e:
    print("Failed to import crypto: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pycryptodome")
    print("")
    sys.exit(1)

from Crypto.PublicKey import RSA
from Crypto.Cipher import PKCS1_OAEP
from Crypto.Hash import SHA256
from pathlib import Path
import argparse


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="""CLI tool to decrypt an ulog file\n""")
    parser.add_argument("ulog_file", help=".ulge/.ulgc, encrypted log file", nargs='?', default=None)
    parser.add_argument("ulog_key", help=".ulgk, legacy encrypted key (give empty string '' to ignore for .ulge)", nargs='?', default=None)
    parser.add_argument("rsa_key", help=".pem format key for decrypting the ulog key", nargs='?', default=None)

    args = parser.parse_args()

    # Check all arguments are given
    if not args.rsa_key:
        print('Need all arguments, the encrypted ulog file, key file (or empty string if not needed) and the key decryption key (.pem)')
        sys.exit(1)

    # Read the private RSA key to decrypt the cahcha key
    with open(args.rsa_key, 'rb') as f:
        r = RSA.importKey(f.read(), passphrase='')

    if args.ulog_key == "":
        key_data_filename = args.ulog_file
        magic = "ULogEnc"
    else:
        key_data_filename = args.ulog_key
        magic = "ULogKey"

    with open(key_data_filename, 'rb') as f:
        # Read the encrypted xchacha key and the nonce
        ulog_key_header = f.read(22)

        # Parse the header
        try:
            # magic
            if not ulog_key_header.startswith(bytearray(magic.encode())):
                print("Incorrect header magic")
                raise Exception()
            # version
            if ulog_key_header[7] != 1:
                print("Unsupported header version")
                raise Exception()
            # expected key exchange algorithm (RSA_OAEP)
            if ulog_key_header[16] != 4:
                print("Unsupported key algorithm")
                raise Exception()
            key_size = ulog_key_header[19] << 8 | ulog_key_header[18]
            nonce_size = ulog_key_header[21] << 8 | ulog_key_header[20]
            ulog_key_cipher = f.read(key_size)
            nonce = f.read(nonce_size)
        except:
            print("Keydata format error")
            sys.exit(1)

    if magic == "ULogEnc":
        data_offset = 22 + key_size + nonce_size
    else:
        data_offset = 0

    # Decrypt the xchacha key
    cipher_rsa = PKCS1_OAEP.new(r,SHA256)
    ulog_key = cipher_rsa.decrypt(ulog_key_cipher)
    #print(binascii.hexlify(ulog_key))

    # Read and decrypt the ulog data
    cipher = ChaCha20.new(key=ulog_key, nonce=nonce)

    outfilename = Path(args.ulog_file).stem + ".ulog"
    with open(args.ulog_file, 'rb') as f:
        if data_offset > 0:
            f.seek(data_offset)
        with open(outfilename, 'wb') as out:
            out.write(cipher.decrypt(f.read()))
