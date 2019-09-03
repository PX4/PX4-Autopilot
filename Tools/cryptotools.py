#!/usr/bin/env python3

import nacl.encoding
import nacl.signing
import nacl.encoding
import nacl.hash
import struct
import binascii
import json
import time
import argparse
from pathlib import Path
import sys


# Definitins for the meta data heder of the signed application
meta_data ={'appp_size':0,
            'app_hash':0,
            }

META_DATA_STRUCT = "<I64s60x" # [4 length][64 sha512][60 padding]
META_DATA_LEN = 128


def pack_meta_data(meta_data):
    """
    Packs the given meta data into a binary struct, to be written
    on top of the binary application file.
    """
    return struct.pack(META_DATA_STRUCT, meta_data['appp_size'],
                                         meta_data['app_hash'])

def make_public_key_h_file(signing_key):
    """
    This file generate the public key header file
    to be included into the bootloader build.
    """
    public_key_c=''
    public_key_c="const uint8_t public_key={\n"
    for i,c in enumerate(signing_key.verify_key.encode(encoder=nacl.encoding.RawEncoder)):
        public_key_c+= hex(c)
        public_key_c+= ', '
        if((i+1)%8==0):
            public_key_c+= '\n'
    public_key_c+= "};"
    with open("public_key.h" ,mode='w') as f:
        f.write("//Public key to verify signed binaries")
        f.write(public_key_c)

def make_key_file(signing_key):
    """
    Writes the key.json file.
    Attention do not override your existing key files.
    Do not publish your private key!!
    """

    key_file = Path("keys.json")
    if key_file.is_file():
        print("ATTENTION: key.json already exists, are you sure you wnat to overwrite it?")
        print("Remove file and run script again.")
        print("Script aborted!")
        sys.exit(-1)

    keys={}
    keys["date"] = time.asctime()
    keys["public"] = (signing_key.verify_key.encode(encoder=nacl.encoding.HexEncoder)).decode()
    keys["private"] = binascii.hexlify(signing_key._seed).decode()
    print (keys)
    with open("keys.json", "w") as write_file:
        json.dump(keys, write_file)

def ed25519_sign(private_key, signee_bin):
    """
    This functino does the magic. It tkaes the pricate key and the binary file
    and generates the signed binary. it adds the meta data to the beginning of the file
    Ouput: "SignedBin.bin"
    """

    # metadata is already included into the source bin,
    # but filled with dummy data

    meta_data['appp_size'] =  len(signee_bin) - META_DATA_LEN

    signing_key = nacl.signing.SigningKey(private_key, encoder=nacl.encoding.HexEncoder)

    # Sign a message with the signing key
    signed = signing_key.sign(signee_bin[META_DATA_LEN:],encoder=nacl.encoding.RawEncoder)

    meta_data['app_hash'] = signed.signature

    # Obtain the verify key for a given signing key
    verify_key = signing_key.verify_key

    # Serialize the verify key to send it to a third party
    verify_key_hex = verify_key.encode(encoder=nacl.encoding.HexEncoder)

    print(len(signed.signature), "signature:" ,binascii.hexlify(signed.signature))
    print('Public key: ', verify_key_hex)

    with open("SignedBin.bin" ,mode='wb') as f:
        data = pack_meta_data(meta_data)
        print("meta data header: ", binascii.hexlify(data),len(data))

        f.write(data)
        f.write(signee_bin[META_DATA_LEN:])

def sign(bin_file_path, key_file_path=None):
    """
    reads the binary file and the key file.
    If the key file does not exist, it generates a
    new key file.
    """

    with open(bin_file_path,mode='rb') as f:
        signee_bin = f.read()
    if key_file_path == None:
        print('generating new key')
        generate_key()
        key_file_path = 'keys.json'

    with open(key_file_path,mode='r') as f:
        keys = json.load(f)
        print(keys)

    ed25519_sign(keys["private"], signee_bin)



def generate_key():
    """
    Call it and it generate two files,
    one file is made to be include in the bootloader build
    so its the "public_key.h" containg the verfication key.
    The other file key.json, containt both private and public key.
    Do not leak or loose the key file. This is mandatory for signing
    all future binaries you want to deploy!

    """

    # Generate a new random signing key
    signing_key = nacl.signing.SigningKey.generate()
     # Serialize the verify key to send it to a third party
    verify_key_hex = signing_key.verify_key.encode(encoder=nacl.encoding.HexEncoder)
    print("public key :",verify_key_hex)

    private_key_hex=binascii.hexlify(signing_key._seed)
    print("private key :",private_key_hex)

    make_key_file(signing_key)
    make_public_key_h_file(signing_key)



if(__name__ == "__main__"):

    parser = argparse.ArgumentParser(description="""CLI tool to calculate and add signature to px4. bin files\n
                                                  if given it takes an existing key file, else it generate new keys""",
                                    epilog="Output: SignedBin.bin and a key.json file")
    parser.add_argument("signee", help=".bin file to add signature")
    parser.add_argument("--key", help="key.json file", default=None)
    args = parser.parse_args()

    sign(args.signee, args.key)
