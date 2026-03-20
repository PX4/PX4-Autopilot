#!/usr/bin/env python3
from os import EX_CANTCREAT
import nacl.encoding
import nacl.signing
import nacl.hash
import struct
import zlib
import json
import base64
from pathlib import Path
from dataclasses import dataclass
import cryptotools
import argparse
import os

# Dictionary describing the possible TOC flags, they can be OR'ed together
toc_flag_dict = {
    'TOC_FLAG1_BOOT': 0x1,
    'TOC_FLAG1_VTORS': 0x2,
    'TOC_FLAG1_CHECK_SIGNATURE': 0x4,
    'TOC_FLAG1_DECRYPT': 0x8,
    'TOC_FLAG1_COPY': 0x10,
    'TOC_FLAG1_RDCT': 0x80,
}

# Dataclasses describing the TOC data, used to parse the to and from binary


@dataclass
class TOC_start:
    start_magic: str = None
    version: int = None
    STRUCT_STRUCTURE = "<4sI"  # type defininig string from the STRUCT package

# Dataclass to describe the TOC for 2 entries, one describing the signature typen
# and the second one describing the signature location.


@dataclass
class TOC_entry:
    toc_position: int = 0
    app_name: str = None
    app_start: int = None
    app_end: int = None
    app_target: int = None
    app_signature_idx: int = None
    app_signature_key: int = None
    app_encryption_key: int = None
    app_flags1: int = None
    app_reserved: int = None
    sig_name: str = None
    sig_start: int = None
    sig_end: int = None
    sig_target: int = None
    sig_signature_idx: int = None
    sig_signature_key: int = None
    sig_encryption_key: int = None
    sig_flags1: int = None
    sig_reserved: int = None
    # type defininig string from the STRUCT package
    STRUCT_STRUCTURE = "<4sIIIBBBBI4sIIIBBBBI"


def toc2bin(data):
    '''
    Takes as TOC data class and converts it to the binary representation.
    Prepare a TOC_entry data class with values and hand it over to this function.

    data: is a dataclass TOC_entry
    retrun: a packed binary to add to the px4 bin file
    '''
    return struct.pack(data.STRUCT_STRUCTURE,
                       data.app_name, data.app_start,
                       data.app_end, data.app_target,
                       data.app_signature_idx,
                       data.app_signature_key,
                       data.app_encryption_key,
                       data.app_flags1,
                       data.app_reserved,
                       data.sig_name, data.sig_start,
                       data.sig_end, data.sig_target,
                       data.sig_signature_idx,
                       data.sig_signature_key,
                       data.sig_encryption_key,
                       data.sig_flags1,
                       data.sig_reserved)


def bin2toc_start(bin):
    '''
    Takes binary data and unpacks a Toc_start header.

    bin: binary data to parse for the TOC header.

    return: a dataclass TOC_start
    '''
    data = TOC_start()
    (data.start_magic, data.version) = struct.unpack(data.STRUCT_STRUCTURE, bin)
    return data


def bin2toc_entry(bin):
    '''
    Takes binary data and unpacks it into a TOC entry dataclass.
    bin: Binary data to unpack the TOC entry from.

    return: A dataclass of TOC_entry
    '''

    data = TOC_entry()
    (data.app_name, data.app_start,
     data.app_end, data.app_target,
     data.app_signature_idx,
     data.app_signature_key,
     data.app_encryption_key,
     data.app_flags1,
     data.app_reserved,
     data.sig_name, data.sig_start,
     data.sig_end, data.sig_target,
     data.sig_signature_idx,
     data.sig_signature_key,
     data.sig_encryption_key,
     data.sig_flags1,
     data.sig_reserved
     ) = struct.unpack(data.STRUCT_STRUCTURE, bin)

    return data


def parse_toc(bin):
    '''
    Searches for the TOC in the binary data.
    This function looks for a TOC within certain boundaries in the TOC. It
    also checks to find a valid TOC_end.
    Throws exceptions, if a toc is not found, wrong version or end is not found.

    bin: Binary data to look for the TOC

    return: dataclass with the TOC entry
    '''

    # This is a fixed address from the linker file, TOC is placed after.
    BOOT_DELAY_ADDR = 0x200
    TOC_LEN_MAX = 64
    EXPECTED_TOC_VERSION = 1

    toc_start_len = struct.calcsize(TOC_start().STRUCT_STRUCTURE)
    toc_entry_len = struct.calcsize(TOC_entry().STRUCT_STRUCTURE)

    start_indx = bin.find(b'TOC', BOOT_DELAY_ADDR,
                          BOOT_DELAY_ADDR + TOC_LEN_MAX)
    if start_indx <= 0:
        raise Exception('TOC not found')

    toc_start_header = bin2toc_start(
        bin[start_indx:(start_indx+toc_start_len)])

    print('TOC start found', toc_start_header,'@: ',hex(start_indx))

    if toc_start_header.version != EXPECTED_TOC_VERSION:
        raise Exception('Wrong TOC version!')

    t = bin2toc_entry(
        bin[start_indx+toc_start_len: (start_indx+toc_start_len+toc_entry_len)])
    t.toc_position = start_indx+toc_start_len
    print(t)

    indx = bin.find(b'END', start_indx, start_indx + 512)
    if indx <= 0:
        toc_end = False
        raise Exception('TOC end  not found')

    return t


def unpackPx4(file_path):
    '''
    Unpacks a .px4 file to get access to its binary data.

    filepath: Path to a px4 file to extract.

    return: A tuple of (binary data, json_data) of the file.

    '''
    # read the file
    with open(file_path, "r") as f:
        desc = json.load(f)
    image = bytearray(zlib.decompress(base64.b64decode(desc['image'])))
    return image, desc


def packPx4(bin_image, json_data, file_path, pub_key):
    '''
    Packs a new .px4 file with given binary data and information from jason file.

    bin_image: New binary data with signature to add to .px4 file
    json_data: The json data from the previously parsed .px4.
    file_path: File path to then new .px4.sec image.

    return: Nothing
    '''

    head,tail=os.path.splitext(file_path)
    with open(head +'_signed.px4', 'w') as f:
        json_data['signed'] = 'Hash512_Ed25519'
        json_data['pub_key'] = pub_key.decode('utf-8')
        json_data['image_size'] = len(bin_image)
        json_data['image'] = base64.b64encode(
            zlib.compress(bin_image, 9)).decode('utf-8')
        print('Pack new signed.px4 file with signature')
        json.dump(json_data, f, indent=4)


def ed25519_sign(private_key, signee_bin):
    """
    This function creates the signature. It takes the private key and the binary file
    and returns the tuple (signature, public key)
    """

    signing_key = nacl.signing.SigningKey(
        private_key, encoder=nacl.encoding.HexEncoder)

    # Sign a message with the signing key
    signed = signing_key.sign(signee_bin, encoder=nacl.encoding.RawEncoder)

    # Obtain the verify key for a given signing key
    verify_key = signing_key.verify_key

    # Serialize the verify key to send it to a third party
    verify_key_hex = verify_key.encode(encoder=nacl.encoding.HexEncoder)

    return signed.signature, verify_key_hex


def write_toc(toc_old, bin, signature_name, new_pub_key_index, new_flags):
    '''
    Writes a new TOC entry.
    toc_old: The parsed data_class with the data of the old TOC
    bin: The binary to insert the new TOC
    signature_name: 4 char name of the newly added signature
    new_pub_key_index: Key index of the public key belongs to the signature
    new_flags: New Toc flags from toc_flag_dict dictionary.

    return: New bin with modified TOC
    '''

    toc_new = toc_old
    toc_new.app_signature_key = new_pub_key_index
    toc_new.app_flags1 = new_flags
    if len(signature_name) != 4:
        raise Exception('Signature name has not the right length')
    toc_new.app_name = bytes(signature_name, 'utf_8')
    toc_new_bin = toc2bin(toc_new)

    toc_entry_len = struct.calcsize(TOC_entry().STRUCT_STRUCTURE)
    bin[toc_old.toc_position:toc_old.toc_position+toc_entry_len] = toc_new_bin

    return bin


def cli():
    '''
    Comand lined interface to the signtool.
    See usage comand.

    return: class with parsed arguments.
    '''

    parser = argparse.ArgumentParser(
        description='Tool to extract and find crypto TOC from .px4 file. And append a signature from a given private key')
    # defining arguments for parser object
    parser.add_argument("--signee", type=str,
                        metavar="file_path", default=None,
                        help="Opens and reads the specified px4 file.")
    parser.add_argument("--private_key", type=str,
                        metavar="key string", default=None,
                        help="Private key to sign the px4 image")
    parser.add_argument("--key_index", type=int,
                        metavar="Number", default=None,
                        help="Index of the public key used to verify the binary")

    parser.add_argument("--TOC_flags", type=str, choices=['TOC_FLAG1_BOOT', 'TOC_FLAG1_RDCT'],
                        default=None,
                        help="TOC flags to indicate signature")
    args = parser.parse_args()
    return args


def sign(file_path, private_key, key_index, TOC_flags):
    '''
    Signs a binary file and updates TOC accordingly.
    Reads a .px4 or a .bin at specified location and writes a new _signed.[px4|bin] file at the same location.

    file_path: Path to a .px4 or .bin file to sign.
    private_key: String of the private key used to sign the binary.
    key_index: Index of the public key used to verify signature.
    TOC_flags: New toc flags to be written.
    '''
    head,tail=os.path.splitext(file_path)
    if(tail == '.px4'):
        bin, json_data = unpackPx4(file_path)
    elif(tail == '.bin'):
        with open(file_path,mode='rb') as f:
            bin =bytearray(f.read())
    else:
        raise Exception('Error: Unknown file type')


    toc_old = parse_toc(bin)
    new_bin = write_toc(toc_old, bin, 'MSTR', key_index,
                        toc_flag_dict[TOC_flags])
    app_len = toc_old.app_end - toc_old.app_start
    print('Calculate new signature')
    signature, pub_key = cryptotools.ed25519_sign(
        private_key, bytes(new_bin[0:app_len]))
    # Append signature to binary
    print('Append signature to binary: ',signature.hex(),'\n','length: ',app_len )

    sig_start=toc_old.sig_start - toc_old.app_start
    sig_end = toc_old.sig_end - toc_old.app_start

    if sig_start != app_len:
        padding = bytearray(b'\xff')*(sig_start - app_len)
        new_bin += padding

    new_bin[toc_old.sig_start-toc_old.app_start:toc_old.sig_end -
            toc_old.app_start] = signature

    if(tail == '.px4'):
        packPx4(new_bin, json_data, file_path,pub_key)
    elif(tail == '.bin'):
        head,tail=os.path.splitext(file_path)
        with open(head +'_signed.bin',mode='wb') as f:
            f.write(bin)
    else:
        raise Exception('Error: Unknown file type!')



if(__name__ == "__main__"):
    args = cli()
    # test input to the sign function if not run from CLI
    # sign('PX4-Autopilot/build/px4_fmu-v5x_default/px4_fmu-v5x_default.px4',
    #      "8e969454c3f1ba924b4d436bc58b87d20c082ea99fb6a77b9e00f7b20dab0fd8",
    #      0, 'TOC_FLAG1_BOOT')
    # CLI test input:
    # --signee /PX4-Autopilot/build/px4_fmu-v5x_default/px4_fmu-v5x_default.px4
    #  --private_key 8e969454c3f1ba924b4d436bc58b87d20c082ea99fb6a77b9e00f7b20dab0fd8 --key_index 0 --TOC_flags TOC_FLAG1_BOOT
    sign(args.signee, args.private_key, args.key_index, args.TOC_flags)
