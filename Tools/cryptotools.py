import nacl.encoding
import nacl.signing
import nacl.encoding
import nacl.hash
import struct
import binascii

meta_data ={'appp_size':0,
            'app_hash':0,
            }

META_DATA_STRUCT = "<I64s60x" # [4 length][64 sha512][60 padding]
META_DATA_LEN = 128

def pack_meta_data(meta_data):
    return struct.pack(META_DATA_STRUCT, meta_data['appp_size'],
                                         meta_data['app_hash'])
def generate_sha512_header():
    with open("../build/px4_fmu-v5_crypto_btl_default/px4_fmu-v5.bin" ,mode='rb') as f:
        source_bin = f.read()

        # metadata is already included into the source bin,
        # but filled with dummy data

        meta_data['appp_size']=  len(source_bin)-META_DATA_LEN
        meta_data['app_hash'] = nacl.hash.sha512(source_bin[META_DATA_LEN:], encoder=nacl.encoding.RawEncoder)

        print("size of binary is: ", meta_data['appp_size'],hex(meta_data['appp_size'],))
        print("SHA512 of binary is: ",meta_data['app_hash'])

    with open("../build/px4_fmu-v5_crypto_btl_default/px4_fmu-v5crypto.bin" ,mode='wb') as f:
        data = pack_meta_data(meta_data)
        print("meta data heder: ", binascii.hexlify(data),len(data))

        f.write(data)
        f.write(source_bin[META_DATA_LEN:])

print("new crypto bin written")


def ed25519_sign():
     with open("build/px4_fmu-v5_crypto_btl_default/px4_fmu-v5.bin" ,mode='rb') as f:
        source_bin = f.read()

        # metadata is already included into the source bin,
        # but filled with dummy data

        meta_data['appp_size']=  len(source_bin)-META_DATA_LEN


        # Generate a new random signing key
        signing_key = nacl.signing.SigningKey.generate()
        private_key =b'e874323dea846720715f2179ae2c03f3c80a9caaa5e25eb3951096fd80f71ab7'

        signing_key = nacl.signing.SigningKey(private_key, encoder=nacl.encoding.HexEncoder)

        # Sign a message with the signing key
        signed = signing_key.sign(source_bin[META_DATA_LEN:],encoder=nacl.encoding.RawEncoder)

        meta_data['app_hash'] = signed.signature

        # Obtain the verify key for a given signing key
        verify_key = signing_key.verify_key

        # Serialize the verify key to send it to a third party
        verify_key_hex = verify_key.encode(encoder=nacl.encoding.HexEncoder)

        print(len(signed.signature), "signature:" ,binascii.hexlify(signed.signature))
        print('Public key: ', verify_key_hex)

        with open("build/px4_fmu-v5_crypto_btl_default/px4_fmu-v5crypto.bin" ,mode='wb') as f:
            data = pack_meta_data(meta_data)
            print("meta data header: ", binascii.hexlify(data),len(data))

            f.write(data)
            f.write(source_bin[META_DATA_LEN:])




if(__name__ == "__main__"):
    #generate_sha512_header()
    ed25519_sign()

#import monocypher

## Generate a new random signing key
#signing_key = nacl.signing.SigningKey.generate()
#
## Sign a message with the signing key
#signed = signing_key.sign(b"Attack at Dawn")
#
## Obtain the verify key for a given signing key
#verify_key = signing_key.verify_key
#
## Serialize the verify key to send it to a third party
#verify_key_hex = verify_key.encode(encoder=nacl.encoding.HexEncoder)
#print(verify_key_hex)
#print(signed)
#print(signed)
#print(signing_key)
#
