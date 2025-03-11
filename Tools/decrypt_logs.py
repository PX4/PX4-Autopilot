#!/usr/bin/env python3

import os
import sys
import argparse
from pathlib import Path

try:
    from Crypto.Cipher import ChaCha20
    from Crypto.PublicKey import RSA
    from Crypto.Cipher import PKCS1_OAEP
    from Crypto.Hash import SHA256
except ImportError as e:
    print("Failed to import crypto: " + str(e))
    print("You may need to install it using:")
    print("    pip3 install --user pycryptodome")
    sys.exit(1)

# Define PX4 paths
PX4_MAIN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ENCRYPTED_LOGS_DIR = os.path.join(PX4_MAIN_DIR, "log/encrypted_logs")
DECRYPTED_LOGS_DIR = os.path.join(PX4_MAIN_DIR, "log/decrypted_logs")
DEFAULT_PRIVATE_KEY = os.path.join(PX4_MAIN_DIR, "key/private/private_key.pem")

def decrypt_log_file(ulog_file, ulog_key, rsa_key, output_folder):
    """Decrypts a single log file and saves it as .ulg in the output folder."""

    try:
        # Read the private RSA key
        with open(rsa_key, 'rb') as f:
            r = RSA.import_key(f.read())

        # Determine key data filename
        if ulog_key == "":
            key_data_filename = ulog_file
            magic = "ULogEnc"
        else:
            key_data_filename = ulog_key
            magic = "ULogKey"

        with open(key_data_filename, 'rb') as f:
            # Read the encrypted ChaCha key and the nonce
            ulog_key_header = f.read(22)

            # Parse the header
            if not ulog_key_header.startswith(bytearray(magic.encode())):
                print(f"Skipping {ulog_file}: Incorrect header magic")
                return
            if ulog_key_header[7] != 1:
                print(f"Skipping {ulog_file}: Unsupported header version")
                return
            if ulog_key_header[16] != 4:
                print(f"Skipping {ulog_file}: Unsupported key algorithm")
                return

            key_size = ulog_key_header[19] << 8 | ulog_key_header[18]
            nonce_size = ulog_key_header[21] << 8 | ulog_key_header[20]
            ulog_key_cipher = f.read(key_size)
            nonce = f.read(nonce_size)

        data_offset = 22 + key_size + nonce_size if magic == "ULogEnc" else 0

        # Try to decrypt the ChaCha key
        cipher_rsa = PKCS1_OAEP.new(r, SHA256)
        try:
            ulog_key = cipher_rsa.decrypt(ulog_key_cipher)
        except ValueError:
            print(f"Skipping {ulog_file}: Incorrect decryption (wrong key)")
            return

        # Read and decrypt the log data
        cipher = ChaCha20.new(key=ulog_key, nonce=nonce)

        # Save decrypted log with .ulg extension
        output_path = os.path.join(output_folder, Path(ulog_file).stem + ".ulg")
        with open(ulog_file, 'rb') as f:
            if data_offset > 0:
                f.seek(data_offset)
            with open(output_path, 'wb') as out:
                out.write(cipher.decrypt(f.read()))

        print(f"Decrypted log saved to: {output_path}")

    except Exception as e:
        print(f"Skipping {ulog_file}: Error occurred - {e}")

def decrypt_all_logs(private_key_path):
    """Decrypts all logs in the encrypted folder and saves them in the decrypted folder."""

    # Ensure output directory exists
    os.makedirs(DECRYPTED_LOGS_DIR, exist_ok=True)

    # Find all encrypted logs
    encrypted_logs = [f for f in os.listdir(ENCRYPTED_LOGS_DIR) if f.endswith(".ulge") or f.endswith(".ulgc")]

    if not encrypted_logs:
        print("No encrypted logs found.")
        return

    print(f"Found {len(encrypted_logs)} encrypted logs. Decrypting...")

    for log_file in encrypted_logs:
        log_path = os.path.join(ENCRYPTED_LOGS_DIR, log_file)
        log_key_path = log_path.replace(".ulge", ".ulgk").replace(".ulgc", ".ulgk")

        # If the key file does not exist, use an empty string
        if not os.path.exists(log_key_path):
            log_key_path = ""

        decrypt_log_file(log_path, log_key_path, private_key_path, DECRYPTED_LOGS_DIR)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Decrypt encrypted PX4 log files")
    parser.add_argument("private_key", nargs="?", default=None, help="Path to the private RSA key (.pem)")

    args = parser.parse_args()

    # If no key is provided, use the default private key
    private_key_path = args.private_key if args.private_key else DEFAULT_PRIVATE_KEY

    if not os.path.exists(private_key_path):
        print(f"Error: Private key file not found at {private_key_path}")
        sys.exit(1)

    decrypt_all_logs(private_key_path)
