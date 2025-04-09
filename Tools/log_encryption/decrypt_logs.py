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

PX4_MAIN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
ENCRYPTED_LOGS_DIR = os.path.join(PX4_MAIN_DIR, "logs/encrypted")
DECRYPTED_LOGS_DIR = os.path.join(PX4_MAIN_DIR, "logs/decrypted")
DEFAULT_PRIVATE_KEY = os.path.join(PX4_MAIN_DIR, "keys/private/private_key.pem")

def decrypt_log_file(ulog_file, private_key, output_folder):
    """Decrypts a single encrypted log file (.ulge) and saves it as .ulg in the output folder."""

    try:
        # Read the private RSA key
        with open(private_key, 'rb') as f:
            key = RSA.import_key(f.read())

        magic = "ULogEnc"
        header_size = 22

        with open(ulog_file, 'rb') as f:
            # Encrypted .ulge file contains following sections:
            # -------------------------
            # | Header                |
            # -------------------------
            # | Wrapped symmetric key |
            # -------------------------
            # | Encrypted ulog data   |
            # -------------------------
            header = f.read(header_size)

            # Parse the header
            if not header.startswith(bytearray(magic.encode())):
                print(f"Skipping {ulog_file}: Incorrect header magic")
                return
            if header[7] != 1:
                print(f"Skipping {ulog_file}: Unsupported header version")
                return
            if header[16] != 4:
                print(f"Skipping {ulog_file}: Unsupported key algorithm")
                return

            key_size = header[19] << 8 | header[18]
            nonce_size = header[21] << 8 | header[20]
            cipher = f.read(key_size)
            nonce = f.read(nonce_size)

        data_offset = header_size + key_size + nonce_size

        # Try to decrypt the ChaCha key
        cipher_rsa = PKCS1_OAEP.new(key, SHA256)
        try:
            ulog_key = cipher_rsa.decrypt(cipher)
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

        print(f"{output_path}")

    except Exception as e:
        print(f"Skipping {ulog_file}: Error occurred - {e}")


def decrypt_all_logs(private_key_path, log_source_path=None):
    """Decrypts all logs in the given folder or a single file."""

    if log_source_path and os.path.isfile(log_source_path):
        logs = [log_source_path]
    else:
        # Use default encrypted logs directory if not provided
        folder = log_source_path if log_source_path else ENCRYPTED_LOGS_DIR
        logs = [os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".ulge")]

    if not logs:
        print("No encrypted logs found.")
        return

    print(f"Found {len(logs)} encrypted log(s). Decrypting...")

    os.makedirs(DECRYPTED_LOGS_DIR, exist_ok=True)

    for log_path in logs:
        decrypt_log_file(log_path, private_key_path, DECRYPTED_LOGS_DIR)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
    description="Decrypt PX4 encrypted log files (.ulge) using a ChaCha20+RSA scheme.\n\n"
                "Usage examples:\n"
                "  python3 decrypt_logs.py /path/to/private_key.pem /path/to/custom_log.ulge\n"
                "  python3 decrypt_logs.py /path/to/private_key.pem /path/to/folder_with_ulge_files\n"
                "  python3 decrypt_logs.py                  # Uses default key + default log folder\n",
    formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument("private_key", nargs="?", default=None,
                        help="Path to the private RSA key (.pem). If omitted, uses default key.")
    parser.add_argument("log_file_or_folder", nargs="?", default=None,
                        help="Path to a single .ulge file or folder containing them. If omitted, uses default encrypted log folder.")


    args = parser.parse_args()

    private_key_path = args.private_key if args.private_key else DEFAULT_PRIVATE_KEY

    if not os.path.exists(private_key_path):
        print(f"Error: Private key file not found at {private_key_path}")
        sys.exit(1)

    decrypt_all_logs(private_key_path, args.log_file_or_folder)
