import os
import subprocess

# Define the main PX4 directory (one level up from Tools)
PX4_MAIN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

# Define the key folder paths
KEY_FOLDER = os.path.join(PX4_MAIN_DIR, "keys")
PUBLIC_KEY_FOLDER = os.path.join(KEY_FOLDER, "public")
PRIVATE_KEY_FOLDER = os.path.join(KEY_FOLDER, "private")

# Define the key file paths
PRIVATE_KEY_PATH = os.path.join(PRIVATE_KEY_FOLDER, "private_key.pem")
PUBLIC_KEY_DER_PATH = os.path.join(PUBLIC_KEY_FOLDER, "public_key.der")
PUBLIC_KEY_PUB_PATH = os.path.join(PUBLIC_KEY_FOLDER, "public_key.pub")

def create_key_folders():
    """Creates key, public, and private folders if they do not exist."""
    for folder in [KEY_FOLDER, PUBLIC_KEY_FOLDER, PRIVATE_KEY_FOLDER]:
        if not os.path.exists(folder):
            os.makedirs(folder)
            print(f"Created '{folder}' directory.")
        else:
            print(f"'{folder}' directory already exists.")

def generate_private_key():
    """Generates a private key if it does not exist."""
    if not os.path.exists(PRIVATE_KEY_PATH):
        print("Generating private key...")
        subprocess.run(["openssl", "genpkey", "-algorithm", "RSA", "-out", PRIVATE_KEY_PATH, "-pkeyopt", "rsa_keygen_bits:2048"])
        print(f"Private key generated at: {PRIVATE_KEY_PATH}")
    else:
        print("Private key already exists.")

def generate_public_key():
    """Generates a public key in DER and PUB formats if they do not exist."""
    if not os.path.exists(PUBLIC_KEY_DER_PATH):
        print("Generating public key in DER format...")
        subprocess.run(["openssl", "rsa", "-pubout", "-in", PRIVATE_KEY_PATH, "-outform", "DER", "-out", PUBLIC_KEY_DER_PATH])
        print(f"Public key (DER) generated at: {PUBLIC_KEY_DER_PATH}")
    else:
        print("Public key (DER) already exists.")

    if not os.path.exists(PUBLIC_KEY_PUB_PATH):
        print("Generating public key in hex format...")
        with open(PUBLIC_KEY_PUB_PATH, "w") as pub_file:
            process = subprocess.Popen(["xxd", "-p", PUBLIC_KEY_DER_PATH], stdout=subprocess.PIPE)
            output, _ = process.communicate()
            hex_string = output.decode().strip().replace("\n", "")
            formatted_hex = ", ".join(f"0x{hex_string[i:i+2]}" for i in range(0, len(hex_string), 2))
            pub_file.write(formatted_hex)
        print(f"Public key (hex) generated at: {PUBLIC_KEY_PUB_PATH}")
    else:
        print("Public key (hex) already exists.")

if __name__ == "__main__":
    create_key_folders()
    generate_private_key()
    generate_public_key()
