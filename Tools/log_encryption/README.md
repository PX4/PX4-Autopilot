# PX4 Log Encryption Tools

   Tools for generating encryption keys, building PX4 firmware with encrypted logs, downloading logs, and decrypting them.

## Usage

1. **Get the board file**:
   To be able to use these methode you need to create your own board file in your board folder. You can just copy one that is used below.
   ```bash
   encrypted_logs.px4board
   ```
   Then using menuconfig you should Enable the settings: Blake2s hash algorithm, Entropy pool and strong random number generator, and Use interrupts to feed timing randomness to entropy pool.
   Once you have generated the keys make sure you add them to the boardconfig.

   ```bash
   make <your_board_name>_encrypted_logs menuconfig
   ```

2. **Generate Keys**:
   ```bash
   cd PX4-Autopilot/Tools/log_encryption
   python3 generate_keys.py
   ```

   Make sure you have the right key in your board file
   ```CONFIG_PUBLIC_KEY1="../../../keys/public/public_key.pub"```

3. **Build Firmware**:
   ```bash
   cd PX4-Autopilot

   AND

   make <your_board_name>_encrypted_logs

   OR

   make ark_fmu-v6x_encrypted_logs

   OR

   make ark_fpv_encrypted_logs

   OR

   make ark_pi6x_encrypted_logs

   AND

   Upload the custom fw on your flight controller and record couple of logs
   ```

4. **Download Logs**:
   ```bash
   cd PX4-Autopilot/Tools/log_encryption

   python3 download_logs.py serial:/dev/ttyACM0:57600

   OR

   python3 download_logs.py udp:0.0.0.0:14550
   ```

   Addresses might need to be adjusted

5. **Decrypt Logs**:
   The easiest way to run this is to have your private key and encrypted logs in te following folders respectively:
   ```bash
   PX4-Autopilot/keys/private
   PX4-Autopilot/logs/encrypted
   ```
   Then run:
   ```bash
   cd PX4-Autopilot/Tools/log_encryption

   AND
   # Uses default key + default folder
   python3 decrypt_logs.py

   OR
   # Uses custom key
   python3 decrypt_logs.py path/to/private_key.pem

   OR
   # Uses custom key + specific file
   python3 decrypt_logs.py path/to/private_key.pem path/to/custom_log.ulge

   OR
   # Uses custom key + specific folder
   python3 decrypt_logs.py path/to/private_key.pem path/to/custom_log_folder

   OR
   # Uses default key + specific file
   python3 decrypt_logs.py "" path/to/custom_log.ulge

   OR
   # Uses default key + specific folder
   python3 decrypt_logs.py "" path/to/custom_log_folder
   ```

   Your decrypted logs can be found in:
   ```bash
   PX4-Autopilot/logs/decrypted
   ```
   Otherwise

## Directory Structure

- **`keys/`**: Encryption keys.
- **`logs/encrypted/`**: Downloaded encrypted logs.
- **`logs/decrypted/`**: Decrypted logs.
