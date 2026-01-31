# PX4 Log Encryption Tools

Tools for generating encryption keys, building PX4 firmware with encrypted logs, downloading logs, and decrypting them.

For more information see: https://docs.px4.io/main/en/dev_log/log_encryption.html

## Usage

1. **Get the board file**:
   In order to use these tools you need to create an `encrypted_logs` target in your target board directory. For example:
   ```bash
   encrypted_logs.px4board
   ```
   Using `make menuconfig` you should enable these settings: `Blake2s hash algorithm`, `entropy pool` and `strong random number generator` and select `use interrupts` to feed timing randomness to the entropy pool.
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

   FOR INSTANCE
   make_ark_fpv_encrypted_logs

   Upload the custom firmware on your flight controller and record some logs
   ```

4. **Download Logs**:
   ```bash
   cd PX4-Autopilot/Tools/log_encryption

   python3 download_logs.py /dev/ttyACM0 --baudrate 57600

   OR

   python3 download_logs.py udp:0.0.0.0:14550
   ```

   Addresses might need to be adjusted

5. **Decrypt Logs**:
   The easiest way to run this is to have your private key and encrypted logs in the following folders respectively:
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
   # Use --help to get all the options
   python3 decrypt_logs.py --help
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
