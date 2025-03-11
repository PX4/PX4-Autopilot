# PX4 Log Encryption Tools

Tools for generating encryption keys, building PX4 firmware with encrypted logs, downloading logs, and decrypting them.

## Prerequisites

- Python 3.x
- pip3 install --user pycryptodome pymavlink
- PX4 development environment

## Usage



1. **Generate Keys**:
   ```bash
   cd PX4-Autopilot/Tools/log_encryption
   python3 generate_keys.py
   ```

2. **Build Firmware**:
   ```bash
   cd PX4-Autopilot

   AND

   make ark_fmu-v6x_encrypted_logs

   OR

   make ark_fpv_encrypted_logs

   OR

   make ark_pi6x_encrypted_logs

   AND

   Upload the custom fw on your flight controller and record couple of logs
   ```

3. **Download Logs**:
   ```bash
   cd PX4-Autopilot/Tools/log_encryption
   mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
   python3 download_logs.py serial:/dev/ttyACM0:57600
   ```
   For some reason running the mavproxy is neccessary, also udp connection was not tested yet
   Also your port might need to be adjusted and QGC closed

4. **Decrypt Logs**:
   ```bash
   cd PX4-Autopilot/Tools/log_encryption
   python3 decrypt_logs.py
   ```



## Directory Structure

- **`keys/`**: Encryption keys.
- **`encrypted_logs/`**: Downloaded encrypted logs.
- **`decrypted_logs/`**: Decrypted logs.


