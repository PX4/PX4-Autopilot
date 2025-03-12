#!/usr/bin/env python3

import os
import sys
import time
import shutil
from pymavlink import mavutil
from argparse import ArgumentParser

class MavlinkLogDownloader:
    def __init__(self, connection_url, output_dir):
        self.connection_url = connection_url
        self.output_dir = output_dir

        # Define directories
        self.tmp_dir = os.path.join(output_dir, "tmp")
        self.decrypted_logs_dir = os.path.join(output_dir, "decrypted_logs")
        self.encrypted_logs_dir = os.path.join(output_dir, "encrypted_logs")

        # Ensure directories exist
        os.makedirs(self.tmp_dir, exist_ok=True)
        os.makedirs(self.decrypted_logs_dir, exist_ok=True)
        os.makedirs(self.encrypted_logs_dir, exist_ok=True)

        # Handle serial or UDP connections
        if connection_url.startswith("serial:"):
            parts = connection_url.split(":")
            if len(parts) != 3:
                raise ValueError("Invalid serial connection URL. Use format: serial:/dev/ttyACM1:57600")
            device, baudrate = parts[1], int(parts[2])
            self.mav = mavutil.mavlink_connection(device, baud=baudrate)
        else:
            self.mav = mavutil.mavlink_connection(connection_url)

        self.mav.wait_heartbeat()
        print(f"Heartbeat received from system {self.mav.target_system}, component {self.mav.target_component}")

    def download_logs(self):
        """Downloads logs to the temporary folder."""
        self.mav.mav.log_request_list_send(self.mav.target_system, self.mav.target_component, 0, 0xFFFF)

        log_entries = {}  # Use a dictionary to store unique log entries
        start_time = time.time()

        while time.time() - start_time < 5:  # Wait for log entries (5s)
            msg = self.mav.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
            if msg and msg.id not in log_entries:
                log_entries[msg.id] = msg  # Store the log entry with its ID
                print(f"Log ID: {msg.id}, Size: {msg.size} bytes, Date: {msg.time_utc}")

        if not log_entries:
            print("No log entries found.")
            return

        for entry in log_entries.values():
            self.download_log_file(entry)  # Download logs only once per unique ID

        # After downloading, classify logs as encrypted or decrypted
        self.classify_logs()


    def download_log_file(self, log_entry):
        """Downloads a log file to the temporary folder."""
        log_id = log_entry.id
        log_size = log_entry.size
        log_date = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime(log_entry.time_utc))
        output_filename = os.path.join(self.tmp_dir, f"log-{log_date}_{log_id}.ulg")

        print(f"Downloading log {log_id} ({log_size} bytes) to {output_filename}...")

        with open(output_filename, 'wb') as f:
            self.mav.mav.log_request_data_send(self.mav.target_system, self.mav.target_component, log_id, 0, 0xFFFFFFFF)

            bytes_received = 0
            while bytes_received < log_size:
                msg = self.mav.recv_match(type='LOG_DATA', blocking=True, timeout=5)
                if msg:
                    data_bytes = bytes(msg.data[:msg.count])
                    f.write(data_bytes)
                    bytes_received += msg.count
                #     print(f"Received {bytes_received} of {log_size} bytes...")
                else:
                    print("Timeout waiting for log data.")
                    break

        print(f"Finished downloading log {log_id}.")

    def classify_logs(self):
        """Classifies logs as encrypted (.ulge) or decrypted (.ulg) based on file content."""
        for log_file in os.listdir(self.tmp_dir):
            log_path = os.path.join(self.tmp_dir, log_file)

            if not os.path.isfile(log_path):
                continue

            # Read first 10 bytes to check for "ULogEnc" (encrypted) or "ULog" (decrypted)
            with open(log_path, 'rb') as f:
                first_bytes = f.read(10)

            if b'ULogEnc' in first_bytes:
                new_filename = log_file.replace(".ulg", ".ulge")
                new_path = os.path.join(self.encrypted_logs_dir, new_filename)
                print(f"Encrypted log detected: Moving {log_file} to {self.encrypted_logs_dir} as {new_filename}")
            elif b'ULog' in first_bytes:
                new_path = os.path.join(self.decrypted_logs_dir, log_file)
                print(f"Decrypted log detected: Moving {log_file} to {self.decrypted_logs_dir}")
            else:
                print(f"WARNING: Could not determine log type for {log_file}, assuming encrypted.")
                new_filename = log_file.replace(".ulg", ".ulge")
                new_path = os.path.join(self.encrypted_logs_dir, new_filename)

            shutil.move(log_path, new_path)

        print("Log classification complete.")

def main():
    parser = ArgumentParser(description="Download PX4 log files over MAVLink.")
    parser.add_argument('connection_url', help="MAVLink connection URL (e.g., udp:127.0.0.1:14550, serial:/dev/ttyACM1:57600)")
    parser.add_argument('--output', '-o', default=os.path.join(os.path.dirname(__file__), "../..", "log"), help="Output directory for log files (default: ../../log)")

    args = parser.parse_args()

    output_dir = os.path.abspath(args.output)
    if not os.path.isdir(output_dir):
        print(f"Output directory {output_dir} does not exist.")
        return

    print(f"Connecting to {args.connection_url}...")
    log_downloader = MavlinkLogDownloader(args.connection_url, output_dir)
    log_downloader.download_logs()

if __name__ == '__main__':
    main()
