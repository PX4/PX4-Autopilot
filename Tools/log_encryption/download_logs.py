#!/usr/bin/env python3

import os
import sys
import time
import shutil
import threading
from pymavlink import mavutil
from argparse import ArgumentParser

class MavlinkLogDownloader:
    def __init__(self, connection_url, output_dir, baudrate=57600, source_system=254):
        self.connection_url = connection_url
        self.output_dir = output_dir
        self.encrypted_dir = os.path.join(output_dir, "encrypted")
        self.running = True

        # Ensure directories exist
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.encrypted_dir, exist_ok=True)

        # # Handle serial or UDP connections
        if os.path.exists(connection_url):  # likely a serial device
            self.mav = mavutil.mavlink_connection(connection_url, baud=baudrate, source_system=source_system)
        else:
            self.mav = mavutil.mavlink_connection(connection_url, source_system=source_system)


        self.mav.WIRE_PROTOCOL_VERSION = "2.0"

        # Start heartbeat thread
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat_thread)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()

        self.mav.wait_heartbeat()
        print(f"Heartbeat received from system {self.mav.target_system}, component {self.mav.target_component}")

        # Waking up the autopilot, it is needed to ensure we get answer for log request
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command ID 512
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,  # param1: Message ID 148
            0, 0, 0, 0, 0, 0  # params 2–7 are not used for this message
        )

        # Allow heartbeats to establish connection
        time.sleep(3)


    def send_heartbeat_thread(self):
        while self.running:
            self.mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, 0
            )
            time.sleep(1)


    def download_logs(self):
        """Downloads logs to the output_dir."""
        print("Request logs...")
        self.mav.mav.log_request_list_send(self.mav.target_system, self.mav.target_component, 0, 0xFFFF)

        log_entries = {}
        total_logs = None
        start_time = time.time()
        last_entry_time = None

        while True:
            current_time = time.time()

            # Case 1: If we haven't received any entries yet and we've waited more than 5 seconds
            if not log_entries and current_time > start_time + 5:
                print("Timed out waiting for first log entry (5s)")
                break

            # Case 2: If we have received at least one entry and it's been more than 3 seconds since the last one
            if last_entry_time and current_time - last_entry_time > 3:
                print(f"No new log entries received for 3 seconds. Moving on.")
                break

            # Case 3: If we've received all expected logs
            if total_logs is not None and len(log_entries) >= total_logs:
                print(f"Received all {total_logs} log entries.")
                break

            msg = self.mav.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)

            if msg and msg.id not in log_entries:
                last_entry_time = time.time()
                log_entries[msg.id] = msg

                if total_logs is None:
                    total_logs = msg.num_logs

                print(f"Log ID: {msg.id}, Size: {msg.size} bytes, Date: {msg.time_utc} ({len(log_entries)}/{total_logs})")

        if not log_entries:
            print("No log entries found.")
            return

        for entry in log_entries.values():
            self.download_log_file(entry)

        self.classify_logs()


    def download_log_file(self, log_entry):
        """Downloads a log file to the output_dir."""
        log_id = log_entry.id
        log_size = log_entry.size
        log_date = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime(log_entry.time_utc))
        output_filename = os.path.join(self.output_dir, f"log-{log_date}_{log_id}.ulg")

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
                else:
                    print("Timeout waiting for log data.")
                    break

        print(f"Finished downloading log {log_id}.")

    def classify_logs(self):
        """Classifies logs as encrypted (.ulge) based on file content."""
        for log_file in os.listdir(self.output_dir):
            log_path = os.path.join(self.output_dir, log_file)

            if not os.path.isfile(log_path):
                continue

            # Read first 10 bytes to check for "ULogEnc"
            with open(log_path, 'rb') as f:
                first_bytes = f.read(10)

            if b'ULogEnc' in first_bytes:
                new_filename = log_file.replace(".ulg", ".ulge")
                new_path = os.path.join(self.encrypted_dir, new_filename)
                print(f"Found encrypted log: {new_path}")
                shutil.move(log_path, new_path)


    def cleanup(self):
        """Stop the heartbeat thread and clean up resources."""
        self.running = False
        if self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2.0)


def main():
    parser = ArgumentParser(description="Download PX4 log files over MAVLink.")
    parser.add_argument('connection_url', help="MAVLink connection URL (e.g., udp:0.0.0.0:14550, /dev/ttyACM0 --baudrate 57600)")
    parser.add_argument('--output', '-o', default=os.path.join(os.path.dirname(__file__), "../..", "logs"), help="Output directory for log files (default: ../../logs)")
    parser.add_argument('--baudrate', type=int, default=57600, help="Baudrate for serial connection (default: 57600)")
    parser.add_argument('--source-system', type=int, default=254, help="MAVLink source system ID (default: 254)")


    args = parser.parse_args()

    output_dir = os.path.abspath(args.output)

    print(f"Connecting to {args.connection_url}...")
    log_downloader = MavlinkLogDownloader(
        args.connection_url,
        output_dir,
        baudrate=args.baudrate,
        source_system=args.source_system
    )


    try:
        log_downloader.download_logs()
    finally:
        log_downloader.cleanup()

if __name__ == '__main__':
    main()
