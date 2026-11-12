#!/usr/bin/env python3
"""
Virtual COM port bridge via MAVLink SERIAL_CONTROL.

Creates a virtual serial port (PTY) and forwards all traffic bidirectionally
through MAVLink SERIAL_CONTROL messages to a PX4 serialpassthrough module.
See docs/en/uart/serial_passthrough.md for the protocol this implements.

Usage:
    Tools/mavlink_serial_bridge.py --connection tcp:10.41.1.1:5790 --port esc0 --setup
    Tools/mavlink_serial_bridge.py --connection udpin:0.0.0.0:14550 --port telem2 --port-baud 115200
    Tools/mavlink_serial_bridge.py --connection udpin:0.0.0.0:14550 --port gps1 --port-baud 9600
    Tools/mavlink_serial_bridge.py --connection udpin:0.0.0.0:14550 --port gps2 --port-baud 57600

Add --verbose/-v to print a hex dump of every forwarded packet.
"""

import argparse
import os
import pty
import select
import struct
import sys
import tty
import threading
import time

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)

# SERIAL_CONTROL_FLAG bitmask values
SERIAL_CONTROL_FLAG_REPLY     = 1
SERIAL_CONTROL_FLAG_RESPOND   = 2
SERIAL_CONTROL_FLAG_EXCLUSIVE = 4

MAX_PAYLOAD = 70

# pymavlink is not thread-safe; hold this for every mav access from the bridge threads
MAV_LOCK = threading.Lock()

PORT_MAP = {
    'telem1': 0,
    'telem2': 1,
    'gps1': 2,
    'gps2': 3,
    'telem3': 4,
    'telem4': 5,
    'esc0': 20,
    'esc1': 21,
    'esc2': 22,
    'esc3': 23,
    'esc4': 24,
    'esc5': 25,
    'esc6': 26,
    'esc7': 27,
}


def send_serial_control(mav, device, port_baud, data=b''):
    """Send a SERIAL_CONTROL message with the RESPOND|EXCLUSIVE flags used throughout the bridge."""
    # pymavlink needs all 70 entries; MAVLink 2 trims trailing zeros on the wire
    payload = list(data) + [0] * (MAX_PAYLOAD - len(data))
    with MAV_LOCK:
        mav.mav.serial_control_send(
            device=device,
            flags=SERIAL_CONTROL_FLAG_RESPOND | SERIAL_CONTROL_FLAG_EXCLUSIVE,
            timeout=0,
            baudrate=port_baud,
            count=len(data),
            data=payload,
        )


def setup_passthrough(mav):
    """
    Set PASSTHRU_EN=1 via PARAM_SET and reboot the FMU.
    Waits for the FMU to come back online before returning.
    """
    param_id = b'PASSTHRU_EN'
    param_id_padded = param_id[:16].ljust(16, b'\x00')

    print("Setting PASSTHRU_EN=1...")
    mav.mav.param_set_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        param_id=param_id_padded,
        param_value=struct.unpack('f', struct.pack('<i', 1))[0],
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    # Wait for PARAM_VALUE confirmation (up to 3 s)
    ack = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if ack:
        print(f"PARAM_VALUE ack: {ack.param_id.rstrip(chr(0))} = {ack.param_value}")
    else:
        print("WARNING: no PARAM_VALUE ack received, continuing anyway")

    print("Sending reboot command (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)...")
    mav.mav.command_long_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        confirmation=0,
        param1=1,  # reboot autopilot
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    print("Reboot command sent. Waiting for FMU to come back online...")
    time.sleep(2)  # Give the FMU time to start rebooting
    # Overall deadline: heartbeats from other components must not keep the wait alive
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=deadline - time.monotonic())
        if hb and hb.get_srcSystem() == mav.target_system and hb.get_srcComponent() == mav.target_component:
            break
    else:
        raise TimeoutError("FMU did not come back online within 30 seconds")
    print("FMU back online")


class SerialBridge:
    """Forwards bytes between a PTY and SERIAL_CONTROL messages for one FMU device."""

    def __init__(self, mav, master_fd, device, port_baud, verbose=False):
        self.mav = mav
        self.master_fd = master_fd
        self.device = device
        self.port_baud = port_baud
        self.verbose = verbose
        self.stop = threading.Event()

    def pty_to_mavlink(self):
        """Read from PTY master, forward as SERIAL_CONTROL messages."""
        try:
            while not self.stop.is_set():
                try:
                    data = os.read(self.master_fd, MAX_PAYLOAD)
                except OSError:
                    break
                if self.stop.is_set():  # wake-up byte from shutdown, don't forward
                    break
                if not data:
                    continue
                send_serial_control(self.mav, self.device, self.port_baud, data)
                if self.verbose:
                    print(f"  PTY -> MAVLink: {len(data)} bytes: {data.hex(' ')}")
        except Exception as e:
            print(f"ERROR: pty_to_mavlink crashed: {e}", file=sys.stderr)
        finally:
            self.stop.set()

    def mavlink_to_pty(self):
        """Receive SERIAL_CONTROL FLAG_REPLY messages, write immediately to PTY."""
        try:
            while not self.stop.is_set():
                with MAV_LOCK:
                    msg = self.mav.recv_match(type='SERIAL_CONTROL', blocking=False)
                if msg is None:
                    # Nothing buffered (locally or at the OS level) right now —
                    # block until the connection's fd has more data.
                    select.select([self.mav.fd], [], [], 0.2)
                    continue
                if not (msg.flags & SERIAL_CONTROL_FLAG_REPLY):
                    continue

                if msg.device != self.device:
                    if self.verbose:
                        print(f"  Ignoring message for device {msg.device} (current device: {self.device})")
                    # Stale reply for the previous ESC channel — discard.
                    continue
                reply = bytes(msg.data[:msg.count])
                if reply:
                    os.write(self.master_fd, reply)
                    if self.verbose:
                        print(f"  MAVLink -> PTY: {msg.count} bytes: {reply.hex(' ')}")
        except Exception as e:
            print(f"ERROR: mavlink_to_pty crashed: {e}", file=sys.stderr)
        finally:
            self.stop.set()

    def stdin_listener(self):
        """Read SWITCH <device_id> commands from stdin to hot-swap ESC channel."""
        try:
            for line in sys.stdin:
                line = line.strip()
                if line.startswith('SWITCH '):
                    try:
                        new_device = int(line.split()[1])
                    except (IndexError, ValueError):
                        continue
                    self.device = new_device
                    print(f"  Switching to device {self.device}")
                    send_serial_control(self.mav, self.device, self.port_baud)
        except Exception as e:
            print(f"ERROR: stdin_listener crashed: {e}", file=sys.stderr)
        finally:
            self.stop.set()


def run_bridge(connection_str, baud, device, port_baud, setup=False, verbose=False):
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)

    # Set PTY to raw mode — prevents line discipline from transforming/buffering binary data
    tty.setraw(master_fd)
    tty.setraw(slave_fd)

    print(f"Virtual COM port: {slave_path}", flush=True)

    mav = mavutil.mavlink_connection(connection_str, baud=baud)
    print(f"Waiting for heartbeat on {connection_str}...")
    while True:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True)
        if hb and hb.get_srcSystem() != 0 and hb.get_srcComponent() == 1:
            mav.target_system = hb.get_srcSystem()
            mav.target_component = hb.get_srcComponent()
            break
    print(f"Heartbeat received (sysid={mav.target_system}, compid={mav.target_component})")

    if setup:
        setup_passthrough(mav)

    # Send an init message with count=0 to trigger FMU-side startForDevice()
    # before any data arrives. baudrate field carries the target UART baud rate.
    print(f"Initializing FMU passthrough: device={device}, port_baud={port_baud}...")
    send_serial_control(mav, device, port_baud)
    time.sleep(2)  # Give FMU time to spawn the task
    print("Bridge running. Press Ctrl+C to stop.\n", flush=True)

    bridge = SerialBridge(mav, master_fd, device, port_baud, verbose)
    threads = [threading.Thread(target=fn, daemon=True)
               for fn in (bridge.pty_to_mavlink, bridge.mavlink_to_pty, bridge.stdin_listener)]
    for t in threads:
        t.start()

    # stop.wait() only returns without a KeyboardInterrupt if a thread crashed.
    crashed = True
    try:
        bridge.stop.wait()  # blocks until Ctrl+C (KeyboardInterrupt) or a thread crash
    except KeyboardInterrupt:
        print("\nShutting down.")
        crashed = False
    finally:
        bridge.stop.set()
        try: os.write(slave_fd, b'\x00')
        except OSError: pass
        try: os.close(slave_fd)
        except OSError: pass
        try: os.close(master_fd)
        except OSError: pass
        for t in threads:
            t.join(timeout=0.5)
        mav.close()

    if crashed:
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Virtual COM port bridge via MAVLink SERIAL_CONTROL')
    parser.add_argument('--connection', default='udpin:0.0.0.0:14550',
                        help='MAVLink connection string (default: udpin:0.0.0.0:14550)')
    parser.add_argument('--baud', type=int, default=57600,
                        help='Serial baud rate if using serial connection (default: 57600)')
    parser.add_argument('--port', default='esc0',
                        choices=list(PORT_MAP.keys()),
                        help='Target port on the FMU (default: esc0). SERIAL_CONTROL device IDs: '
                             + ', '.join(f'{name}={dev}' for name, dev in PORT_MAP.items()))
    parser.add_argument('--port-baud', type=int, default=115200,
                        help='Baudrate to set on the target UART (default: 115200; ignored for esc ports, which use 19200)')
    parser.add_argument('--setup', action='store_true', default=False,
                        help='Set PASSTHRU_EN=1 and reboot FMU before starting bridge')
    parser.add_argument('--verbose', '-v', action='store_true', default=False,
                        help='Print a hex dump of every forwarded packet (noisy, disabled by default)')
    args = parser.parse_args()

    device = PORT_MAP[args.port]
    port_baud = 19200 if args.port.startswith('esc') else args.port_baud
    print(f"Port: {args.port} (device ID {device}), UART baud: {port_baud}")

    run_bridge(
        connection_str=args.connection,
        baud=args.baud,
        device=device,
        port_baud=port_baud,
        setup=args.setup,
        verbose=args.verbose,
    )
