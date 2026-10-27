#!/usr/bin/env python3
"""Exercise the real Roboclaw driver against two packet-serial controllers on a PTY.

Requires a native PX4 SITL binary built with CONFIG_DRIVERS_ROBOCLAW=y and
cmake -DNOLOCKSTEP=ON (no simulator is started by this test).
Run: python3 src/drivers/roboclaw/tests/serial_integration.py <path/to/px4>
No physical serial devices or motors are used.
"""

import errno
import os
from pathlib import Path
import pty
import re
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty


def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


class Bus:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.addresses = {128, 129}
        self.packets = []
        self.errors = []
        self.corrupt = False
        self.stop = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buffer = bytearray()
        while not self.stop:
            if not select.select([self.master], [], [], 0.05)[0]:
                continue
            try:
                buffer.extend(os.read(self.master, 4096))
            except OSError as error:
                if error.errno != errno.EIO:
                    self.errors.append(str(error))
                continue
            while len(buffer) >= 2:
                address, command = buffer[:2]
                size = 5 if command in (0, 1, 4, 5) else 2
                if len(buffer) < size:
                    break
                packet = bytes(buffer[:size])
                del buffer[:size]
                self.packets.append(packet)
                if address not in self.addresses:
                    continue
                if size == 5:
                    if crc16(packet[:-2]) != int.from_bytes(packet[-2:], 'big'):
                        self.errors.append(f'Invalid command CRC: {packet.hex()}')
                    os.write(self.master, b'\xff')
                    continue
                if command == 90:
                    payload = bytes(4)
                elif command in (18, 19):
                    payload = struct.pack('>IB', 1200, int(command == 19))
                elif command == 78:
                    payload = struct.pack('>ii', 1200 + address - 128, -2400)
                else:
                    self.errors.append(f'Unknown command {command}')
                    continue
                checksum = crc16(packet + payload) ^ int(self.corrupt)
                reply = payload + struct.pack('>H', checksum)
                # Exercise fragmented reads, rather than only complete responses.
                os.write(self.master, reply[:1])
                time.sleep(0.001)
                os.write(self.master, reply[1:])


class Console:
    def __init__(self, binary, directory):
        script = Path(directory) / 'startup.sh'
        script.write_text('')
        self.output = ''
        self.process = subprocess.Popen(
            [str(binary), '-s', str(script), '-w', directory, '-i', str(os.getpid())],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()
        time.sleep(1)

    def read(self):
        while True:
            char = self.process.stdout.read(1)
            if not char:
                break
            self.output += char

    def command(self, command, timeout=5):
        start = len(self.output)
        self.process.stdin.write(command + '\n')
        self.process.stdin.flush()
        # pxh prompt is emitted after processing each command.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            output = self.output[start:]
            command_end = output.find(command + '\n')
            if command_end >= 0 and 'pxh> ' in output[command_end + len(command) + 1:]:
                return output
            if self.process.poll() is not None:
                raise AssertionError(f'PX4 exited: {self.output}')
            time.sleep(0.01)
        raise AssertionError(f'Command timed out: {command}\n{self.output[start:]}')


def main():
    binary = Path(sys.argv[1]).resolve()
    bus = Bus()
    with tempfile.TemporaryDirectory(prefix='roboclaw-test-') as directory:
        console = Console(binary, directory)
        try:
            console.command(f'param select {directory}/parameters.bson')
            console.command('param set RBCLW_COUNT 2')
            console.command('param set RBCLW_FUNC1 101')
            console.command('param set RBCLW_FUNC2 102')
            console.command('param set RBCLW_FUNC3 103')
            console.command('param set RBCLW_FUNC4 104')
            result = console.command(f'roboclaw start {bus.path}')
            assert 'Task start failed' not in result, result
            time.sleep(0.5)
            status = console.command('roboclaw status')
            assert 'Communication errors: 0' in status, status
            assert 'Address 0x80' in status and 'Address 0x81' in status, status
            assert status.count('min: 127, max: 254') == 4, status
            samples = re.findall(r'encoder updates: (\d+)', status)
            assert len(samples) == 2 and all(int(v) > 0 for v in samples), status
            encoders = console.command('listener wheel_encoders')
            assert '6.283' in encoders and '-6.283' in encoders and '-12.566' in encoders, encoders
            for address in (128, 129):
                for motor_command in (0, 4):
                    assert any(p[0] == address and p[1] == motor_command and p[2] == 0
                               for p in bus.packets if len(p) == 5), (address, motor_command)
            assert all(p[2] == 0 for p in bus.packets if len(p) == 5), bus.packets
            # Corrupt responses must be rejected rather than published as telemetry.
            bus.corrupt = True
            time.sleep(0.3)
            status = console.command('roboclaw status')
            assert re.search(r'Communication errors: [1-9]', status), status
            bus.corrupt = False
            console.command('roboclaw stop')
            assert 'not running' in console.command('roboclaw status')
            # Failed probes and repeated starts/stops must release the port and work queue.
            bus.addresses = {128}
            start = len(console.output)
            console.command(f'roboclaw start {bus.path}')
            time.sleep(0.4)
            assert 'No valid response from address 0x81' in console.output[start:], console.output[start:]
            bus.addresses = {128, 129}
            for _ in range(3):
                result = console.command(f'roboclaw start {bus.path}')
                time.sleep(0.2)
                assert 'Task start failed' not in result, result
                console.command('roboclaw stop')
            console.command('param set RBCLW_ADDRESS 135')
            start = len(console.output)
            console.command(f'roboclaw start {bus.path}')
            time.sleep(0.2)
            assert 'Invalid controller count' in console.output[start:], console.output[start:]
            assert not bus.errors, bus.errors
            print('PASS: two addresses, fragmented replies, signed encoder speed/position, zero outputs, CRC rejection, failed probe, restart, invalid address range')
        finally:
            if console.process.poll() is None:
                try:
                    console.command('shutdown', timeout=1)
                except (AssertionError, BrokenPipeError):
                    pass
                if console.process.poll() is None:
                    console.process.terminate()
                console.process.wait(timeout=5)
            bus.stop = True
            bus.thread.join(timeout=1)
            os.close(bus.master)
            os.close(bus.slave)


if __name__ == '__main__':
    main()
