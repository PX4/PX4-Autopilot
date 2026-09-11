#!/usr/bin/env python3
"""
Open an NSH shell on a DroneCAN node over uavcan.protocol.AccessCommandShell.
"""

import argparse
import logging
import os
import select
import sys
import termios
import time
import dronecan

REQ_RESET_SHELL = 1
REQ_READ_OUTPUT = 64 | 128
RSP_PENDING_OUTPUT = 64 | 128

MAX_INPUT = 128
TIMEOUT = 1.0
POLL_INTERVAL = 0.2
SCAN_TIME = 5.0


def spin(node, timeout=0.02):
    """Wrapper around node.spin() that ignores TransferError exceptions"""
    try:
        node.spin(timeout=timeout)
    except dronecan.transport.TransferError:
        pass


def scan_nodes(node):
    """Returns {node_id: name} for the nodes heard from during the scan."""
    print(f"Scanning the bus ({SCAN_TIME:.0f} s)...")
    nodes = {}
    handler = node.add_handler(dronecan.uavcan.protocol.NodeStatus,
                               lambda e: nodes.setdefault(e.transfer.source_node_id, '?'))
    deadline = time.monotonic() + SCAN_TIME

    while time.monotonic() < deadline:
        spin(node, 0.1)

    handler.remove()
    nodes.pop(node.node_id, None)

    # ask everyone for its name at once, then wait out a single timeout
    for node_id in nodes:
        node.request(dronecan.uavcan.protocol.GetNodeInfo.Request(), node_id,
                     lambda e, nid=node_id: e and nodes.__setitem__(nid, e.response.name.decode()),
                     timeout=TIMEOUT)

    deadline = time.monotonic() + TIMEOUT + 0.5

    while time.monotonic() < deadline:
        spin(node, 0.05)

    return nodes


def choose_node(nodes):
    for node_id, name in sorted(nodes.items()):
        print(f"  {node_id}: {name}")

    while True:
        choice = input("node id: ").strip()

        if choice.isdigit() and int(choice) in nodes:
            return int(choice)

        print("pick one from the list")


def terminal(node, target_node_id):
    pending = [(b'\n', 0)]  # a newline first, so NSH prints a prompt
    in_flight = False
    next_poll = 0.0

    def send(data, flags=0):
        nonlocal in_flight
        in_flight = True

        def on_response(event):
            nonlocal in_flight, next_poll
            in_flight = False
            next_poll = time.monotonic() + POLL_INTERVAL

            if event is None:
                return

            if event.response.output:
                sys.stdout.write(bytes(event.response.output).decode('utf-8', errors='replace'))
                sys.stdout.flush()

            if event.response.flags & RSP_PENDING_OUTPUT:
                next_poll = time.monotonic()   # more waiting, ask again now

        node.request(dronecan.uavcan.protocol.AccessCommandShell.Request(
            flags=REQ_READ_OUTPUT | flags, input=bytearray(data)),
            target_node_id, on_response, timeout=TIMEOUT)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    attr = termios.tcgetattr(fd)
    # ISIG off so Ctrl-C reaches us as a byte instead of a local signal
    attr[3] &= ~termios.ECHO & ~termios.ICANON & ~termios.ISIG
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    line = b''

    sys.stdout.write(f"node {target_node_id}: Ctrl-C interrupts, Ctrl-D goes back\r\n")
    sys.stdout.flush()

    try:
        while True:
            if select.select([fd], [], [], 0)[0]:
                for b in os.read(fd, 4096):
                    if b == 0x04:  # Ctrl-D
                        return

                    if b == 0x03:  # Ctrl-C
                        # a stdin 0x03 raises no signal on the far end, so reset the shell
                        sys.stdout.write('^C\r\n')
                        line = b''
                        pending.append((b'\n', REQ_RESET_SHELL))
                    elif b in (0x0d, 0x0a):  # Enter
                        sys.stdout.write('\r\n')
                        line += b'\n'
                        pending.extend((line[i:i + MAX_INPUT], 0) for i in range(0, len(line), MAX_INPUT))
                        line = b''
                    elif b in (0x7f, 0x08) and line:  # backspace
                        line = line[:-1]
                        sys.stdout.write('\b \b')
                    elif 0x20 <= b < 0x7f:
                        line += bytes([b])
                        sys.stdout.write(chr(b))

                sys.stdout.flush()

            if not in_flight:
                if pending:
                    send(*pending.pop(0))
                elif time.monotonic() >= next_poll:
                    send(b'')

            spin(node)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        sys.stdout.write('\r\n')
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("device", help="e.g /dev/ttyACM0")
    parser.add_argument("--node-id", type=int, default=100,
                        help="node ID to use for this shell (default: 100)")
    parser.add_argument("--allocator", action="store_true",
                        help="this node will allocate node IDs for other nodes")
    parser.add_argument("--baudrate", type=int, default=115200, help="serial baudrate (default: 115200)")
    parser.add_argument("--bitrate", type=int, default=1000000, help="CAN bitrate (default: 1000000)")
    args = parser.parse_args()

    # Set the logging level to CRITICAL to suppress debug/info messages from the dronecan library
    logging.getLogger('dronecan').setLevel(logging.CRITICAL)
    node = dronecan.make_node(args.device, node_id=args.node_id,
                              bitrate=args.bitrate, baudrate=args.baudrate)
    node.health = 0  # HEALTH_OK
    node.mode = 0    # MODE_OPERATIONAL

    if args.allocator:
        monitor = dronecan.app.node_monitor.NodeMonitor(node)
        dronecan.app.dynamic_node_id.CentralizedServer(node, monitor)

    try:
        while True:
            nodes = scan_nodes(node)

            if not nodes:
                print("no nodes on the bus" + ("" if args.allocator else "; try --allocator"))
                return 1

            terminal(node, choose_node(nodes))
    except (EOFError, KeyboardInterrupt):
        print()
    except dronecan.driver.DriverError as ex:
        print(f"\nlost the adapter: {ex}")
        return 1
    finally:
        node.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
