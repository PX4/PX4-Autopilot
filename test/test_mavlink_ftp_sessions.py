#!/usr/bin/env python3
"""
MAVLink FTP concurrent session regression test.

Drives the MAVLink FTP server of a running PX4 SITL instance with raw
FILE_TRANSFER_PROTOCOL messages from several MAVLink identities on one link,
and checks that sessions are allocated, isolated, retried and cleaned up
independently.

Usage:
    python3 test_mavlink_ftp_sessions.py [--url udpout:127.0.0.1:18570] [--pid PID] [unittest args]

Set FTP_SESSIONS_SLOW=1 to include the tests that wait for the 30 s idle close.

Requirements:
    pip install pymavlink
"""

import argparse
import os
import struct
import sys
import time
import unittest
from typing import Dict, List, NamedTuple, Optional, Tuple

os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil  # type: ignore[import-not-found]
except ImportError:
    print("ERROR: pymavlink not installed.  Run: pip install pymavlink")
    sys.exit(1)

# Opcodes and error codes, see MavlinkFTP in src/modules/mavlink/mavlink_ftp.h
CMD_TERMINATE_SESSION = 1
CMD_RESET_SESSIONS = 2
CMD_OPEN_FILE_RO = 4
CMD_READ_FILE = 5
CMD_CREATE_FILE = 6
CMD_WRITE_FILE = 7
CMD_CREATE_DIRECTORY = 9
CMD_OPEN_FILE_WO = 11
CMD_BURST_READ_FILE = 15
RSP_ACK = 128
RSP_NAK = 129

ERR_FAIL_ERRNO = 2
ERR_INVALID_SESSION = 4
ERR_NO_SESSIONS_AVAILABLE = 5
ERR_EOF = 6
ERR_FILE_EXISTS = 8

HEADER = struct.Struct("<HBBBBBBI")
PAYLOAD_LEN = 251
MAX_DATA = PAYLOAD_LEN - HEADER.size
# MavlinkFTP::kRadioMaxPacketLength less the header, target fields and unsigned MAVLink 2 framing
RADIO_MAX_PAYLOAD = 118 - HEADER.size - 3 - 12

MAX_SESSIONS = 3  # MavlinkFTP::kMaxSessions

TEST_DIR = "/fs/ftp_sessions_test"

URL = "udpout:127.0.0.1:18570"
PX4_PID: Optional[int] = None
IDLE_TIMEOUT = 30  # MavlinkFTP::send()


class Reply(NamedTuple):
    seq: int
    session: int
    opcode: int
    size: int
    req_opcode: int
    burst_complete: int
    offset: int
    data: bytes

    @property
    def ack(self) -> bool:
        return self.opcode == RSP_ACK

    @property
    def error(self) -> Optional[int]:
        return self.data[0] if self.opcode == RSP_NAK else None

    def u32(self) -> int:
        return int(struct.unpack_from("<I", self.data)[0])


class Link:
    """One MAVLink connection shared by every client identity, like a GCS and a companion behind one router."""

    def __init__(self, url: str) -> None:
        self.conn = mavutil.mavlink_connection(url, source_system=250)
        # udpout only learns where PX4 is once something was sent
        self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        hb = self.conn.wait_heartbeat(timeout=10)

        if hb is None:
            raise RuntimeError("no heartbeat from PX4 on " + url)

        self.target_system = hb.get_srcSystem()
        self.target_component = hb.get_srcComponent()
        self.inbox: Dict[Tuple[int, int], List[Reply]] = {}

    def send(self, sysid: int, compid: int, seq: int, opcode: int, session: int, offset: int, data: bytes,
             size: Optional[int] = None) -> None:
        size = len(data) if size is None else size
        payload = HEADER.pack(seq, session, opcode, size, 0, 0, 0, offset) + data
        payload += bytes(PAYLOAD_LEN - len(payload))
        self.conn.mav.srcSystem = sysid
        self.conn.mav.srcComponent = compid
        self.conn.mav.file_transfer_protocol_send(0, self.target_system, self.target_component, payload)

    def poll(self, timeout: float) -> None:
        msg = self.conn.recv_match(type="FILE_TRANSFER_PROTOCOL", blocking=True, timeout=timeout)

        while msg is not None:
            raw = bytes(msg.payload)
            fields = HEADER.unpack_from(raw)
            reply = Reply(fields[0], fields[1], fields[2], fields[3], fields[4], fields[5], fields[7],
                          raw[HEADER.size:HEADER.size + fields[3]])
            self.inbox.setdefault((msg.target_system, msg.target_component), []).append(reply)
            msg = self.conn.recv_match(type="FILE_TRANSFER_PROTOCOL", blocking=False)

    def radio_status(self) -> None:
        """Pretend a telemetry radio with an empty buffer sits on this link, which turns on burst pacing."""
        self.conn.mav.srcSystem = 51
        self.conn.mav.srcComponent = 68
        self.conn.mav.radio_status_send(200, 200, 100, 0, 0, 0, 0)

    def take(self, sysid: int, compid: int, match, timeout: float) -> Optional[Reply]:
        deadline = time.monotonic() + timeout

        while True:
            box = self.inbox.setdefault((sysid, compid), [])

            for i, reply in enumerate(box):
                if match(reply):
                    return box.pop(i)

            remaining = deadline - time.monotonic()

            if remaining <= 0:
                return None

            self.poll(min(remaining, 0.1))

    def drain(self, quiet: float = 0.2) -> None:
        self.poll(quiet)
        self.inbox.clear()


class Client:
    """One MAVLink FTP client identity with its own sequence counter."""

    def __init__(self, link: Link, sysid: int, compid: int = 190) -> None:
        self.link = link
        self.sysid = sysid
        self.compid = compid
        self.seq = 0

    def send(self, opcode: int, session: int = 0, offset: int = 0, data: bytes = b"", size: Optional[int] = None,
             seq: Optional[int] = None) -> int:
        """Send a request without waiting, return its sequence number for a later retry."""
        if seq is None:
            seq = self.seq
            self.seq = (self.seq + 1) & 0xFFFF

        self.link.send(self.sysid, self.compid, seq, opcode, session, offset, data, size)
        return seq

    def reply(self, seq: int, timeout: float = 2.0) -> Reply:
        expected = (seq + 1) & 0xFFFF
        # burst data is collected by burst(), only a NAK answers a burst request directly
        reply = self.link.take(self.sysid, self.compid,
                               lambda r: r.seq == expected and (r.req_opcode != CMD_BURST_READ_FILE or not r.ack),
                               timeout)

        if reply is None:
            raise AssertionError("no reply to seq {} for {}".format(seq, self.sysid))

        return reply

    def request(self, opcode: int, session: int = 0, offset: int = 0, data: bytes = b"",
                size: Optional[int] = None) -> Reply:
        return self.reply(self.send(opcode, session, offset, data, size))

    def open(self, path: str, opcode: int = CMD_OPEN_FILE_RO) -> Reply:
        return self.request(opcode, data=path.encode())

    def read(self, session: int, offset: int = 0, size: int = MAX_DATA) -> Reply:
        return self.request(CMD_READ_FILE, session, offset, size=size)

    def write(self, session: int, offset: int, data: bytes) -> Reply:
        return self.request(CMD_WRITE_FILE, session, offset, data)

    def terminate(self, session: int) -> Reply:
        return self.request(CMD_TERMINATE_SESSION, session)

    def read_all(self, session: int) -> bytes:
        content = b""

        while True:
            reply = self.read(session, len(content))

            if reply.error == ERR_EOF:
                return content

            assert reply.ack, "read failed: {}".format(reply)
            content += reply.data

    def put(self, path: str, content: bytes) -> None:
        opened = self.open(path, CMD_CREATE_FILE)
        assert opened.ack, "create {} failed: {}".format(path, opened)

        for offset in range(0, len(content), MAX_DATA):
            assert self.write(opened.session, offset, content[offset:offset + MAX_DATA]).ack

        assert self.terminate(opened.session).ack

    def get(self, path: str) -> bytes:
        opened = self.open(path)
        assert opened.ack, "open {} failed: {}".format(path, opened)
        content = self.read_all(opened.session)
        assert self.terminate(opened.session).ack
        return content


class Burst:
    """A burst download of one session, re-requested at every chunk end until the file ended."""

    def __init__(self, client: Client, session: int) -> None:
        self.client = client
        self.session = session
        self.content = bytearray()
        self.sizes: List[int] = []
        self.done = False
        self.client.send(CMD_BURST_READ_FILE, session)

    def take(self) -> bool:
        """Consume one packet if one arrived, return whether it did."""
        reply = self.client.link.take(self.client.sysid, self.client.compid,
                                      lambda r: r.req_opcode == CMD_BURST_READ_FILE, 0)

        if reply is None:
            return False

        assert reply.session == self.session, reply

        if not reply.ack:
            assert reply.error == ERR_EOF and reply.burst_complete, reply
            self.done = True
            return True

        assert reply.offset == len(self.content), "lost burst packet: {}".format(reply)
        self.content += reply.data
        self.sizes.append(reply.size)

        if reply.burst_complete:
            self.client.send(CMD_BURST_READ_FILE, self.session, len(self.content))

        return True


def run_bursts(link: Link, bursts: List[Burst], radio: bool = False, timeout: float = 30) -> List[int]:
    """Drive bursts to their end, return which of them each data or EOF packet belonged to, in arrival order."""
    order: List[int] = []
    deadline = time.monotonic() + timeout
    next_radio = 0.0

    while not all(burst.done for burst in bursts):
        assert time.monotonic() < deadline, "bursts did not finish"

        if radio and time.monotonic() > next_radio:
            link.radio_status()
            next_radio = time.monotonic() + 0.5

        link.poll(0.05)

        for i, burst in enumerate(bursts):
            while not burst.done and burst.take():
                order.append(i)

    return order


class FtpSessionTest(unittest.TestCase):
    link: Link

    @classmethod
    def setUpClass(cls) -> None:
        cls.link = Link(URL)
        setup = Client(cls.link, 251)
        made = setup.request(CMD_CREATE_DIRECTORY, data=TEST_DIR.encode())
        assert made.ack or made.error in (ERR_FILE_EXISTS, ERR_FAIL_ERRNO), made

    def setUp(self) -> None:
        self.link.drain()
        self.a = Client(self.link, 245)
        self.b = Client(self.link, 246)
        self.assertTrue(Client(self.link, 251).request(CMD_RESET_SESSIONS).ack)

    def tearDown(self) -> None:
        Client(self.link, 251).request(CMD_RESET_SESSIONS)

    def path(self, name: str) -> str:
        return "{}/{}_{}".format(TEST_DIR, self.id().rsplit(".", 1)[-1], name)

    def test_second_open_keeps_first_session(self) -> None:
        alpha, bravo = self.path("alpha"), self.path("bravo")
        self.a.put(alpha, b"alpha" * 60)
        self.a.put(bravo, b"bravo" * 60)

        first = self.a.open(alpha)
        second = self.b.open(bravo)
        self.assertTrue(first.ack and second.ack)
        self.assertNotEqual(first.session, second.session)

        self.assertEqual(self.a.read_all(first.session), b"alpha" * 60)
        self.assertEqual(self.b.read_all(second.session), b"bravo" * 60)
        self.assertTrue(self.a.terminate(first.session).ack)
        self.assertTrue(self.b.terminate(second.session).ack)

    def test_capacity_is_bounded(self) -> None:
        name = self.path("shared")
        self.a.put(name, b"x")
        clients = [Client(self.link, 230 + i) for i in range(MAX_SESSIONS + 1)]
        opened = [client.open(name) for client in clients[:MAX_SESSIONS]]
        self.assertTrue(all(reply.ack for reply in opened), opened)
        self.assertEqual(len({reply.session for reply in opened}), MAX_SESSIONS)

        self.assertEqual(clients[-1].open(name).error, ERR_NO_SESSIONS_AVAILABLE)
        # a refused open takes nothing, the others are all still usable
        self.assertTrue(all(c.read(r.session).data == b"x" for c, r in zip(clients, opened)))

        self.assertTrue(clients[0].terminate(opened[0].session).ack)
        self.assertTrue(clients[-1].open(name).ack)

    def test_session_belongs_to_its_client(self) -> None:
        name = self.path("private")
        self.a.put(name, b"mine")
        opened = self.a.open(name)
        self.assertTrue(opened.ack)

        # same component id from another system, and another component of the same system
        for other in (self.b, Client(self.link, self.a.sysid, self.a.compid + 1)):
            self.assertEqual(other.read(opened.session).error, ERR_INVALID_SESSION)
            self.assertEqual(other.request(CMD_BURST_READ_FILE, opened.session).error, ERR_INVALID_SESSION)
            self.assertEqual(other.write(opened.session, 0, b"theirs").error, ERR_INVALID_SESSION)
            self.assertEqual(other.terminate(opened.session).error, ERR_INVALID_SESSION)

        self.assertEqual(self.a.read_all(opened.session), b"mine")
        self.assertTrue(self.a.terminate(opened.session).ack)

    def test_one_client_interleaves_sessions(self) -> None:
        source, copy = self.path("source"), self.path("copy")
        content = bytes(range(256)) * 4
        self.a.put(source, content)

        reading = self.a.open(source)
        writing = self.a.open(copy, CMD_CREATE_FILE)
        self.assertTrue(reading.ack and writing.ack)
        self.assertNotEqual(reading.session, writing.session)

        offset = 0

        while True:
            chunk = self.a.read(reading.session, offset)

            if chunk.error == ERR_EOF:
                break

            self.assertTrue(chunk.ack, chunk)
            self.assertTrue(self.a.write(writing.session, offset, chunk.data).ack)
            offset += len(chunk.data)

        self.assertTrue(self.a.terminate(writing.session).ack)
        self.assertTrue(self.a.terminate(reading.session).ack)
        self.assertEqual(self.a.get(copy), content)

    def open_descriptors(self, name: str) -> int:
        """Descriptors PX4 holds on test files whose name starts with this one, -1 without --pid."""
        if PX4_PID is None:
            return -1

        fd_dir = "/proc/{}/fd".format(PX4_PID)
        stem = "/" + self.path(name).rsplit("/", 1)[-1]
        count = 0

        for fd in os.listdir(fd_dir):
            try:
                count += stem in os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                pass  # closed since listdir

        return count

    def test_reset_closes_every_session(self) -> None:
        names = [self.path("reset{}".format(i)) for i in range(MAX_SESSIONS)]

        for name in names:
            self.a.put(name, b"reset")

        held = [(self.a, self.a.open(names[0])), (self.b, self.b.open(names[1])),
                (self.b, self.b.open(names[2], CMD_OPEN_FILE_WO))]
        self.assertTrue(all(reply.ack for _, reply in held))
        self.assertIn(self.open_descriptors("reset"), (-1, MAX_SESSIONS))

        # any client resets every session, not only its own
        self.assertTrue(Client(self.link, 247).request(CMD_RESET_SESSIONS).ack)

        for client, reply in held:
            self.assertEqual(client.read(reply.session).error, ERR_INVALID_SESSION)

        self.assertIn(self.open_descriptors("reset"), (-1, 0))
        reopened = [Client(self.link, 230 + i).open(names[i]) for i in range(MAX_SESSIONS)]
        self.assertTrue(all(reply.ack for reply in reopened), reopened)

    @unittest.skipUnless(os.environ.get("FTP_SESSIONS_SLOW"), "set FTP_SESSIONS_SLOW=1 to wait for the idle close")
    def test_idle_sessions_expire_independently(self) -> None:
        name = self.path("idle")
        self.a.put(name, b"idle")
        idle, busy = self.a.open(name), self.b.open(name)
        self.assertTrue(idle.ack and busy.ack)

        # the idle close is 30 s, client A says nothing meanwhile
        deadline = time.monotonic() + IDLE_TIMEOUT + 3

        while time.monotonic() < deadline:
            self.assertEqual(self.b.read(busy.session).data, b"idle")
            self.link.poll(3)

        self.assertEqual(self.a.read(idle.session).error, ERR_INVALID_SESSION)
        self.assertEqual(self.b.read(busy.session).data, b"idle")
        self.assertTrue(self.b.terminate(busy.session).ack)

    def open_bursts(self, clients: List[Client], size: int) -> List[Tuple[bytes, int]]:
        files = []

        for i, client in enumerate(clients):
            content = bytes((i * 37 + n) & 0xFF for n in range(size))
            name = self.path("burst{}".format(i))
            client.put(name, content)
            opened = client.open(name)
            self.assertTrue(opened.ack)
            files.append((content, opened.session))

        return files

    def assert_interleaved(self, order: List[int]) -> None:
        # each burst gets packets out before the other one has finished
        self.assertLess(order.index(1), len(order) - 1 - order[::-1].index(0), order)
        self.assertLess(order.index(0), len(order) - 1 - order[::-1].index(1), order)

    def test_bursts_interleave(self) -> None:
        # larger than one send() call puts out on a UDP link, so a burst cannot finish before the next starts
        files = self.open_bursts([self.a, self.b], 40000)
        bursts = [Burst(self.a, files[0][1]), Burst(self.b, files[1][1])]
        order = run_bursts(self.link, bursts)

        for burst, (content, _) in zip(bursts, files):
            self.assertEqual(bytes(burst.content), content)

        self.assert_interleaved(order)

    def test_paced_bursts_share_one_budget(self) -> None:
        size = 6000
        files = self.open_bursts([self.a, self.b], size)
        self.link.radio_status()
        self.link.drain(1.0)

        start = time.monotonic()
        single = Burst(self.a, files[0][1])
        run_bursts(self.link, [single], radio=True)
        single_time = time.monotonic() - start
        self.assertEqual(bytes(single.content), files[0][0])

        start = time.monotonic()
        bursts = [Burst(self.a, files[0][1]), Burst(self.b, files[1][1])]
        order = run_bursts(self.link, bursts, radio=True)
        both_time = time.monotonic() - start

        for burst, (content, _) in zip(bursts, files):
            self.assertEqual(bytes(burst.content), content)
            # behind a radio every packet stays within one radio packet of 118 bytes
            self.assertLessEqual(max(burst.sizes), RADIO_MAX_PAYLOAD)

        self.assert_interleaved(order)
        # twice the data through the same budget takes about twice as long, not as long as one burst
        self.assertGreater(both_time, 1.6 * single_time, (single_time, both_time))


def main() -> None:
    global URL, PX4_PID
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default=URL, help="pymavlink connection to the PX4 GCS link")
    parser.add_argument("--pid", type=int, help="PX4 process on this host, to check it closes the files")
    args, rest = parser.parse_known_args()
    URL = args.url
    PX4_PID = args.pid
    unittest.main(argv=[sys.argv[0]] + rest, verbosity=2)


if __name__ == "__main__":
    main()
