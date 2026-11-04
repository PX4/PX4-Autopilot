#!/usr/bin/env python3
"""ubx_monitor.py — watch a u-blox receiver's serial output live.

Autobauds by scoring candidate rates on valid UBX frames and NMEA sentences, then draws a
full-screen view of the link: connection state, link utilisation, receiver identity from
MON-VER, fix state from NAV-PVT, and a table of every message seen with its rate. Losing the
stream drops back to autobauding, so it survives a receiver reboot or a reconfiguration.

Point it at whatever the receiver is wired to: the flight controller's GPS port through an
adapter, or the receiver's UART2 in the GPS_UBX_MODE diagnostic modes.

    Tools/ubx_monitor.py                                 # /dev/ttyUSB1, autobaud
    Tools/ubx_monitor.py -p /dev/ttyUSB0 -b 230400       # known port and rate
    Tools/ubx_monitor.py --bauds 230400,115200 --no-tx   # narrow the search, never transmit

Quit with q, reset the counters with r. Off a TTY, with --plain, or without rich installed
(pip install rich) it prints one line per interval instead of drawing, so it can be piped to a
log. pyserial is required either way.
"""
from __future__ import annotations

import argparse
import contextlib
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import serial

try:
    from rich import box
    from rich.console import Console
    from rich.layout import Layout
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    HAVE_RICH = True
except ImportError:
    HAVE_RICH = False

    class Console:  # stand-in for the plain mode: print with the markup stripped
        def print(self, renderable="") -> None:
            print(re.sub(r"\[/?[a-z0-9 #_]*\]", "", str(renderable)), flush=True)

BAUDS = [115200, 38400, 9600, 230400, 460800, 921600, 57600, 19200, 4800]
REFRESH_PER_SECOND = 8
RATE_WINDOW_SECONDS = 3.0        # sliding window for per-message rates
BITRATE_WINDOW_SECONDS = 1.0     # sliding window for the instantaneous bitrate
HISTORY_SECONDS = 30.0           # width of the bitrate sparkline
STALE_SECONDS = 2.0              # no bytes for this long and the link is drawn as down
VERSION_POLL_SECONDS = 3.0       # MON-VER poll interval until the receiver answers
SPARK_BLOCKS = " ▁▂▃▄▅▆▇█"

NMEA_RE = re.compile(rb"[$!]([^$!\r\n*]{5,120})\*([0-9A-Fa-f]{2})\r?\n")

UBX_CLASSES = {
    0x01: "NAV", 0x02: "RXM", 0x04: "INF", 0x05: "ACK", 0x06: "CFG", 0x09: "UPD",
    0x0A: "MON", 0x0B: "AID", 0x0D: "TIM", 0x10: "ESF", 0x13: "MGA", 0x21: "LOG",
    0x27: "SEC", 0x28: "HNR", 0x29: "NAV2",
}

UBX_MESSAGES = {
    (0x01, 0x01): "NAV-POSECEF", (0x01, 0x02): "NAV-POSLLH", (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x04): "NAV-DOP", (0x01, 0x06): "NAV-SOL", (0x01, 0x07): "NAV-PVT",
    (0x01, 0x09): "NAV-ODO", (0x01, 0x11): "NAV-VELECEF", (0x01, 0x12): "NAV-VELNED",
    (0x01, 0x13): "NAV-HPPOSECEF", (0x01, 0x14): "NAV-HPPOSLLH", (0x01, 0x20): "NAV-TIMEGPS",
    (0x01, 0x21): "NAV-TIMEUTC", (0x01, 0x22): "NAV-CLOCK", (0x01, 0x26): "NAV-TIMELS",
    (0x01, 0x30): "NAV-SVINFO", (0x01, 0x32): "NAV-SBAS", (0x01, 0x34): "NAV-ORB",
    (0x01, 0x35): "NAV-SAT", (0x01, 0x36): "NAV-COV", (0x01, 0x3B): "NAV-SVIN",
    (0x01, 0x3C): "NAV-RELPOSNED", (0x01, 0x43): "NAV-SIG", (0x01, 0x61): "NAV-EOE",
    (0x02, 0x13): "RXM-SFRBX", (0x02, 0x14): "RXM-MEASX", (0x02, 0x15): "RXM-RAWX",
    (0x02, 0x32): "RXM-RTCM", (0x02, 0x34): "RXM-COR", (0x02, 0x59): "RXM-RLM",
    (0x02, 0x72): "RXM-PMP", (0x02, 0x73): "RXM-QZSSL6", (0x02, 0x84): "RXM-SPARTN",
    (0x04, 0x00): "INF-ERROR", (0x04, 0x01): "INF-WARNING", (0x04, 0x02): "INF-NOTICE",
    (0x04, 0x03): "INF-TEST", (0x04, 0x04): "INF-DEBUG",
    (0x05, 0x00): "ACK-NAK", (0x05, 0x01): "ACK-ACK",
    (0x06, 0x8A): "CFG-VALSET", (0x06, 0x8B): "CFG-VALGET", (0x06, 0x8C): "CFG-VALDEL",
    (0x0A, 0x02): "MON-IO", (0x0A, 0x04): "MON-VER", (0x0A, 0x06): "MON-MSGPP",
    (0x0A, 0x07): "MON-RXBUF", (0x0A, 0x08): "MON-TXBUF", (0x0A, 0x09): "MON-HW",
    (0x0A, 0x0B): "MON-HW2", (0x0A, 0x21): "MON-RXR", (0x0A, 0x28): "MON-GNSS",
    (0x0A, 0x31): "MON-SPAN", (0x0A, 0x36): "MON-COMMS", (0x0A, 0x37): "MON-HW3",
    (0x0A, 0x38): "MON-RF", (0x0A, 0x39): "MON-SYS",
    (0x0D, 0x01): "TIM-TP", (0x0D, 0x03): "TIM-TM2", (0x0D, 0x15): "TIM-SVIN",
    (0x10, 0x02): "ESF-MEAS", (0x10, 0x03): "ESF-RAW", (0x10, 0x10): "ESF-STATUS",
    (0x10, 0x14): "ESF-ALG", (0x10, 0x15): "ESF-INS",
    (0x13, 0x20): "MGA-ANO", (0x13, 0x40): "MGA-INI", (0x13, 0x60): "MGA-ACK",
    (0x27, 0x03): "SEC-UNIQID", (0x27, 0x09): "SEC-SIG", (0x27, 0x10): "SEC-SIGLOG",
}

FIX_TYPES = {0: "no fix", 1: "dead reckoning", 2: "2D fix", 3: "3D fix", 4: "GNSS+DR", 5: "time only"}
CARRIER_SOLUTIONS = {1: "RTK float", 2: "RTK fixed"}


def message_name(msg_class: int, msg_id: int) -> str:
    known = UBX_MESSAGES.get((msg_class, msg_id))
    if known:
        return known
    group = UBX_CLASSES.get(msg_class)
    return f"{group}-0x{msg_id:02X}" if group else f"UBX 0x{msg_class:02X}-0x{msg_id:02X}"


def ubx_checksum(payload: bytes) -> tuple[int, int]:
    ck_a = ck_b = 0
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def ubx_frame(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_class, msg_id, len(payload) & 0xFF, len(payload) >> 8]) + payload
    return b"\xb5\x62" + body + bytes(ubx_checksum(body))


MON_VER_POLL = ubx_frame(0x0A, 0x04)


# --- statistics ------------------------------------------------------------------------------------

@dataclass
class MessageRecord:
    count: int = 0
    total_bytes: int = 0
    last_seen: float = 0.0
    arrivals: deque = field(default_factory=lambda: deque(maxlen=1024))


@dataclass
class Receiver:
    """Whatever the receiver has said about itself in MON-VER."""

    software: str = ""
    hardware: str = ""
    firmware: str = ""
    protocol: str = ""
    module: str = ""

    @property
    def known(self) -> bool:
        return bool(self.software or self.module)


@dataclass
class FixState:
    fix_type: int = -1
    carrier: int = 0
    satellites: int = 0
    horizontal_accuracy: float = 0.0  # metres

    def describe(self) -> str:
        if self.fix_type < 0:
            return "—"
        return CARRIER_SOLUTIONS.get(self.carrier) or FIX_TYPES.get(self.fix_type, f"type {self.fix_type}")


class Stats:
    """Everything the reader thread records and the renderer reads, under one lock."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.reset(time.monotonic())

    def reset(self, now: float) -> None:
        self.messages: dict[str, MessageRecord] = {}
        # Seeded so the very first report measures against the reset, not against itself.
        self.byte_samples: deque = deque([(now, 0)])  # (timestamp, cumulative bytes)
        self.bitrate_history: deque = deque()   # (timestamp, bytes per second)
        self.total_bytes = 0
        self.ubx_bytes = 0
        self.nmea_bytes = 0
        self.junk_bytes = 0
        self.bad_checksums = 0
        self.peak_rate = 0.0
        self.start_time = now
        self.last_data = 0.0
        self.receiver = Receiver()
        self.fix = FixState()

    def record(self, name: str, size: int, now: float) -> None:
        record = self.messages.get(name)
        if record is None:
            record = self.messages[name] = MessageRecord()
        record.count += 1
        record.total_bytes += size
        record.last_seen = now
        record.arrivals.append(now)

    def sample(self, now: float) -> None:
        self.byte_samples.append((now, self.total_bytes))
        while len(self.byte_samples) > 2 and self.byte_samples[0][0] < now - BITRATE_WINDOW_SECONDS:
            self.byte_samples.popleft()
        first_time, first_bytes = self.byte_samples[0]
        span = now - first_time
        rate = (self.total_bytes - first_bytes) / span if span > 0 else 0.0
        self.bitrate_history.append((now, rate))
        while self.bitrate_history and self.bitrate_history[0][0] < now - HISTORY_SECONDS:
            self.bitrate_history.popleft()
        self.peak_rate = max(self.peak_rate, rate)


# --- parsing ---------------------------------------------------------------------------------------

class Parser:
    """Incremental UBX/NMEA framer. Every byte lands in exactly one bucket — a framed message, a
    checksum failure, or junk — so the shares in the header describe the whole link."""

    def __init__(self, stats: Stats):
        self.stats = stats
        self.buffer = bytearray()

    def feed(self, data: bytes, now: float) -> None:
        self.buffer += data
        with self.stats.lock:
            self.stats.total_bytes += len(data)
            self.stats.last_data = now
            self._drain(now)

    def _drain(self, now: float) -> None:
        buffer = self.buffer
        while buffer:
            start = buffer[0]
            if start == 0xB5:
                if len(buffer) < 2:
                    return
                if buffer[1] != 0x62:
                    self._discard(1)
                    continue
                if len(buffer) < 8:
                    return
                length = buffer[4] | (buffer[5] << 8)
                if length > 8192:
                    self._discard(2)
                    continue
                end = 6 + length + 2
                if len(buffer) < end:
                    return
                ck_a, ck_b = ubx_checksum(bytes(buffer[2:6 + length]))
                if (ck_a, ck_b) != (buffer[end - 2], buffer[end - 1]):
                    self.stats.bad_checksums += 1
                    self._discard(2)
                    continue
                self._ubx(bytes(buffer[:end]), now)
                del buffer[:end]
            elif start in (0x24, 0x21):  # '$' or '!'
                match = NMEA_RE.match(bytes(buffer))
                if match:
                    self._nmea(match, now)
                    continue
                # Wait for the rest of the sentence, unless it is already too long to be one.
                if b"\n" not in buffer[:128] and len(buffer) <= 128:
                    return
                self._discard(1)
            else:
                nearest = len(buffer)
                for marker in (b"\xb5\x62", b"$", b"!"):
                    found = buffer.find(marker, 1)
                    if found >= 0:
                        nearest = min(nearest, found)
                self._discard(nearest)

    def _discard(self, count: int) -> None:
        self.stats.junk_bytes += count
        del self.buffer[:count]

    def _ubx(self, frame: bytes, now: float) -> None:
        msg_class, msg_id = frame[2], frame[3]
        self.stats.ubx_bytes += len(frame)
        self.stats.record(message_name(msg_class, msg_id), len(frame), now)
        payload = frame[6:-2]
        if (msg_class, msg_id) == (0x0A, 0x04):
            self._mon_ver(payload)
        elif (msg_class, msg_id) == (0x01, 0x07) and len(payload) >= 44:
            self._nav_pvt(payload)

    def _nmea(self, match: "re.Match[bytes]", now: float) -> None:
        body, checksum = match.group(1), match.group(2)
        parity = 0
        for byte in body:
            parity ^= byte
        if parity != int(checksum, 16):
            self.stats.bad_checksums += 1
            self._discard(1)
            return
        self.stats.nmea_bytes += match.end()
        sentence = body.split(b",")[0].decode("ascii", "replace")
        self.stats.record(f"NMEA {sentence}", match.end(), now)
        del self.buffer[:match.end()]

    def _mon_ver(self, payload: bytes) -> None:
        receiver = self.stats.receiver
        receiver.software = payload[0:30].split(b"\0")[0].decode("ascii", "replace")
        receiver.hardware = payload[30:40].split(b"\0")[0].decode("ascii", "replace")
        for offset in range(40, len(payload) - 29, 30):
            extension = payload[offset:offset + 30].split(b"\0")[0].decode("ascii", "replace")
            for prefix, attribute in (("FWVER=", "firmware"), ("PROTVER=", "protocol"), ("MOD=", "module")):
                if extension.startswith(prefix):
                    setattr(receiver, attribute, extension[len(prefix):])

    def _nav_pvt(self, payload: bytes) -> None:
        fix = self.stats.fix
        fix.fix_type = payload[20]
        fix.carrier = (payload[21] >> 6) & 0x03
        fix.satellites = payload[23]
        fix.horizontal_accuracy = int.from_bytes(payload[40:44], "little") / 1000.0


# --- autobaud --------------------------------------------------------------------------------------

def score(sample: bytes) -> tuple[int, int, int]:
    """(score, UBX frames, NMEA sentences) for a raw capture — how u-blox does this look?"""
    probe = Stats()
    Parser(probe).feed(sample, time.monotonic())
    frames = sum(record.count for name, record in probe.messages.items() if not name.startswith("NMEA"))
    sentences = sum(record.count for name, record in probe.messages.items() if name.startswith("NMEA"))
    return frames * 2 + sentences, frames, sentences


def autobaud(console: Console, port: str, window: float, bauds: list[int], sweep: int) -> int | None:
    """One sweep over the candidate rates. Returns the winner, or None if nothing framed."""
    best: tuple[int, int] | None = None
    seen: list[str] = []
    for baud in bauds:
        try:
            with serial.Serial(port, baud, timeout=0.1) as link:
                link.reset_input_buffer()
                sample = bytearray()
                deadline = time.monotonic() + window
                while time.monotonic() < deadline:
                    sample += link.read(4096)
        except serial.SerialException as error:
            console.print(f"  [red]{baud:>7}[/]  {error}")
            time.sleep(0.5)
            continue
        points, frames, sentences = score(bytes(sample))
        if sample:
            seen.append(f"  [cyan]{baud:>7}[/]  {len(sample):>6} B   ubx {frames:>3}   "
                        f"nmea {sentences:>3}   score {points}")
        if points and (best is None or points > best[1]):
            best = (baud, points)
    for line in seen:
        console.print(line)
    if best is None:
        console.print(f"[yellow]sweep {sweep}[/]: no u-blox traffic at any rate "
                      f"[dim]({time.strftime('%H:%M:%S')})[/]")
        return None
    return best[0]


# --- rendering -------------------------------------------------------------------------------------

def human_rate(bytes_per_second: float) -> str:
    bits = bytes_per_second * 10  # 8N1: 10 line bits per byte
    for unit in ("bit", "kbit"):
        if bits < 1000:
            return f"{bits:.1f} {unit}/s"
        bits /= 1000
    return f"{bits:.1f} Mbit/s"


def human_bytes(total: int) -> str:
    value = float(total)
    for unit in ("B", "KiB", "MiB"):
        if value < 1024:
            return f"{value:.0f} {unit}" if unit == "B" else f"{value:.1f} {unit}"
        value /= 1024
    return f"{value:.1f} GiB"


def clock(seconds: float) -> str:
    seconds = int(seconds)
    hours, remainder = divmod(seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:d}:{minutes:02d}:{secs:02d}" if hours else f"{minutes:02d}:{secs:02d}"


def utilisation_bar(fraction: float, width: int = 20) -> Text:
    fraction = max(0.0, min(fraction, 1.0))
    filled = int(round(fraction * width))
    style = "green" if fraction < 0.6 else "yellow" if fraction < 0.85 else "red"
    bar = Text()
    bar.append("█" * filled, style=style)
    bar.append("─" * (width - filled), style="grey37")
    return bar


def sparkline(values: list[float], width: int) -> Text:
    if not values or width <= 0:
        return Text("")
    step = max(1, len(values) // width)
    sampled = [max(values[index:index + step]) for index in range(0, len(values), step)][-width:]
    ceiling = max(sampled) or 1.0
    top = len(SPARK_BLOCKS) - 1
    return Text("".join(SPARK_BLOCKS[min(int(value / ceiling * top), top)] for value in sampled), style="cyan")


@dataclass
class Snapshot:
    """A consistent copy of the stats, taken under the lock, for one frame."""

    rate: float
    peak: float
    total_bytes: int
    shares: tuple[float, float, float]
    bad_checksums: int
    elapsed: float
    stale: float
    history: list[float]
    receiver: Receiver
    fix: FixState
    rows: list[tuple[str, int, float, int]]


def snapshot(stats: Stats, now: float) -> Snapshot:
    with stats.lock:
        stats.sample(now)
        window = max(min(now - stats.start_time, RATE_WINDOW_SECONDS), 1e-3)
        rows = []
        for name, record in stats.messages.items():
            while record.arrivals and record.arrivals[0] < now - RATE_WINDOW_SECONDS:
                record.arrivals.popleft()
            rows.append((name, record.count, len(record.arrivals) / window, record.total_bytes))
        rows.sort(key=lambda row: row[0])
        total = max(stats.total_bytes, 1)
        return Snapshot(
            rate=stats.bitrate_history[-1][1] if stats.bitrate_history else 0.0,
            peak=stats.peak_rate,
            total_bytes=stats.total_bytes,
            shares=(stats.ubx_bytes / total, stats.nmea_bytes / total, stats.junk_bytes / total),
            bad_checksums=stats.bad_checksums,
            elapsed=now - stats.start_time,
            # Before the first byte the idle timer runs from the session start, so a link that
            # never says anything still gets idle_limit seconds to prove itself.
            stale=now - (stats.last_data or stats.start_time),
            history=[rate for _, rate in stats.bitrate_history],
            receiver=stats.receiver,
            fix=stats.fix,
            rows=rows,
        )


def header_panel(port: str, baud: int, view: Snapshot) -> Panel:
    live = view.stale < STALE_SECONDS
    # Both lines are cropped rather than wrapped: a wrapped first line would push the second one
    # out of the fixed-height panel.
    line = Text(no_wrap=True, overflow="ellipsis")
    line.append(" LINK UP " if live else " NO DATA ", style="bold white on green" if live else "bold white on red")
    line.append(f"  {port}  ")
    line.append(f"{baud} baud", style="bold")
    line.append("   ")
    line.append(f"{human_rate(view.rate):>12}", style="bold cyan")
    line.append("  ")
    line.append_text(utilisation_bar(view.rate * 10 / baud))
    line.append(f" {view.rate * 1000 / baud:3.0f}% of link   ")
    line.append(f"peak {human_rate(view.peak)}   ", style="dim")
    line.append(f"{human_bytes(view.total_bytes)}   up {clock(view.elapsed)}", style="dim")

    identity = Text(no_wrap=True, overflow="ellipsis")
    receiver = view.receiver
    if receiver.known:
        identity.append(receiver.module or receiver.hardware or "u-blox", style="bold")
        for label, value in (("fw", receiver.firmware), ("proto", receiver.protocol), ("sw", receiver.software)):
            if value:
                identity.append(f"  {label} ")
                identity.append(value, style="cyan")
    else:
        identity.append("receiver identity unknown (no MON-VER yet)", style="dim")
    identity.append("   ")
    fix = view.fix
    if fix.fix_type >= 0:
        style = "green" if fix.fix_type >= 3 else "yellow"
        identity.append(fix.describe(), style=f"bold {style}")
        identity.append(f"  {fix.satellites} sats  hAcc {fix.horizontal_accuracy:.3f} m", style=style)
    ubx_share, nmea_share, junk_share = view.shares
    identity.append(f"   ubx {ubx_share * 100:.0f}%  nmea {nmea_share * 100:.0f}%  junk {junk_share * 100:.0f}%",
                    style="dim")
    if view.bad_checksums:
        identity.append(f"  crc errors {view.bad_checksums}", style="red")

    body = Text(no_wrap=True, overflow="ellipsis")
    body.append_text(line)
    body.append("\n")
    body.append_text(identity)
    return Panel(body, border_style="green" if live else "red", padding=(0, 1),
                 title="u-blox link", title_align="left")


class Sparkline:
    """Bitrate history, drawn to whatever width the region ends up with."""

    def __init__(self, view: Snapshot):
        self.view = view

    def __rich_console__(self, console, options):
        width = max(int(options.max_width), 20)
        line = Text()
        line.append_text(sparkline(self.view.history, width - 20))
        line.append(f"  now {human_rate(self.view.rate)}", style="dim")
        yield line


def message_table(view: Snapshot) -> Table:
    table = Table(expand=True, box=box.SIMPLE_HEAD, pad_edge=False)
    table.add_column("message", ratio=3, no_wrap=True)
    table.add_column("rate", justify="right", ratio=1)
    table.add_column("count", justify="right", ratio=1)
    table.add_column("bytes", justify="right", ratio=1)
    table.add_column("share", justify="right", ratio=1)
    total = max(view.total_bytes, 1)
    for name, count, rate, message_bytes in view.rows:
        table.add_row(Text(name, style="yellow" if name.startswith("NMEA") else "cyan"),
                      f"{rate:.1f} Hz", str(count), human_bytes(message_bytes),
                      f"{message_bytes * 100 / total:.0f}%")
    return table


def render(port: str, baud: int, view: Snapshot) -> Layout:
    layout = Layout()
    layout.split_column(
        Layout(header_panel(port, baud, view), name="header", size=4),
        Layout(Panel(Sparkline(view), border_style="cyan", padding=(0, 1),
                     title=f"bitrate, last {HISTORY_SECONDS:.0f}s", title_align="left"), name="rate", size=3),
        Layout(Panel(message_table(view), border_style="blue", padding=(0, 1),
                     title=f"messages ({len(view.rows)})  —  q quit, r reset", title_align="left"),
               name="messages", ratio=1),
    )
    return layout


# --- live loop -------------------------------------------------------------------------------------

@contextlib.contextmanager
def raw_keyboard():
    """cbreak so q/r are read without Enter; Ctrl-C still interrupts. No-op off a TTY."""
    if not sys.stdin.isatty():
        yield
        return
    import termios
    import tty
    descriptor = sys.stdin.fileno()
    saved = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        yield
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, saved)


def pending_key() -> str:
    if not sys.stdin.isatty():
        return ""
    import select
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if ready else ""


def reader(link: serial.Serial, parser: Parser, stop: threading.Event, poll: bool) -> None:
    """Own the port: bytes are framed here, and the MON-VER poll is written from here too, so only
    one thread ever touches the serial device."""
    next_poll = 0.0
    while not stop.is_set():
        try:
            chunk = link.read(4096)
        except serial.SerialException:
            stop.set()
            return
        now = time.monotonic()
        if chunk:
            parser.feed(chunk, now)
        if poll and now >= next_poll:
            with parser.stats.lock:
                known = parser.stats.receiver.known
            if not known:
                try:
                    link.write(MON_VER_POLL)
                except serial.SerialException:
                    poll = False
            next_poll = now + VERSION_POLL_SECONDS


def open_and_read(port: str, baud: int, stats: Stats, poll: bool):
    """Open the port and start the reader thread. Returns (link, parser, stop, thread)."""
    link = serial.Serial(port, baud, timeout=0.2)
    parser = Parser(stats)
    with stats.lock:
        stats.reset(time.monotonic())
    stop = threading.Event()
    thread = threading.Thread(target=reader, args=(link, parser, stop, poll), daemon=True)
    thread.start()
    return link, stop, thread


def watch(console: Console, port: str, baud: int, stats: Stats, idle_limit: float, poll: bool) -> str:
    """Draw until the stream goes quiet, the port dies, or the user quits. Returns why."""
    try:
        link, stop, thread = open_and_read(port, baud, stats, poll)
    except serial.SerialException as error:
        console.print(f"[red]cannot open {port} at {baud}: {error}[/]")
        return "error"
    interval = 1.0 / REFRESH_PER_SECOND
    try:
        with raw_keyboard(), Live(console=console, screen=True, auto_refresh=False) as live:
            while True:
                time.sleep(interval)
                now = time.monotonic()
                view = snapshot(stats, now)
                live.update(render(port, baud, view), refresh=True)
                key = pending_key()
                if key in ("q", "Q"):
                    return "quit"
                if key in ("r", "R"):
                    with stats.lock:
                        stats.reset(now)
                if stop.is_set():
                    return "error"
                if view.stale > idle_limit:
                    return "idle"
    finally:
        stop.set()
        thread.join(timeout=1.0)
        link.close()


def watch_plain(port: str, baud: int, stats: Stats, interval: float, idle_limit: float, poll: bool) -> str:
    """One line per interval, for pipes and logs."""
    try:
        link, stop, thread = open_and_read(port, baud, stats, poll)
    except serial.SerialException as error:
        print(f"cannot open {port} at {baud}: {error}", file=sys.stderr, flush=True)
        return "error"
    try:
        while True:
            time.sleep(interval)
            view = snapshot(stats, time.monotonic())
            messages = " ".join(f"{name}:{rate:.1f}" for name, _, rate, _ in view.rows)
            print(f"{time.strftime('%H:%M:%S')}  {view.rate:8.0f} B/s  {human_rate(view.rate):>12}  "
                  f"{view.rate * 1000 / baud:3.0f}%  {messages}", flush=True)
            if stop.is_set():
                return "error"
            if view.stale > idle_limit:
                print(f"  quiet for {view.stale:.0f}s -- re-autobauding", flush=True)
                return "idle"
    finally:
        stop.set()
        thread.join(timeout=1.0)
        link.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-p", "--port", default="/dev/ttyUSB1")
    parser.add_argument("-b", "--baud", type=int, help="skip autobaud and use this rate")
    parser.add_argument("-i", "--interval", type=float, default=2.0, help="--plain report interval")
    parser.add_argument("-w", "--window", type=float, default=1.2, help="seconds to listen per candidate baud")
    parser.add_argument("--bauds", help="comma-separated candidate list")
    parser.add_argument("--idle", type=float, default=6.0, help="seconds of silence before returning to autobaud")
    parser.add_argument("--no-tx", action="store_true", help="never transmit (skips the MON-VER poll)")
    parser.add_argument("--plain", action="store_true", help="print lines instead of drawing")
    arguments = parser.parse_args()

    console = Console()
    drawing = HAVE_RICH and not arguments.plain and sys.stdout.isatty()
    if not HAVE_RICH and not arguments.plain:
        console.print("[dim]rich is not installed (pip install rich) — printing lines instead[/]")
    bauds = [int(value) for value in arguments.bauds.split(",")] if arguments.bauds else BAUDS
    stats = Stats()
    poll = not arguments.no_tx
    sweep = 0

    if arguments.baud is None:
        console.print(f"autobauding [bold]{arguments.port}[/] ({arguments.window}s per rate, ctrl-c to stop)")

    while True:
        baud = arguments.baud
        if baud is None:
            sweep += 1
            baud = autobaud(console, arguments.port, arguments.window, bauds, sweep)
            if baud is None:
                continue
            console.print(f"-> [bold green]{baud}[/] baud")
        if drawing:
            reason = watch(console, arguments.port, baud, stats, arguments.idle, poll)
        else:
            reason = watch_plain(arguments.port, baud, stats, arguments.interval, arguments.idle, poll)
        if reason == "quit":
            return 0
        if reason == "error":
            time.sleep(1.0)
        elif arguments.baud is not None:
            console.print(f"[yellow]no data for {arguments.idle:.0f}s[/] at {baud} baud, still listening")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
