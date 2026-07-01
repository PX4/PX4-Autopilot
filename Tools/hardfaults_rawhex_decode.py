#!/usr/bin/env python3
# Copyright (c) 2026, Auterion AG
# SPDX-License-Identifier: BSD-3-Clause

"""
Decode a NuttX hardfault info_s hex dump into postmortem debug artifacts.

Accepts a plain hex file, a tab-separated data_streamer log, or a chunked
hardfault_stream log (offset-prefixed chunks + CRC line).

Typical usage:

    python3 scripts/hardfaults_decode.py dump.dmp
    python3 scripts/hardfaults_decode.py dump.dmp --cpu m7

Output files:
  coredump_<name>.txt  - CrashDebug-compatible memory + register dump
                         (pass directly via --coredump to emdbg.bench.fmu)
  <name>.hardfault.log - PX4-style human-readable fault log; also loadable
                         via --coredump (goes through emdbg.analyze.hardfault)

Supported --cpu values: M4 (default), M7.

Porting: the fullcontext_s struct layout is identical across all NuttX ARMv7-M
targets. Only task_name_size and filename_size in _L are board-specific
(CONFIG_TASK_NAME_SIZE / MAX_FILE_PATH_LENGTH); update _L if they differ.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

_HEX_TOKEN = re.compile(r"[0-9a-fA-F]+")


# ---------------------------------------------------------------------------
# Struct layout - ARMv7-M with FPU (NuttX XCPTCONTEXT_REGS = 53)
#
# SW-saved:  R13, BASEPRI, R4-R11, EXC_RETURN, S16-S31  (27 words)
# HW-saved:  R0-R3, R12, LR, PC, XPSR, S0-S15, FPSCR, reserved  (26 words)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _Layout:
    reg_count: int
    # fault_reg_count=7 for Cortex-M4: the struct only has 6 fault registers
    # (cfsr/hfsr/dfsr/mmfsr/bfar/afsr); the 7th read deliberately "steals"
    # stacks.user.sp so that fault_regs[6] == user.sp, which _parse() then
    # uses as user_sp.  The remaining 5 stack words (user.top/size +
    # interrupt.sp/top/size) are consumed by the 3*4 + 2*4 block below.
    fault_reg_count: int
    task_name_size: int      # CONFIG_TASK_NAME_SIZE + 1  (includes null terminator)
    filename_size: int       # MAX_FILE_PATH_LENGTH
    struct_padding: int      # trailing padding bytes in info_s so sizeof(info_s) is a multiple of 4
    idx_sp: int
    idx_basepri: int
    idx_r4: int; idx_r5: int; idx_r6: int; idx_r7: int
    idx_r8: int; idx_r9: int; idx_r10: int; idx_r11: int
    idx_exc_return: int
    idx_r0: int; idx_r1: int; idx_r2: int; idx_r3: int
    idx_r12: int; idx_lr: int; idx_pc: int; idx_xpsr: int

    @property
    def total_size(self) -> int:
        # Must equal sizeof(info_s): raw field bytes (341) + struct_padding (3) = 344.
        # C rounds struct size to the largest member's alignment (uint32_t = 4 bytes).
        return (4 * 4                          # flags, current_regs, lineno, pid
                + self.reg_count * 4
                + self.fault_reg_count * 4     # includes stolen user.sp word
                + 3 * 4                        # user.top, user.size, interrupt.sp
                + 2 * 4                        # interrupt.top, interrupt.size
                + self.task_name_size
                + self.filename_size
                + self.struct_padding)         # trailing alignment pad

# Cortex-M4:
#   CONFIG_TASK_NAME_SIZE      = 24  → name buffer = 25 bytes (includes null terminator)
#   MAX_FILE_PATH_LENGTH       = 40
#   sizeof(info_s) field bytes = 341; sizeof(info_s) = 344 (3-byte trailing pad for 4-byte alignment)
#   iStack-size and uStack-size are read from the dump itself
#   (irq_stack.size / payload size)
_L = _Layout(
    reg_count=53, fault_reg_count=7, task_name_size=25, filename_size=40, struct_padding=3,
    idx_sp=0, idx_basepri=1,
    idx_r4=2, idx_r5=3, idx_r6=4, idx_r7=5,
    idx_r8=6, idx_r9=7, idx_r10=8, idx_r11=9,
    idx_exc_return=10,
    # 11-26: S16-S31
    idx_r0=27, idx_r1=28, idx_r2=29, idx_r3=30,
    idx_r12=31, idx_lr=32, idx_pc=33, idx_xpsr=34,
    # 35-50: S0-S15  51: FPSCR  52: reserved
)

# Stack arrays appended after info_s.  istack and ustack have DIFFERENT sizes
# (CONFIG_ARCH_INTERRUPTSTACK / 4 vs ustack).
# Each array is center-indexed: array[CENTER] is the word at SP.
# array[CENTER + k] → address SP - k*4   (below SP, deeper in call stack)
# array[CENTER - k] → address SP + k*4   (above SP, toward stack top)


# ---------------------------------------------------------------------------
# Input: hex extraction
# ---------------------------------------------------------------------------

_CHUNK_OFFSET_RE = re.compile(r"^([0-9a-fA-F]{4})\s(.*)$")
_CRC_LINE_RE     = re.compile(r"^crc\s+([0-9a-fA-F]{8})$")


def _crc32part(data: bytes, crc: int = 0) -> int:
    """NuttX crc32part: reflected CRC32, no init/final XOR."""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc & 0xFFFFFFFF


def _load_payload(path: Path) -> tuple[bytes, list[str]]:
    """
    Parse *path* and return (payload_bytes, warnings).

    Handles 3 file-formats:
    - hardfault_stream log  - tab-separated, column after module-name is ``XXXX <hex>``
                              with a trailing ``crc XXXXXXXX`` line
    - data_streamer log     - tab-separated, column after module-name is raw hex
    - plain hex file        - just hex tokens, no module marker

    For hardfault_stream the chunks are assembled by their byte offset so gaps
    are detected. The CRC is verified over the original ASCII hex lines.
    """
    chunks: dict[int, bytes] = {} # offset → data  (hardfault_stream)
    plain_parts: list[str]   = [] # simple token accumulation (others)
    expected_crc: int | None = None
    crc_acc = 0xFFFFFFFF
    is_chunked = False

    for line in path.read_text().splitlines():
        module_name = "hardfault_stream"
        if module_name in line:
            seg = line.split(module_name, maxsplit=1)[1].strip()

            # CRC line: "crc XXXXXXXX"
            m_crc = _CRC_LINE_RE.match(seg)
            if m_crc:
                expected_crc = int(m_crc.group(1), 16)
                break

            # Chunk line: "XXXX <hexdata>"
            m_off = _CHUNK_OFFSET_RE.match(seg)
            if m_off:
                is_chunked = True
                offset   = int(m_off.group(1), 16)
                hex_part = "".join(
                    t.group() for t in _HEX_TOKEN.finditer(m_off.group(2))
                    if len(t.group()) % 2 == 0
                )
                data = bytes.fromhex(hex_part)
                chunks[offset] = data
                ascii_line = f"{offset:04x} {hex_part}"
                crc_acc = _crc32part(ascii_line.encode(), crc_acc)
                continue

            # data_streamer: raw hex after marker
            for t in _HEX_TOKEN.finditer(seg):
                if len(t.group()) % 2 == 0:
                    plain_parts.append(t.group())

        else:
            # Plain hex file (no marker found)
            for t in _HEX_TOKEN.finditer(line):
                if len(t.group()) % 2 == 0:
                    plain_parts.append(t.group())

    warnings: list[str] = []

    if is_chunked:
        if not chunks:
            raise SystemExit("No hardfault_stream chunks found")

        offsets = sorted(chunks)
        # Detect chunk size from the first gap
        chunk_size = (offsets[1] - offsets[0]) if len(offsets) > 1 else len(chunks[offsets[0]])
        total      = offsets[-1] + len(chunks[offsets[-1]])

        missing = [off for off in range(0, total, chunk_size) if off not in chunks]
        if missing:
            warnings.append(
                f"WARN: {len(missing)} missing chunk(s) at offsets: "
                + ", ".join(f"0x{o:04x}" for o in missing)
            )

        buf = bytearray(total)
        for off, data in chunks.items():
            buf[off:off + len(data)] = data
        payload = bytes(buf)

        # CRC verification
        if expected_crc is not None:
            computed = crc_acc ^ 0xFFFFFFFF
            if computed != expected_crc:
                warnings.append(
                    f"WARN: CRC mismatch - expected 0x{expected_crc:08x}, "
                    f"got 0x{computed:08x} (possible bit-flip in stream)"
                )
        else:
            warnings.append("WARN: no CRC line found in stream")

    else:
        payload = bytes.fromhex("".join(plain_parts))

    if len(payload) < _L.total_size:
        raise SystemExit(f"Payload too short: {len(payload)} B, need ≥{_L.total_size} B")

    return payload, warnings


# ---------------------------------------------------------------------------
# Struct parsing
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class StackRegion:
    top: int   # high address of allocated region (base + size)
    size: int  # total allocated bytes

    @property
    def base(self) -> int:
        return (self.top - self.size) & 0xFFFFFFFF


@dataclass(frozen=True)
class ParsedHardfault:
    flags: int
    current_regs: int
    lineno: int
    pid: int
    regs: list[int]
    fault_regs: list[int]
    user_stack: StackRegion
    user_sp: int          # PSP at fault time (from fault_regs[6])
    irq_stack: StackRegion
    irq_sp: int           # MSP at fault time (from user_stack field 2)
    istack_bytes: int     # = irq_stack.size = CONFIG_ARCH_INTERRUPTSTACK
    ustack_bytes: int     # = payload_size - total_size - istack_bytes
    task_name: str
    filename: str
    raw: bytes


def _parse(raw: bytes) -> ParsedHardfault:
    def u32(off: int) -> int:
        return int.from_bytes(raw[off: off + 4], "little")

    off = 0
    flags      = u32(off); off += 4
    cur_regs   = u32(off); off += 4
    lineno     = u32(off); off += 4
    pid        = u32(off); off += 4

    regs       = [u32(off + i * 4) for i in range(_L.reg_count)];       off += _L.reg_count * 4
    fault_regs = [u32(off + i * 4) for i in range(_L.fault_reg_count)]; off += _L.fault_reg_count * 4

    user_top  = u32(off); user_size = u32(off + 4); irq_sp = u32(off + 8); off += 12
    irq_top   = u32(off); irq_size  = u32(off + 4);                        off += 8
    user_stack = StackRegion(user_top, user_size)
    irq_stack  = StackRegion(irq_top,  irq_size)
    user_sp    = fault_regs[6] if len(fault_regs) > 6 else 0

    # Scan the remaining string region for the two null-terminated fields.
    # Skipping inter-field null padding makes this robust across different
    # CONFIG_TASK_NAME_SIZE / filename-buffer-size build configurations.
    strings = [s.decode("ascii", errors="replace")
               for s in raw[off: off + _L.task_name_size + _L.filename_size].split(b"\x00")
               if s]
    task_name = strings[0] if strings else ""
    filename  = strings[1] if len(strings) > 1 else ""

    istack_bytes = irq_stack.size
    ustack_bytes = max(0, len(raw) - _L.total_size - istack_bytes)

    return ParsedHardfault(
        flags=flags, current_regs=cur_regs, lineno=lineno, pid=pid,
        regs=regs, fault_regs=fault_regs,
        user_stack=user_stack, user_sp=user_sp,
        irq_stack=irq_stack, irq_sp=irq_sp,
        istack_bytes=istack_bytes, ustack_bytes=ustack_bytes,
        task_name=task_name, filename=filename, raw=raw,
    )


# ---------------------------------------------------------------------------
# Output formatters
# ---------------------------------------------------------------------------

_ADDR_RANGES = [
    (0x2000_0000, 0x80000),
    (0x2400_0000, 0x80000),
    (0x3000_0000, 0x48000),
    (0x3800_0000, 0x10000),
    (0x3880_0000, 0x01000),
    (0xE004_2000, 4),
    (0x5C00_1000, 4),
    (0xE000_E000, 0xFFF),
]

_UNKNOWN = 0xFEEDC0DE
_CPUIDS = {
    "m4": 0x410FC240,  # Cortex-M4 r0p0 (STM32F412)
    "m7": 0x411FC271,  # Cortex-M7 r1p1 (STM32H7)
}

_REG_NAMES = [
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc",
    "xpsr", "msp", "psp", "primask", "basepri", "faultmask", "control", "fpscr",
    "s0",  "s1",  "s2",  "s3",  "s4",  "s5",  "s6",  "s7",
    "s8",  "s9",  "s10", "s11", "s12", "s13", "s14", "s15", "s16",
    "s17", "s18", "s19", "s20", "s21", "s22", "s23",
    "s24", "s25", "s26", "s27", "s28", "s29", "s30", "s31",
    "d0",  "d1",  "d2",  "d3",  "d4",  "d5",  "d6",  "d7",
    "d8",  "d9",  "d10", "d11", "d12", "d13", "d14", "d15",
]


def format_coredump(p: ParsedHardfault, cpuid: int) -> str:
    base = p.current_regs - (_L.idx_r0 * 4) - 16

    # Map the struct bytes at their base address in SRAM
    mems: dict[int, int] = {
        base + i: int.from_bytes(p.raw[i: i + 4], "little")
        for i in range(0, _L.total_size, 4)
    }

    # Map istack and ustack using center-based addressing.
    # array[CENTER + k] → SP - k*4  (below SP)
    # array[CENTER - k] → SP + k*4  (above SP)
    irq_dump  = p.raw[_L.total_size:                          _L.total_size + p.istack_bytes]
    user_dump = p.raw[_L.total_size + p.istack_bytes:         _L.total_size + p.istack_bytes + p.ustack_bytes]
    irq_words   = p.istack_bytes // 4
    user_words  = p.ustack_bytes // 4
    irq_center  = irq_words  // 2
    user_center = user_words // 2
    for idx in range(irq_words):
        addr = (p.irq_sp  + (irq_center  - idx) * 4) & 0xFFFFFFFF
        mems[addr] = int.from_bytes(irq_dump[idx * 4: idx * 4 + 4],  "little")
    for idx in range(user_words):
        addr = (p.user_sp + (user_center - idx) * 4) & 0xFFFFFFFF
        mems[addr] = int.from_bytes(user_dump[idx * 4: idx * 4 + 4], "little")

    r = p.regs
    regs = {n: _UNKNOWN for n in _REG_NAMES}
    regs.update({
        "r0": r[_L.idx_r0], "r1": r[_L.idx_r1], "r2": r[_L.idx_r2], "r3": r[_L.idx_r3],
        "r4": r[_L.idx_r4], "r5": r[_L.idx_r5], "r6": r[_L.idx_r6], "r7": r[_L.idx_r7],
        "r8": r[_L.idx_r8], "r9": r[_L.idx_r9], "r10": r[_L.idx_r10], "r11": r[_L.idx_r11],
        "r12": r[_L.idx_r12], "sp": r[_L.idx_sp], "lr": r[_L.idx_lr], "pc": r[_L.idx_pc],
        "xpsr": r[_L.idx_xpsr], "msp": r[_L.idx_sp], "psp": r[_L.idx_sp],
        "basepri": r[_L.idx_basepri], "control": 0,
    })

    cfsr  = p.fault_regs[0]
    hfsr  = p.fault_regs[1]
    dfsr  = p.fault_regs[2]
    mmfar = p.fault_regs[3]
    bfar  = p.fault_regs[4]
    afsr  = p.fault_regs[5]
    mems[0x5C00_1000] = 0x10030450
    mems[0xE004_2000] = 0x10030451
    mems[0xE000_ED00] = cpuid
    mems[0xE000_ED28] = cfsr
    mems[0xE000_ED2C] = hfsr
    mems[0xE000_ED30] = dfsr
    mems[0xE000_ED34] = mmfar
    mems[0xE000_ED38] = bfar
    mems[0xE000_ED3C] = afsr

    out = []
    for start, size in _ADDR_RANGES:
        for addr in range(start, start + size, 16):
            out.append("{:#8x}:\t{:#8x}\t{:#8x}\t{:#8x}\t{:#8x}".format(
                addr,
                mems.get(addr,      _UNKNOWN),
                mems.get(addr + 4,  _UNKNOWN),
                mems.get(addr + 8,  _UNKNOWN),
                mems.get(addr + 12, _UNKNOWN),
            ))
    for name, val in regs.items():
        out.append(f"{name:15} {val:#20x} {val:20}")
    return "\n".join(out)


def format_hardfault_log(p: ParsedHardfault, cpuid: int) -> str:
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%d-%H:%M:%S")
    r = p.regs
    lines = [
        f"[data_streamer] -- {ts} Begin Fault Log --",
        f"System fault Occurred on: {ts}",
        f" Type:Hard Fault in file:{p.filename} at line: {p.lineno} running task: {p.task_name}",
        " FW git-hash: unknown",
        " Build datetime: unknown unknown",
        " Build url: unknown ",
    ]

    if p.flags & (0x01 | 0x40):
        lines += [
            f" Processor registers: from 0x{p.current_regs:08x}",
            (f" r0:0x{r[_L.idx_r0]:08x} r1:0x{r[_L.idx_r1]:08x}"
             f"  r2:0x{r[_L.idx_r2]:08x}  r3:0x{r[_L.idx_r3]:08x}"
             f"  r4:0x{r[_L.idx_r4]:08x}  r5:0x{r[_L.idx_r5]:08x}"
             f" r6:0x{r[_L.idx_r6]:08x} r7:0x{r[_L.idx_r7]:08x}"),
            (f" r8:0x{r[_L.idx_r8]:08x} r9:0x{r[_L.idx_r9]:08x}"
             f" r10:0x{r[_L.idx_r10]:08x} r11:0x{r[_L.idx_r11]:08x}"
             f" r12:0x{r[_L.idx_r12]:08x}  sp:0x{r[_L.idx_sp]:08x}"
             f" lr:0x{r[_L.idx_lr]:08x} pc:0x{r[_L.idx_pc]:08x}"),
            f" xpsr:0x{r[_L.idx_xpsr]:08x} basepri:0x{r[_L.idx_basepri]:08x} control:0x00000000",
            f" exe return:0x{r[_L.idx_exc_return]:08x}",
        ]

    if p.flags & 0x08 and p.flags & 0x01:
        cfsr = p.fault_regs[0]
        hfsr = p.fault_regs[1]
        dfsr = p.fault_regs[2]
        bfar = p.fault_regs[4]
        afsr = p.fault_regs[5]
        lines += [
            " Fault status registers: from NVIC",
            f"  CFSR: {cfsr:08x} HFSR: {hfsr:08x} DFSR: {dfsr:08x} BFAR: {bfar:08x} AFSR: {afsr:08x}",
        ]

    def _stackdump(dump: bytes, sp: int, top: int) -> list[str]:
        """Format 8-words-per-line using center-based addressing.

        dump[CENTER*4 + sp - addr] is the word at `addr`.
        Prints from (sp & ~0x1F) up to top.
        """
        result = []
        center_off = (len(dump) // 4 // 2) * 4   # stack_center * 4
        addr = sp & ~0x1F
        end  = (top + 3) & ~3
        while addr < end:
            words = []
            for i in range(0, 32, 4):
                byte_off = center_off + sp - (addr + i)
                if 0 <= byte_off <= len(dump) - 4:
                    words.append(f"0x{int.from_bytes(dump[byte_off:byte_off+4], 'little'):08x}")
                else:
                    break
            if words:
                result.append(f"0x{addr:08x} {' '.join(words)}")
            addr += 32
        return result

    def _stack_region(name: str, sp: int, region: StackRegion, dump: bytes) -> list[str]:
        out = [
            f" {name} Stack:",
            f" sp:     {sp:08x}",
            f"   base: {region.base:08x}",
            f"   size: {region.size:08x}",
        ]
        if region.base <= sp < region.top:
            out += _stackdump(dump, sp, region.top)
        else:
            out.append(f" ERROR: {name} Stack pointer is not within the stack")
        return out

    irq_dump  = p.raw[_L.total_size:                          _L.total_size + p.istack_bytes]
    user_dump = p.raw[_L.total_size + p.istack_bytes:         _L.total_size + p.istack_bytes + p.ustack_bytes]

    if p.flags & (0x04 | 0x08):
        lines += _stack_region("IRQ",  p.irq_sp,  p.irq_stack,  irq_dump)
    if p.flags & 0x02:
        lines += _stack_region("User", p.user_sp, p.user_stack, user_dump)

    lines.append(f"0xe000ed00 {cpuid:#010x}") # expected by emdbg - CrashDebug for Cortex-M4/M7
    lines.append(f"[data_streamer] -- {ts} END Fault Log --")
    return "\n".join(lines) + "\n"

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Decode a NuttX hardfault hex dump for postmortem debugging"
    )
    parser.add_argument("blob", type=Path,
                        help="Hex dump file: plain hex string or data_streamer log")
    parser.add_argument("--cpu", choices=list(_CPUIDS), default="m4",
                        help="Target CPU (default: m4)")
    args = parser.parse_args()

    payload, warnings = _load_payload(args.blob)
    p = _parse(payload)
    cpuid = _CPUIDS[args.cpu]

    out_log = args.blob.with_suffix(".hardfault.log")
    out_coredump = args.blob.parent / f"coredump_{args.blob.stem}.txt"

    out_coredump.write_text(format_coredump(p, cpuid))
    out_log.write_text(format_hardfault_log(p, cpuid))

    for w in warnings:
        print(w)
    print(f"Size:  total={len(payload)} B  istack={p.istack_bytes} B  ustack={p.ustack_bytes} B")
    print(f"Task:  {p.task_name!r}  pid={p.pid}  line={p.lineno}")
    print(f"File:  {p.filename!r}")
    print(f"PC:    0x{p.regs[_L.idx_pc]:08x}  LR: 0x{p.regs[_L.idx_lr]:08x}  SP: 0x{p.regs[_L.idx_sp]:08x}")
    print(f"Wrote: {out_log}")
    print(f"Wrote: {out_coredump}")

    return 1 if any(w.startswith("WARN: CRC") for w in warnings) else 0

if __name__ == "__main__":
    raise SystemExit(main())
