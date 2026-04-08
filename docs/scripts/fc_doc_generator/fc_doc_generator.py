#!/usr/bin/env python3
"""
FC Doc Generator — parse PX4 board source files and generate flight controller
documentation sections.

Currently generates: ## Specifications, ## PWM Outputs, ## Radio Control, ## GPS & Compass

Extension pattern — to add a new section:
  1. Write parse_<thing>(board_path) -> dict and call it in gather_board_data(),
     merging the result into the board entry.
  2. Write generate_<thing>_section(board_key, entry) -> str.
  3. Register both in SECTION_GENERATORS and SECTION_ORDER below.
  4. Re-run fc_doc_generator.py (yarn gen_fc_sections) to regenerate metadata/ JSON + fc_sections.md and apply to docs.
"""

import argparse
import re
import json
from pathlib import Path


def _find_repo_root(start: Path) -> Path:
    """Walk up from start until a directory containing 'boards/' is found."""
    for p in [start, *start.parents]:
        if (p / "boards").is_dir():
            return p
    raise FileNotFoundError(f"Could not find PX4 repo root (no 'boards/' dir) from {start}")


def _make_slug(s: str) -> str:
    """Lowercase, collapse spaces/hyphens to underscores, strip non-ASCII."""
    s = ''.join(c for c in s if ord(c) < 128)  # ASCII-only (removes surrogates too)
    return re.sub(r'[\s\-]+', '_', s.strip().lower())


def _doc_filename(manufacturer: str, product: str, fmu_version: str = None) -> str:
    """holybro + Pixhawk 6X + fmu-v6x  →  holybro_pixhawk_6x_fmu_v6x.md"""
    parts = [_make_slug(manufacturer), _make_slug(product)]
    if fmu_version:
        parts.append(_make_slug(fmu_version))
    return "_".join(parts) + ".md"


SCRIPT_DIR = Path(__file__).resolve().parent
METADATA_DIR = SCRIPT_DIR / "metadata"
REPO = _find_repo_root(SCRIPT_DIR)
BOARDS = REPO / "boards"
FC_DOCS = REPO / "docs/en/flight_controller"

# ---------------------------------------------------------------------------
# Serial UART hardware ordering by chip family.
# Matches the order NuttX registers devices: console → /dev/ttyS0, then the
# rest in this order as /dev/ttyS1, /dev/ttyS2, …
# ---------------------------------------------------------------------------
UART_HW_ORDER = {
    'stm32h7': ['USART1', 'USART2', 'USART3', 'UART4', 'UART5', 'USART6', 'UART7', 'UART8'],
    'stm32f7': ['USART1', 'USART2', 'USART3', 'UART4', 'UART5', 'USART6', 'UART7', 'UART8'],
    'stm32f4': ['USART1', 'USART2', 'USART3', 'UART4', 'UART5', 'USART6'],
    'stm32f1': ['USART1', 'USART2', 'USART3', 'UART4', 'UART5'],
    'imxrt':   ['LPUART1', 'LPUART2', 'LPUART3', 'LPUART4',
                'LPUART5', 'LPUART6', 'LPUART7', 'LPUART8',
                'LPUART9', 'LPUART10', 'LPUART11'],
}

# ---------------------------------------------------------------------------
# BDShot per-channel capture support by chip family and timer
# Source: hw_description.h getTimerChannelDMAMap() per family
# For each timer, list which channel indices (0-based) support capture DMA.
# None = timer not in map at all (no capture)
# ---------------------------------------------------------------------------
BDSHOT_CAPTURE_MAP = {
    "stm32h7": {
        # All 4 channels have dma_map_ch except Timer4 ch3 (index 3 = CH4) = 0
        "Timer1":  [0, 1, 2, 3],
        "Timer2":  [0, 1, 2, 3],
        "Timer3":  [0, 1, 2, 3],
        "Timer4":  [0, 1, 2],   # CH4 (index 3) = 0
        "Timer5":  [0, 1, 2, 3],
        "Timer8":  [0, 1, 2, 3],
        "Timer15": [0],
        "Timer16": [0],
        "Timer17": [0],
    },
    "stm32f7": {
        # Only Timer1 and Timer4 have channel DMA; Timer4 CH4 = 0
        "Timer1":  [0, 1, 2, 3],
        "Timer4":  [0, 1, 2],   # CH4 = 0
    },
    "stm32f4": {
        # getTimerChannelDMAMap is explicitly "Not supported" — no BDShot at all
        "_none": True,
    },
    "stm32f1": {
        # Not supported
        "_none": True,
    },
    "imxrt": {
        # All outputs support BDShot (docs: "All FMU outputs" on iMXRT)
        "_all": True,
    },
}

# ---------------------------------------------------------------------------
# Chip model → hardware specification lookup
# Keyed by model number without package/revision suffix (e.g. "STM32H743").
# Used by generate_specifications_section() to produce the Processor bullet.
# ---------------------------------------------------------------------------
CHIP_SPECS = {
    # STM32H7 family — Cortex-M7, 480 MHz, 2MB flash, 1MB RAM
    'STM32H743': {'core': 'Cortex®-M7', 'mhz': 480, 'flash': '2MB', 'ram': '1MB'},
    'STM32H753': {'core': 'Cortex®-M7', 'mhz': 480, 'flash': '2MB', 'ram': '1MB'},
    'STM32H747': {'core': 'Cortex®-M7', 'mhz': 480, 'flash': '2MB', 'ram': '1MB'},
    'STM32H757': {'core': 'Cortex®-M7', 'mhz': 480, 'flash': '2MB', 'ram': '1MB'},
    # STM32F7 family — Cortex-M7, 216 MHz
    'STM32F765': {'core': 'Cortex®-M7', 'mhz': 216, 'flash': '2MB', 'ram': '512KB'},
    'STM32F767': {'core': 'Cortex®-M7', 'mhz': 216, 'flash': '2MB', 'ram': '512KB'},
    'STM32F777': {'core': 'Cortex®-M7', 'mhz': 216, 'flash': '2MB', 'ram': '512KB'},
    # STM32F4 family — Cortex-M4, 168 MHz
    'STM32F427': {'core': 'Cortex®-M4', 'mhz': 168, 'flash': '2MB', 'ram': '256KB'},
    'STM32F437': {'core': 'Cortex®-M4', 'mhz': 168, 'flash': '2MB', 'ram': '256KB'},
    'STM32F407': {'core': 'Cortex®-M4', 'mhz': 168, 'flash': '1MB',  'ram': '192KB'},
    'STM32F405': {'core': 'Cortex®-M4', 'mhz': 168, 'flash': '1MB',  'ram': '192KB'},
    # iMXRT family — Cortex-M7, 600 MHz
    'MIMXRT1062': {'core': 'Cortex®-M7', 'mhz': 600, 'flash': '8MB', 'ram': '1MB'},
}

# IO co-processor spec (always STM32F100 on PX4 io-v2)
_IO_PROC_SPEC = 'STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)'


def get_chip_family(board_path: Path) -> str:
    """Detect the MCU chip family for a board.

    Mechanism
    ---------
    Primary source: ``nuttx-config/nsh/defconfig``
    NuttX's Kconfig system sets exactly one ``CONFIG_ARCH_CHIP_*`` symbol for
    the target MCU.  The symbols are mutually exclusive, so the first match is
    authoritative.  Patterns checked (in order):

      CONFIG_ARCH_CHIP_STM32H7   → 'stm32h7'
      CONFIG_ARCH_CHIP_STM32F7   → 'stm32f7'
      CONFIG_ARCH_CHIP_STM32F<N> → 'stm32f4'  (F4/F3/F2 era, treated as F4)
      CONFIG_ARCH_CHIP_STM32=y   → 'stm32f1'
      IMXRT (case-insensitive)   → 'imxrt'

    Fallback: scan any ``CMakeLists.txt`` under the board directory for the
    string "imxrt" (catches boards that configure the chip via cmake rather
    than Kconfig).

    Returns 'unknown' when no pattern matches; callers that depend on family
    (e.g. BDSHOT_CAPTURE_MAP, UART_HW_ORDER) will produce empty output rather
    than incorrect data.
    """
    defconfig = board_path / "nuttx-config/nsh/defconfig"
    if defconfig.exists():
        text = defconfig.read_text(errors="ignore")
        if "CONFIG_ARCH_CHIP_STM32H7" in text:
            return "stm32h7"
        if "CONFIG_ARCH_CHIP_STM32F7" in text:
            return "stm32f7"
        if re.search(r'CONFIG_ARCH_CHIP_STM32F[0-9]', text):
            return "stm32f4"  # F4 and F1 era
        if re.search(r'CONFIG_ARCH_CHIP_STM32=y|CONFIG_ARCH_CHIP_STM32F1', text):
            return "stm32f1"
        if re.search(r'IMXRT|imxrt', text, re.IGNORECASE):
            return "imxrt"
    # Try cmake
    for cmake in board_path.rglob("CMakeLists.txt"):
        txt = cmake.read_text(errors="ignore")
        if "imxrt" in txt.lower():
            return "imxrt"
    return "unknown"


def parse_chip_variant(board_path: Path) -> dict:
    """Extract the exact MCU chip model from nuttx-config/nsh/defconfig.

    Reads ``CONFIG_ARCH_CHIP_<VARIANT>=y`` and strips the 2-character package
    suffix to produce a model string suitable as a key into CHIP_SPECS.

    Examples
    --------
      CONFIG_ARCH_CHIP_STM32H743VI=y  →  chip_model='STM32H743'
      CONFIG_ARCH_CHIP_STM32F427II=y  →  chip_model='STM32F427'
      CONFIG_ARCH_CHIP_MIMXRT1062=y   →  chip_model='MIMXRT1062'

    Returns
    -------
    {'chip_model': str | None, 'chip_variant_full': str | None}
    """
    defconfig = board_path / 'nuttx-config/nsh/defconfig'
    if defconfig.exists():
        text = defconfig.read_text(errors='ignore')
        # STM32 variants: CONFIG_ARCH_CHIP_STM32H743VI=y
        m = re.search(r'CONFIG_ARCH_CHIP_(STM32[A-Z0-9]+)=y', text)
        if m:
            full = m.group(1)           # e.g. "STM32H743VI"
            # Strip 2-char package/revision suffix for model lookup
            model = full[:-2] if len(full) > 8 else full
            return {'chip_model': model, 'chip_variant_full': full}
        # iMXRT variants: CONFIG_ARCH_CHIP_MIMXRT1062CVL5A=y
        m = re.search(r'CONFIG_ARCH_CHIP_(MIMXRT[A-Z0-9]+)=y', text)
        if m:
            full = m.group(1)
            # iMXRT model = first 10 chars (e.g. "MIMXRT1062")
            model = full[:10]
            return {'chip_model': model, 'chip_variant_full': full}
    return {'chip_model': None, 'chip_variant_full': None}


def parse_board_config(board_path: Path) -> dict:
    """Parse PWM output count and IO-board presence from board_config.h.

    Mechanism
    ---------
    Source file: ``src/board_config.h``

    **Total FMU PWM outputs** — macro ``DIRECT_PWM_OUTPUT_CHANNELS``:
      #define DIRECT_PWM_OUTPUT_CHANNELS  6
    This is the number of PWM lines driven directly by the FMU (i.e. the AUX
    outputs on boards with an IO board, or MAIN outputs on boards without one).

    **IO board detection** — presence of any of:
      PX4IO_SERIAL, HAVE_PX4IO, PX4IO_DEVICE_PATH
    These are defined whenever the board routes a UART to a co-processor
    running PX4IO firmware (the io-v2 board), which provides 8 additional
    "MAIN" PWM outputs.  The IO output count is therefore hardcoded to 8
    (the io-v2 standard) and does not need to be parsed.

    Returns a dict with keys:
      total_outputs   int  — FMU PWM channel count (may be absent if not found)
      has_io_board    bool — True if a PX4IO co-processor is present
      io_outputs      int  — 8 if has_io_board, else 0
    """
    cfg = board_path / "src/board_config.h"
    if not cfg.exists():
        return {}
    text = cfg.read_text(errors="ignore")
    result = {}
    m = re.search(r'#define\s+DIRECT_PWM_OUTPUT_CHANNELS\s+(\d+)', text)
    if m:
        result["total_outputs"] = int(m.group(1))
    m = re.search(r'#define\s+BOARD_NUM_IO_TIMERS\s+(\d+)', text)
    if m:
        result["num_timers"] = int(m.group(1))
    # Check for IO board (MAIN outputs)
    if "PX4IO_SERIAL" in text or "HAVE_PX4IO" in text or "PX4IO_DEVICE_PATH" in text:
        result["has_io_board"] = True
        # IO board version 2 always has 8 outputs; detect version for future-proofing
        m = re.search(r'#define\s+BOARD_USES_PX4IO_VERSION\s+(\d+)', text)
        result["io_outputs"] = 8  # io-v2 standard
    else:
        result["has_io_board"] = False
        result["io_outputs"] = 0
    return result


def parse_timer_config(board_path: Path) -> dict:
    """Parse PWM timer groups and output channels from timer_config.cpp.

    Mechanism
    ---------
    Source file: ``src/timer_config.cpp``

    PX4 defines PWM outputs through two compile-time ``constexpr`` arrays:

    **io_timers[]** — one entry per hardware timer used for PWM.
    Each entry is constructed by one of:
      initIOTimer(Timer::Timer5)                        — no DShot DMA
      initIOTimer(Timer::Timer5, DMA{DMA::Index1, …})  — DShot capable

    The presence of a ``DMA{…}`` argument is the indicator that the entire
    timer group supports DShot.  The timer name (e.g. ``Timer5``) becomes the
    group identifier.

    **timer_io_channels[]** — one entry per PWM output channel, in output order
    (FMU_CH1, FMU_CH2, …).  Each entry is constructed by one of:
      initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, …)
      initIOTimerChannelOutputClear(…)   — same semantics, clears on disable
      initIOTimerChannelDshot(…)         — iMXRT only: marks channel as DShot
      initIOTimerChannelCapture(…)       — dual-purpose capture/output channel

    The channel index (Channel1 = index 0, Channel2 = index 1, …) is extracted
    and stored for use by compute_bdshot() when determining per-channel BDShot
    DMA capability.  Channels are counted sequentially to assign output numbers.

    **iMXRT special case** — NXP iMXRT boards use a PWM peripheral with a
    different API: timer names are ``PWM1_PWM_A``, ``PWM1_PWM_B``, etc.
    If io_timers[] is empty after parsing (no ``initIOTimer`` calls), the timer
    list is reconstructed by grouping channels by their ``PWMn`` prefix.
    DShot capability is inferred from any channel in the group having
    ``initIOTimerChannelDshot``.

    Returns {'timers': [...], 'channels': [...]}
    """
    tc = board_path / "src/timer_config.cpp"
    if not tc.exists():
        return {}

    text = tc.read_text(errors="ignore")

    # Extract comment block for channel assignments (if present)
    comment_assignments = []
    for line in text.split('\n'):
        # e.g. "* TIM5_CH4  T FMU_CH1"
        m = re.match(r'\s*\*\s+(TIM\w+_CH\w+)\s+\w+\s+(FMU_CH\d+)', line)
        if m:
            comment_assignments.append((m.group(1), m.group(2)))

    # Parse io_timers[] array entries
    # Handle: initIOTimer(Timer::Timer5, DMA{DMA::Index1})
    # Handle: initIOTimer(Timer::Timer3, DMA{DMA::Index1, DMA::Stream2, DMA::Channel5})
    # Handle: initIOTimer(Timer::Timer12)   -- no DMA
    timers = []
    # Find the io_timers[] array block
    io_timers_match = re.search(
        r'constexpr\s+io_timers_t\s+io_timers\[.*?\]\s*=\s*\{(.*?)\};',
        text, re.DOTALL
    )
    if io_timers_match:
        block = re.sub(r'//[^\n]*', '', io_timers_match.group(1))
        for entry in re.finditer(r'initIOTimer\s*\(\s*Timer::(\w+)(.*?)\)', block):
            timer_name = entry.group(1)
            rest = entry.group(2).strip()
            has_dma = bool(re.search(r'DMA\s*\{', rest))
            timers.append({
                "timer": timer_name,
                "dshot": has_dma,
            })

    # Parse timer_io_channels[] array entries
    # Handle: initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, ...)
    # Handle: initIOTimerChannelOutputClear(...), initIOTimerChannelDshot(...)
    # Handle: initIOTimerChannelCapture(...) -- dual-purpose capture/output channels
    channels = []
    io_channels_match = re.search(
        r'constexpr\s+timer_io_channels_t\s+timer_io_channels\[.*?\]\s*=\s*\{(.*?)\};',
        text, re.DOTALL
    )
    if io_channels_match:
        # Strip C++ line comments to avoid parsing commented-out channels
        block = re.sub(r'//[^\n]*', '', io_channels_match.group(1))
        output_idx = 1
        for entry in re.finditer(
            r'(initIOTimerChannel(?:OutputClear|Dshot|Capture)?)\s*\(\s*io_timers\s*,\s*\{(?:Timer|PWM)::(\w+)\s*,\s*(?:Timer::|PWM::)?(\w+)\}',
            block
        ):
            func_name = entry.group(1)
            timer_name = entry.group(2)
            channel_name = entry.group(3)
            is_dshot = "Dshot" in func_name
            # Skip channel mapping entries (not actual channels)
            if "Mapping" in func_name:
                continue
            # Extract channel number
            ch_num_match = re.search(r'Channel(\d+)|Submodule(\d+)|CH(\d+)', channel_name)
            ch_idx = None
            if ch_num_match:
                ch_idx = int(next(g for g in ch_num_match.groups() if g is not None)) - 1
            channels.append({
                "output_index": output_idx,
                "timer": timer_name,
                "channel": channel_name,
                "channel_index": ch_idx,
                "is_dshot_channel": is_dshot,  # iMXRT only
            })
            output_idx += 1

    # For NXP iMXRT boards: timers are derived from unique PWM module names in channels
    if not timers and channels:
        seen = {}
        for ch in channels:
            # e.g. timer = "PWM1_PWM_A" -> group key = "PWM1"
            m = re.match(r'(PWM\d+)', ch["timer"])
            key = m.group(1) if m else ch["timer"]
            if key not in seen:
                seen[key] = ch.get("is_dshot_channel", False)
            elif ch.get("is_dshot_channel"):
                seen[key] = True
        for tname, has_dshot in seen.items():
            timers.append({"timer": tname, "dshot": has_dshot})

    return {
        "timers": timers,
        "channels": channels,
    }


def _serial_label(raw: str) -> str:
    """Normalise a CONFIG_BOARD_SERIAL_* key to a display label.

    TEL1 → TELEM1, TEL2 → TELEM2, etc.  All other keys are returned as-is.
    """
    m = re.fullmatch(r'TEL(\d+)', raw)
    return f'TELEM{m.group(1)}' if m else raw


def parse_serial_config(board_path: Path) -> dict:
    """Parse serial port configuration, producing a UART → device → label mapping.

    Mechanism
    ---------

    **Labels** — ``default.px4board`` (primary, authoritative source)
    Each board's Kconfig file contains ``CONFIG_BOARD_SERIAL_<LABEL>`` entries
    that map PX4 serial-port function names to /dev/ttyS* device numbers, e.g.:

      CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"
      CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS6"
      CONFIG_BOARD_SERIAL_RC="/dev/ttyS5"

    These are the same values shown by the PX4 serial port mapping guide and
    confirmed as the correct reference in PX4-user_guide PR #672 comment by
    davids5.  The ``TEL<N>`` keys are normalised to ``TELEM<N>`` for display.

    82 of the 90 boards with timer_config.cpp have these entries.  The 8 that
    don't are non-standard targets (ESP32, Crazyflie, CAN hub, IO co-processor)
    unlikely to need FC documentation.

    **UART names** — ``nuttx-config/nsh/defconfig``
    The defconfig enables specific UARTs via Kconfig symbols, e.g.:

      CONFIG_STM32H7_USART1=y
      CONFIG_STM32H7_UART4=y

    These are collected and filtered against the chip family's known hardware
    ordering (``UART_HW_ORDER``).  The /dev/ttyS* number for each UART is its
    0-based position in that filtered ordered list — i.e. pure sequential
    assignment in hardware order.  This was validated empirically against 19 of
    the 28 existing documented boards; the 9 remaining differences are
    structural (iMXRT naming, fewer physical connections than enabled UARTs,
    shared doc files across variants) and not fixable from config alone.

    **Console/debug label** — also from defconfig
    ``CONFIG_<UART>_SERIAL_CONSOLE=y`` identifies the UART used for the NuttX
    debug console.  If that device has no label from CONFIG_BOARD_SERIAL_*
    (the debug UART is intentionally excluded from the serial-port function
    list), it is labelled "Debug Console".

    **PX4IO label** — ``src/board_config.h``
    ``#define PX4IO_SERIAL_DEVICE "/dev/ttyS<N>"`` identifies the UART that
    communicates with the PX4IO co-processor.  This device is not listed in
    CONFIG_BOARD_SERIAL_* (it is internal, not user-accessible), so it is
    injected here as a label fallback.  On boards where CONFIG_BOARD_SERIAL_RC
    points to the same device (the IO board also handles RC input), the label
    is combined to "PX4IO/RC".

    Returns {'serial_ports': [{'uart': str, 'device': str, 'label': str, 'flow_control': bool}, …]}

    **Flow control** — ``nuttx-config/include/board.h``
    A UART has hardware flow control if both its RTS and CTS GPIO macros are
    *defined* via ``#define`` (e.g. ``#define GPIO_USART2_RTS ...`` and
    ``#define GPIO_USART2_CTS ...``).  Commented-out lines (``// GPIO_...``)
    are ignored.  This reflects physical hardware wiring rather than whether
    flow control is currently enabled in the defconfig.
    """
    family = get_chip_family(board_path)
    hw_order = UART_HW_ORDER.get(family, [])
    if not hw_order:
        return {'serial_ports': []}

    defconfig = board_path / 'nuttx-config/nsh/defconfig'
    if not defconfig.exists():
        return {'serial_ports': []}

    text = defconfig.read_text(errors='ignore')

    # Enabled UARTs: CONFIG_STM32H7_USART1=y, CONFIG_STM32_UART4=y, CONFIG_IMXRT_LPUART1=y
    enabled = set()
    for m in re.finditer(
        r'CONFIG_(?:STM32\w+|IMXRT)_(USART\d+|UART\d+|LPUART\d+)=y',
        text, re.IGNORECASE
    ):
        enabled.add(m.group(1).upper())

    # Console UART: label fallback only — does not affect device numbering
    console = None
    m = re.search(r'CONFIG_(USART\d+|UART\d+|LPUART\d+)_SERIAL_CONSOLE=y', text, re.IGNORECASE)
    if m:
        console = m.group(1).upper()

    # Active UARTs in hardware order → sequential /dev/ttyS* assignment
    active = [u for u in hw_order if u in enabled]
    if not active:
        return {'serial_ports': []}

    tty_map = {uart: f'/dev/ttyS{i}' for i, uart in enumerate(active)}

    # Primary labels from default.px4board CONFIG_BOARD_SERIAL_* entries
    device_to_label = {}
    px4board = board_path / 'default.px4board'
    if px4board.exists():
        for m in re.finditer(
            r'CONFIG_BOARD_SERIAL_(\w+)="/dev/ttyS(\d+)"',
            px4board.read_text(errors='ignore')
        ):
            device_to_label[f'/dev/ttyS{m.group(2)}'] = _serial_label(m.group(1))

    # PX4IO label: board_config.h PX4IO_SERIAL_DEVICE
    #   - If that device is already labelled "RC" (same UART serves both purposes)
    #     → combine to "PX4IO/RC"
    #   - Otherwise use as fallback for unlabelled devices (e.g. fmu-v6c ttyS4)
    cfg = board_path / 'src/board_config.h'
    if cfg.exists():
        m = re.search(r'#define\s+PX4IO_SERIAL_DEVICE\s+"(/dev/ttyS\d+)"',
                      cfg.read_text(errors='ignore'))
        if m:
            px4io_dev = m.group(1)
            if device_to_label.get(px4io_dev) == 'RC':
                device_to_label[px4io_dev] = 'PX4IO/RC'
            else:
                device_to_label.setdefault(px4io_dev, 'PX4IO')

    # Console UART label fallback (debug port not listed in CONFIG_BOARD_SERIAL_*)
    if console and console in tty_map:
        device_to_label.setdefault(tty_map[console], 'Debug Console')

    # Hardware flow control: detect from board.h GPIO RTS+CTS pin definitions
    fc_uarts: set = set()
    board_h = board_path / 'nuttx-config/include/board.h'
    if board_h.exists():
        board_h_text = board_h.read_text(errors='ignore')
        for uart in active:
            if (re.search(rf'^\s*#define\s+GPIO_{uart}_RTS\b', board_h_text, re.MULTILINE) and
                    re.search(rf'^\s*#define\s+GPIO_{uart}_CTS\b', board_h_text, re.MULTILINE)):
                fc_uarts.add(uart)

    ports = []
    for uart in active:
        device = tty_map[uart]
        ports.append({
            'uart': uart,
            'device': device,
            'label': device_to_label.get(device, ''),
            'flow_control': uart in fc_uarts,
        })

    return {'serial_ports': ports}


def parse_rc_config(board_path: Path) -> dict:
    """Parse RC input capabilities from default.px4board and src/board_config.h.

    PPM detection
    -------------
    PPM (CPPM) requires a hardware timer-capture pin.  The rc_input driver
    guards all PPM logic behind ``#ifdef HRT_PPM_CHANNEL`` (line 675 of
    RCInput.cpp).  Boards that lack this define cannot receive PPM at all,
    even though the rc_input driver is present.

    Three cases:
      has_ppm_pin=False            — PPM not supported (no HRT_PPM_CHANNEL)
      has_ppm_pin=True,
        ppm_shared_with_rc_serial=True  — PPM pin == UART RX pin; same
                                          physical connector, driver
                                          auto-switches between PPM and serial
      has_ppm_pin=True,
        ppm_shared_with_rc_serial=False — dedicated PPM pin; may be the same
                                          or a different connector (not
                                          determinable from source alone)
    """
    result = {
        'has_rc_input': False,
        'has_common_rc': False,
        'rc_serial_device': None,
        'has_ppm_pin': False,
        'ppm_shared_with_rc_serial': False,
    }
    px4board = board_path / 'default.px4board'
    if not px4board.exists():
        return result
    text = px4board.read_text(errors='ignore')
    if re.search(r'^CONFIG_DRIVERS_RC_INPUT=y', text, re.MULTILINE):
        result['has_rc_input'] = True
    if re.search(r'^CONFIG_COMMON_RC=y', text, re.MULTILINE):
        result['has_common_rc'] = True
    m = re.search(r'^CONFIG_BOARD_SERIAL_RC="(/dev/ttyS\d+)"', text, re.MULTILINE)
    if m:
        result['rc_serial_device'] = m.group(1)

    # PPM support lives in board_config.h, not default.px4board
    cfg = board_path / 'src/board_config.h'
    if cfg.exists():
        cfg_text = cfg.read_text(errors='ignore')
        if 'HRT_PPM_CHANNEL' in cfg_text or 'GPIO_PPM_IN' in cfg_text:
            result['has_ppm_pin'] = True
        if 'RC_SERIAL_PORT_SHARED_PPM_PIN_GPIO_RX' in cfg_text:
            result['ppm_shared_with_rc_serial'] = True

    return result


def parse_gps_config(board_path: Path) -> dict:
    """Parse GPS-related hardware capabilities from default.px4board and board_config.h.

    Detected fields
    ---------------
    has_pps_capture   — CONFIG_DRIVERS_PPS_CAPTURE=y in default.px4board
    has_safety_switch — GPIO_BTN_SAFETY or GPIO_SAFETY_SWITCH_IN in board_config.h
    has_safety_led    — GPIO_LED_SAFETY or GPIO_nSAFETY_SWITCH_LED_OUT in board_config.h
    has_buzzer        — GPIO_BUZZER_1 or GPIO_BUZZER in board_config.h
                        (distinct from GPIO_TONE_ALARM_IDLE which is an on-board pad;
                         GPIO_BUZZER_1 is the named pin wired to the GPS connector
                         on Pixhawk-standard boards)
    """
    result = {
        'has_pps_capture':   False,
        'has_safety_switch': False,
        'has_safety_led':    False,
        'has_buzzer':        False,
    }
    px4board = board_path / 'default.px4board'
    if px4board.exists():
        text = px4board.read_text(errors='ignore')
        if re.search(r'^CONFIG_DRIVERS_PPS_CAPTURE=y', text, re.MULTILINE):
            result['has_pps_capture'] = True
    cfg = board_path / 'src/board_config.h'
    if cfg.exists():
        cfg_text = cfg.read_text(errors='ignore')
        if 'GPIO_BTN_SAFETY' in cfg_text or 'GPIO_SAFETY_SWITCH_IN' in cfg_text:
            result['has_safety_switch'] = True
        if 'GPIO_LED_SAFETY' in cfg_text or 'GPIO_nSAFETY_SWITCH_LED_OUT' in cfg_text:
            result['has_safety_led'] = True
        if 'GPIO_BUZZER_1' in cfg_text or re.search(r'\bGPIO_BUZZER\b', cfg_text):
            result['has_buzzer'] = True
    return result


def parse_power_config(board_path: Path) -> dict:
    """Parse power supply configuration from board_config.h and default.px4board.

    Detected fields
    ---------------
    num_power_inputs           — number of power input connectors (defaults to 1 if
                                 no explicit macros found; every board has at least one)
    has_redundant_power        — True when num_power_inputs >= 2
    has_dual_battery_monitoring — True when BOARD_HAS_NBAT_V >= 2
    has_dronecan_power_input   — True when a CAN/DroneCAN power input is detected
    power_monitor_type         — "analog" | "ina226" | "ina238" | "ltc44xx" | "dronecan" | None

    Sources
    -------
    board_config.h:
      BOARD_NUMBER_BRICKS          — analog power mux inputs
      BOARD_HAS_LTC44XX_VALIDS     — LTC44xx power mux chips (one per input)
      BOARD_NUMBER_DIGITAL_BRICKS  — INA-based digital power inputs
      BOARD_HAS_NBAT_V             — battery voltage channels (may have 'd' suffix)
      GPIO_nPOWER_IN_CAN           — explicit CAN brick GPIO (Pattern A)

    default.px4board:
      CONFIG_DRIVERS_POWER_MONITOR_INA238=y
      CONFIG_DRIVERS_POWER_MONITOR_INA226=y
      CONFIG_DRIVERS_UAVCAN=y  — combined with BOARD_NUMBER_DIGITAL_BRICKS (Pattern B)
    """
    result = {
        'num_power_inputs': 1,   # every board has at least one power input
        'has_redundant_power': False,
        'has_dual_battery_monitoring': False,
        'has_dronecan_power_input': False,
        'power_monitor_type': None,
    }

    cfg = board_path / 'src/board_config.h'
    if not cfg.exists():
        return result

    text = cfg.read_text(errors='ignore')

    bricks = 0
    m = re.search(r'#define\s+BOARD_NUMBER_BRICKS\s+(\d+)', text)
    if m:
        bricks = int(m.group(1))

    ltc = 0
    m = re.search(r'#define\s+BOARD_HAS_LTC44XX_VALIDS\s+(\d+)', text)
    if m:
        ltc = int(m.group(1))

    digital = 0
    m = re.search(r'#define\s+BOARD_NUMBER_DIGITAL_BRICKS\s+(\d+)', text)
    if m:
        digital = int(m.group(1))

    num_battery_ch = 0
    m = re.search(r'#define\s+BOARD_HAS_NBAT_V\s+(\w+)', text)
    if m:
        # Strip optional 'd' suffix used for digital channels (e.g. "2d" → 2)
        num_battery_ch = int(re.sub(r'[^\d]', '', m.group(1)))

    # Pattern A: explicit CAN brick GPIO → brick 2 is DroneCAN
    if 'GPIO_nPOWER_IN_CAN' in text:
        result['has_dronecan_power_input'] = True

    total = max(bricks, ltc, digital)
    result['num_power_inputs'] = total if total > 0 else 1
    result['has_redundant_power'] = total >= 2
    result['has_dual_battery_monitoring'] = num_battery_ch >= 2

    # Determine power monitor type — px4board driver entries take precedence
    if ltc:
        result['power_monitor_type'] = 'ltc44xx'

    px4board = board_path / 'default.px4board'
    if px4board.exists():
        ptext = px4board.read_text(errors='ignore')
        if 'CONFIG_DRIVERS_POWER_MONITOR_INA238=y' in ptext:
            result['power_monitor_type'] = 'ina238'
        elif 'CONFIG_DRIVERS_POWER_MONITOR_INA226=y' in ptext:
            result['power_monitor_type'] = 'ina226'
        elif result['power_monitor_type'] is None and digital:
            result['power_monitor_type'] = 'ina226'   # generic INA fallback

        # Pattern B: digital bricks + UAVCAN driver → DroneCAN power input
        if not result['has_dronecan_power_input'] and digital > 0:
            if 'CONFIG_DRIVERS_UAVCAN=y' in ptext:
                result['has_dronecan_power_input'] = True

    if result['power_monitor_type'] is None and bricks:
        result['power_monitor_type'] = 'analog'

    return result


def parse_sd_card_config(board_path: Path) -> dict:
    """Detect SD card, Ethernet, and onboard heater from board config files.

    Sources (checked in priority order)
    ------------------------------------
    nuttx-config/nsh/defconfig:
      CONFIG_MMCSD_SDIO=y     — SD card via SDIO interface (hardware slot present)

    default.px4board:
      CONFIG_SYSTEMCMDS_SD_BENCH=y  — SD bench tool enabled (implies usable SD slot)
      CONFIG_SYSTEMCMDS_SD_STRESS=y — SD stress tool enabled (implies usable SD slot)
      CONFIG_BOARD_ETHERNET=y       — onboard Ethernet PHY
      CONFIG_DRIVERS_HEATER=y       — onboard temperature-controlled heater

    Returns
    -------
    {'has_sd_card': bool, 'has_ethernet': bool, 'has_heater': bool}
    """
    result = {'has_sd_card': False, 'has_ethernet': False, 'has_heater': False}

    # Primary: NuttX defconfig SDIO flag
    defconfig = board_path / 'nuttx-config/nsh/defconfig'
    if defconfig.exists():
        text = defconfig.read_text(errors='ignore')
        if 'CONFIG_MMCSD_SDIO=y' in text:
            result['has_sd_card'] = True

    # default.px4board: SD tools, Ethernet, heater
    px4board = board_path / 'default.px4board'
    if px4board.exists():
        text = px4board.read_text(errors='ignore')
        if not result['has_sd_card'] and (
                'CONFIG_SYSTEMCMDS_SD_BENCH=y' in text
                or 'CONFIG_SYSTEMCMDS_SD_STRESS=y' in text):
            result['has_sd_card'] = True
        if 'CONFIG_BOARD_ETHERNET=y' in text:
            result['has_ethernet'] = True
        if 'CONFIG_DRIVERS_HEATER=y' in text:
            result['has_heater'] = True

    return result


# ---------------------------------------------------------------------------
# Sensor driver config parser
# ---------------------------------------------------------------------------

# Human-readable names for known sensor driver key suffixes.
# Key = last underscore-separated segment of CONFIG_DRIVERS_<CAT>_... (uppercased).
# Value = display name as used in PX4 documentation.
_SENSOR_CHIP_NAMES = {
    # IMU — InvenSense
    'ICM20602':  'ICM-20602',
    'ICM20649':  'ICM-20649',
    'ICM20689':  'ICM-20689',
    'ICM20948':  'ICM-20948',
    'ICM42670P': 'ICM-42670P',
    'ICM42688P': 'ICM-42688P',
    'ICM45686':  'ICM-45686',
    'IIM42652':  'IIM-42652',
    'MPU6000':   'MPU-6000',
    'MPU9250':   'MPU-9250',
    # IMU — Bosch
    'BMI055':    'BMI055',
    'BMI088':    'BMI088',
    'BMI270':    'BMI270',
    # IMU — Analog Devices
    'ADIS16470': 'ADIS16470',
    'ADIS16507': 'ADIS16507',
    # Barometer — InvenSense
    'ICP101XX':  'ICP-10100',
    'ICP201XX':  'ICP-20100',
    # Barometer — Bosch
    'BMP280':    'BMP280',
    'BMP388':    'BMP388',
    'BMP581':    'BMP581',
    # Barometer — others
    'DPS310':    'DPS310',
    'SPA06':     'SPA06',
    'SPL06':     'SPL06',
    'MS5611':    'MS5611',
    'LPS25H':    'LPS25H',
    'MPC2520':   'MPC2520',
    'MPL3115A2': 'MPL3115A2',
    # Magnetometer — Bosch
    'BMM150':    'BMM150',
    'BMM350':    'BMM350',
    # Magnetometer — AKM
    'AK09916':   'AK09916',
    'AK8963':    'AK8963',
    # Magnetometer — Isentek
    'IST8308':   'IST8308',
    'IST8310':   'IST8310',
    # Magnetometer — others
    'HMC5883':   'HMC5883L',
    'LIS2MDL':   'LIS2MDL',
    'LIS3MDL':   'LIS3MDL',
    'LSM303AGR': 'LSM303AGR',
    'RM3100':    'RM3100',
    'QMC5883L':  'QMC5883L',
    'QMC5883P':  'QMC5883P',
    'MMC5983MA': 'MMC5983MA',
    'VCM1193L':  'VCM1193L',
    'IIS2MDC':   'IIS2MDC',
    # OSD chips
    'ATXXXX':    'AT7456E',
    # MSP_OSD is a protocol adapter, not a chip — intentionally omitted
    # Power monitors
    'INA226':    'INA226',
    'INA228':    'INA228',
    'INA238':    'INA238',
}

# CONFIG_DRIVERS_<CATEGORY> prefix → sensor type key
_DRIVER_CATEGORY_MAP = {
    'IMU':           'imu',
    'BAROMETER':     'baro',
    'MAGNETOMETER':  'mag',
    'OSD':           'osd',
}

# Chip key (uppercase, matching _SENSOR_CHIP_NAMES keys) → sensor category.
# Used by parse_rc_board_sensors() to classify driver names found in rc.board_sensors.
_CHIP_TO_CATEGORY: dict[str, str] = {
    # IMU
    'ICM20602': 'imu', 'ICM20649': 'imu', 'ICM20689': 'imu', 'ICM20948': 'imu',
    'ICM42670P': 'imu', 'ICM42688P': 'imu', 'ICM45686': 'imu', 'IIM42652': 'imu',
    'MPU6000': 'imu', 'MPU9250': 'imu',
    'BMI055': 'imu', 'BMI088': 'imu', 'BMI270': 'imu',
    'ADIS16470': 'imu', 'ADIS16507': 'imu',
    # Barometer
    'ICP101XX': 'baro', 'ICP201XX': 'baro',
    'BMP280': 'baro', 'BMP388': 'baro', 'BMP581': 'baro',
    'DPS310': 'baro', 'SPA06': 'baro', 'SPL06': 'baro',
    'MS5611': 'baro', 'LPS25H': 'baro', 'MPC2520': 'baro', 'MPL3115A2': 'baro',
    # Magnetometer
    'BMM150': 'mag', 'BMM350': 'mag',
    'AK09916': 'mag', 'AK8963': 'mag',
    'IST8308': 'mag', 'IST8310': 'mag',
    'HMC5883': 'mag', 'LIS2MDL': 'mag', 'LIS3MDL': 'mag',
    'LSM303AGR': 'mag', 'RM3100': 'mag',
    'QMC5883L': 'mag', 'QMC5883P': 'mag',
    'MMC5983MA': 'mag', 'VCM1193L': 'mag', 'IIS2MDC': 'mag',
    # OSD
    'ATXXXX': 'osd',
    # Power monitors
    'INA226': 'power_monitor',
    'INA228': 'power_monitor',
    'INA238': 'power_monitor',
}


def _driver_key_to_chip_name(suffix: str) -> str | None:
    """Convert a driver key suffix to a human-readable chip name.

    E.g. 'INVENSENSE_ICM42688P' → 'ICM-42688P'
         'BMP388'               → 'BMP388'
         'MSP_OSD'              → None  (skip)
    """
    # Last underscore-separated segment is the chip identifier
    chip_key = suffix.split('_')[-1].upper()
    return _SENSOR_CHIP_NAMES.get(chip_key)


def parse_sensor_config(board_path: Path) -> dict:
    """Parse enabled sensor drivers from default.px4board.

    Reads ``CONFIG_DRIVERS_<CATEGORY>_[<VENDOR>_]<CHIP>=y`` entries and maps
    them to human-readable chip names grouped by sensor type.

    Returns
    -------
    {
        'imu':  list[str],   # e.g. ['ICM-42688P']
        'baro': list[str],   # e.g. ['BMP388', 'ICP-20100']
        'mag':  list[str],   # e.g. ['IST8310']
        'osd':  list[str],   # e.g. ['AT7456E']
    }
    All lists are empty when no drivers of that type are enabled.
    """
    result: dict[str, list] = {k: [] for k in _DRIVER_CATEGORY_MAP.values()}

    px4board = board_path / 'default.px4board'
    if not px4board.exists():
        return result

    text = px4board.read_text(errors='ignore')
    for line in text.splitlines():
        m = re.match(r'CONFIG_DRIVERS_([A-Z0-9_]+)=y', line.strip())
        if not m:
            continue
        full_suffix = m.group(1)  # e.g. "IMU_INVENSENSE_ICM42688P"

        # Identify category (first segment)
        for cat_prefix, type_key in _DRIVER_CATEGORY_MAP.items():
            if full_suffix.startswith(cat_prefix + '_') or full_suffix == cat_prefix:
                after_cat = full_suffix[len(cat_prefix):].lstrip('_')  # "INVENSENSE_ICM42688P"
                name = _driver_key_to_chip_name(after_cat)
                if name and name not in result[type_key]:
                    result[type_key].append(name)
                break

    return result


def parse_rc_board_sensors(board_path: Path) -> dict:
    """Parse sensor driver start commands from init/rc.board_sensors.

    Complements parse_sensor_config(): many boards use CONFIG_COMMON_BAROMETERS=y /
    CONFIG_COMMON_MAGNETOMETER=y in default.px4board (enabling all drivers of that
    class) rather than per-chip entries, so the specific chip is only determinable
    from which driver is actually started at runtime.

    Returns the same structure as parse_sensor_config():
        {'imu': list[str], 'baro': list[str], 'mag': list[str], 'osd': list[str]}
    All lists are empty when the file is absent or no recognised drivers are found.
    """
    result: dict[str, list] = {k: [] for k in _DRIVER_CATEGORY_MAP.values()}

    rc_sensors = board_path / 'init' / 'rc.board_sensors'
    if not rc_sensors.exists():
        return result

    text = rc_sensors.read_text(errors='ignore')
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#') or line.startswith('//'):
            continue
        # First word on the line is the driver executable name
        m = re.match(r'^([a-z][a-z0-9_]+)', line)
        if not m:
            continue
        chip_key = m.group(1).upper()  # e.g. 'ms5611' → 'MS5611'
        category = _CHIP_TO_CATEGORY.get(chip_key)
        display_name = _SENSOR_CHIP_NAMES.get(chip_key)
        if category and display_name and category in result and display_name not in result[category]:
            result[category].append(display_name)

    return result


def _parse_sensor_line(line: str) -> dict | None:
    """Parse a single sensor start command line and return a bus-info dict, or None.

    Extracts chip display name, category, bus type (SPI/I2C), bus number, and external flag
    from a line like ``icm42688p -s -R 4 start`` or ``ist8310 -X -b 1 -R 10 start``.
    """
    m = re.match(r'^([a-z][a-z0-9_]+)', line)
    if not m:
        return None
    chip_key = m.group(1).upper()
    category = _CHIP_TO_CATEGORY.get(chip_key)
    display_name = _SENSOR_CHIP_NAMES.get(chip_key)
    if not category or not display_name:
        return None

    is_spi = bool(re.search(r'\s-s\b', line))
    is_i2c = bool(re.search(r'\s-I\b', line))
    is_external = bool(re.search(r'\s-X\b', line))
    # -X without -s is external I2C (PX4 convention: external SPI uses -s -b N)
    if is_spi:
        bus_type = 'SPI'
    elif is_i2c or (is_external and not is_spi):
        bus_type = 'I2C'
    else:
        bus_type = None
    bus_num_m = re.search(r'\s-b\s+(\d+)', line)
    bus_num = int(bus_num_m.group(1)) if bus_num_m else None

    return {
        'name': display_name,
        'category': category,
        'bus_type': bus_type,
        'bus_num': bus_num,
        'external': is_external,
    }


def _extract_port_label_from_comment(comment: str | None) -> str | None:
    """Extract a port label from a comment preceding a sensor start command.

    Looks for GPS port labels first (GPS1, GPS2), then TELEM labels (TELEM1),
    then numbered I2C port labels (I2C1, I2C2). Returns the first match, or None.

    Examples:
        "External compass on GPS1/I2C1: ..."  → "GPS1"
        "External sensors on I2C1"             → "I2C1"
        "Internal magnetometer on I2C"         → None  (no digit suffix)
    """
    if not comment:
        return None
    m = re.search(r'\bGPS(\d+)\b', comment, re.IGNORECASE)
    if m:
        return f'GPS{m.group(1)}'
    m = re.search(r'\bTELEM(\d+)\b', comment, re.IGNORECASE)
    if m:
        return f'TELEM{m.group(1)}'
    # Numbered I2C port (I2C1, I2C2, ...) — a digit suffix distinguishes port name from generic "on I2C"
    m = re.search(r'\bI2C(\d+)\b', comment, re.IGNORECASE)
    if m:
        return f'I2C{m.group(1)}'
    return None


def parse_rc_board_sensor_bus(board_path: Path) -> dict:
    """Parse sensor bus type and number from init/rc.board_sensors.

    Only processes unconditional sensor commands — lines inside
    ``if ver hwtypecmp`` variant blocks are skipped (see parse_sensor_variant_blocks
    for per-variant sensor data).

    Returns enriched per-instance entries, keeping separate entries for
    the same chip on different buses (e.g. internal vs external IST8310):
        {
          'imu':  [{'name': ..., 'bus_type': 'SPI'|'I2C'|None,
                    'bus_num': int|None, 'external': bool,
                    'port_label': str|None}],
          'baro': [...],
          'mag':  [...],
          'osd':  [],
        }
    port_label is auto-detected from the comment immediately preceding the
    sensor start command (e.g. "# External compass on GPS1/I2C1:" → "GPS1").
    power_monitor entries (INA226/INA228/INA238) are included in a
    ``power_monitor`` category alongside the standard sensor categories.
    """
    result: dict[str, list] = {k: [] for k in _DRIVER_CATEGORY_MAP.values()}
    result['power_monitor'] = []
    seen: set = set()

    rc_sensors = board_path / 'init' / 'rc.board_sensors'
    if not rc_sensors.exists():
        return result

    variant_depth = 0   # depth counter for hwtypecmp if-blocks (lines inside are skipped)
    other_depth = 0     # depth of other if-blocks (still counted to match fi correctly)
    last_comment: str | None = None

    text = rc_sensors.read_text(errors='ignore')
    for line in text.splitlines():
        line = line.strip()
        if not line:
            last_comment = None
            continue
        if line.startswith('#') or line.startswith('//'):
            last_comment = line.lstrip('#/').strip()
            continue

        # Track if/fi structure to skip variant-specific blocks
        if re.match(r'^if\s+ver\s+hwtypecmp\b', line):
            variant_depth += 1
            last_comment = None
            continue
        if re.match(r'^if\b', line):
            if variant_depth > 0:
                other_depth += 1   # nested block inside a variant block
            last_comment = None
            continue
        if line == 'fi':
            if other_depth > 0:
                other_depth -= 1
            elif variant_depth > 0:
                variant_depth -= 1
            last_comment = None
            continue
        if line in ('then', 'else', 'do'):
            last_comment = None
            continue

        if variant_depth > 0:
            last_comment = None
            continue   # skip all lines inside hwtypecmp blocks

        entry = _parse_sensor_line(line)
        port_label = _extract_port_label_from_comment(last_comment)
        last_comment = None
        if entry is None:
            continue

        key = (entry['name'], entry['bus_type'], entry['bus_num'], entry['external'])
        if key not in seen:
            seen.add(key)
            result[entry['category']].append({
                'name': entry['name'],
                'bus_type': entry['bus_type'],
                'bus_num': entry['bus_num'],
                'external': entry['external'],
                'port_label': port_label,
            })

    return result


def parse_sensor_variant_blocks(board_path: Path) -> dict:
    """Parse hardware variant blocks from init/rc.board_sensors.

    Detects ``if ver hwtypecmp CODE(s)`` conditional blocks and extracts
    per-variant and unconditional sensor entries.

    Returns:
        {
          'has_variants': bool,
          'unconditional': {
            'imu':  [{'name': ..., 'bus_type': ..., 'bus_num': ..., 'external': ...}],
            'baro': [...], 'mag': [...], 'osd': [],
          },
          'variants': {
            'VD000001': {'imu': [...], 'baro': [...], 'mag': [...], 'osd': []},
            '__other__': {...},   # else branch (present only when an else clause exists)
          }
        }

    ``else`` blocks are stored under the key ``'__other__'``.

    **Graceful-fail inference:** If the same sensor name appears in both the
    unconditional block and at least one variant block, it is assumed to be a
    PX4 graceful-fail start (the driver fails silently on hardware where the chip is
    absent). Such sensors are removed from ``unconditional`` and kept only in the
    variant dict(s) where they were explicitly listed.
    """
    def _empty_cat_dict() -> dict:
        d = {k: [] for k in _DRIVER_CATEGORY_MAP.values()}
        d['power_monitor'] = []
        return d

    unconditional = _empty_cat_dict()
    variants: dict[str, dict] = {}
    seen_unconditional: set = set()
    seen_variant: dict[str, set] = {}

    rc_sensors = board_path / 'init' / 'rc.board_sensors'
    if not rc_sensors.exists():
        return {'has_variants': False, 'unconditional': unconditional, 'variants': {}}

    # context_stack entries: {'type': 'hwtypecmp'|'other', 'codes': [...], 'in_else': bool}
    context_stack: list[dict] = []

    last_comment: str | None = None
    text = rc_sensors.read_text(errors='ignore')
    for line in text.splitlines():
        line = line.strip()
        if not line:
            last_comment = None
            continue
        if line.startswith('#') or line.startswith('//'):
            last_comment = line.lstrip('#/').strip()
            continue

        # Shell control flow
        m_htc = re.match(r'^if\s+ver\s+hwtypecmp\s+(.+)', line)
        if m_htc:
            codes = m_htc.group(1).split()
            context_stack.append({'type': 'hwtypecmp', 'codes': codes, 'in_else': False})
            last_comment = None
            continue
        if re.match(r'^if\b', line):
            context_stack.append({'type': 'other', 'codes': [], 'in_else': False})
            last_comment = None
            continue
        if line == 'else':
            if context_stack and context_stack[-1]['type'] == 'hwtypecmp':
                context_stack[-1]['in_else'] = True
            last_comment = None
            continue
        if line == 'fi':
            if context_stack:
                context_stack.pop()
            last_comment = None
            continue
        if line in ('then', 'do'):
            last_comment = None
            continue

        entry = _parse_sensor_line(line)
        port_label = _extract_port_label_from_comment(last_comment)
        last_comment = None
        if entry is None:
            continue

        # Find innermost hwtypecmp context
        htc_ctx = next(
            (c for c in reversed(context_stack) if c['type'] == 'hwtypecmp'), None
        )

        if htc_ctx is None:
            # Unconditional
            key = (entry['name'], entry['bus_type'], entry['bus_num'], entry['external'])
            if key not in seen_unconditional:
                seen_unconditional.add(key)
                unconditional[entry['category']].append({
                    'name': entry['name'],
                    'bus_type': entry['bus_type'],
                    'bus_num': entry['bus_num'],
                    'external': entry['external'],
                    'port_label': port_label,
                })
        elif htc_ctx['in_else']:
            # else branch → store under __other__
            target_keys = ['__other__']
            for vk in target_keys:
                if vk not in variants:
                    variants[vk] = _empty_cat_dict()
                    seen_variant[vk] = set()
                key = (entry['name'], entry['bus_type'], entry['bus_num'], entry['external'])
                if key not in seen_variant[vk]:
                    seen_variant[vk].add(key)
                    variants[vk][entry['category']].append({
                        'name': entry['name'],
                        'bus_type': entry['bus_type'],
                        'bus_num': entry['bus_num'],
                        'external': entry['external'],
                        'port_label': port_label,
                    })
        else:
            # Inside a specific variant block — add to each matching code
            for code in htc_ctx['codes']:
                if code not in variants:
                    variants[code] = _empty_cat_dict()
                    seen_variant[code] = set()
                key = (entry['name'], entry['bus_type'], entry['bus_num'], entry['external'])
                if key not in seen_variant[code]:
                    seen_variant[code].add(key)
                    variants[code][entry['category']].append({
                        'name': entry['name'],
                        'bus_type': entry['bus_type'],
                        'bus_num': entry['bus_num'],
                        'external': entry['external'],
                        'port_label': port_label,
                    })

    has_variants = bool(variants)

    # Graceful-fail inference: if a sensor name appears in both the unconditional block
    # AND at least one variant block, the unconditional start is a PX4 graceful-fail
    # pattern (the driver fails silently on hardware where the chip is absent).
    # Move such sensors out of unconditional so they are annotated per-variant instead.
    if has_variants:
        variant_sensor_names: set = set()
        for vdata in variants.values():
            for cat_list in vdata.values():
                for e in cat_list:
                    variant_sensor_names.add(e['name'])
        for cat in unconditional:
            unconditional[cat] = [
                e for e in unconditional[cat]
                if e['name'] not in variant_sensor_names
            ]

    return {
        'has_variants': has_variants,
        'unconditional': unconditional,
        'variants': variants,
    }


def parse_interface_config(board_path: Path) -> dict:
    """Detect I2C, SPI, CAN bus counts and USB presence from board source files.

    Sources
    -------
    - ``nuttx-config/nsh/defconfig`` : I2C and SPI bus enables
    - ``src/board_config.h``         : CAN GPIO defines; USB VBUS GPIO

    Returns
    -------
    {
        'num_i2c_buses': int,
        'num_spi_buses': int,
        'num_can_buses': int,
        'has_usb': bool,
    }
    """
    defconfig = board_path / 'nuttx-config' / 'nsh' / 'defconfig'
    board_cfg = board_path / 'src' / 'board_config.h'
    text_def = defconfig.read_text(errors='ignore') if defconfig.exists() else ''
    text_cfg = board_cfg.read_text(errors='ignore') if board_cfg.exists() else ''

    # I2C: CONFIG_STM32H7_I2C1=y, CONFIG_STM32_I2C1=y, CONFIG_IMXRT_LPI2C1=y, etc.
    i2c = set(re.findall(
        r'CONFIG_(?:STM32[A-Z0-9]*|IMXRT)_(?:LP)?I2C(\d+)=y', text_def))
    # SPI: CONFIG_STM32H7_SPI1=y, CONFIG_STM32_SPI1=y, CONFIG_IMXRT_LPSPI2=y, etc.
    # The base enable has no suffix after the number; _DMA / _DMA_BUFFER entries are ignored.
    spi = set(re.findall(
        r'CONFIG_(?:STM32[A-Z0-9]*|IMXRT)_(?:LP)?SPI(\d+)=y', text_def))
    # CAN: GPIO_CAN1_TX or GPIO_CAN1_SILENT_S0 in board_config.h
    can = set(re.findall(r'GPIO_CAN(\d+)_(?:TX|SILENT)', text_cfg))
    # USB: OTGFS / OTGHS in defconfig, or VBUS GPIO in board_config.h
    has_usb = bool(
        re.search(r'CONFIG_(?:STM32[A-Z0-9]+_OTG(?:FS|HS)|IMXRT_USBOTG)=y', text_def)
        or 'GPIO_OTGFS_VBUS' in text_cfg
        or 'GPIO_OTGHS_VBUS' in text_cfg
    )

    return {
        'num_i2c_buses': len(i2c),
        'num_spi_buses': len(spi),
        'num_can_buses': len(can),
        'has_usb': has_usb,
    }


def parse_i2c_bus_config(board_path: Path) -> dict:
    """Parse authoritative I2C bus routing from src/i2c.cpp.

    Reads ``initI2CBusExternal(N)`` and ``initI2CBusInternal(N)`` entries to
    determine which I2C buses go to external connectors (user-accessible) and
    which are wired only to on-board chips (not externally accessible).

    Returns
    -------
    {
        'external': [1, 2, 4],   # bus numbers routed to external connectors
        'internal': [3],          # bus numbers wired only to on-board chips
    }
    Returns an empty dict ``{}`` when ``src/i2c.cpp`` is absent or contains no
    recognised entries.
    """
    i2c_cpp = board_path / 'src' / 'i2c.cpp'
    if not i2c_cpp.exists():
        return {}

    text = i2c_cpp.read_text(errors='ignore')
    external = sorted(
        int(m) for m in re.findall(r'initI2CBusExternal\s*\(\s*(\d+)\s*\)', text)
    )
    internal = sorted(
        int(m) for m in re.findall(r'initI2CBusInternal\s*\(\s*(\d+)\s*\)', text)
    )
    if not external and not internal:
        return {}
    return {'external': external, 'internal': internal}


def compute_groups(timers: list, channels: list) -> list:
    """Group PWM output channels by their shared hardware timer.

    Mechanism
    ---------
    Inputs come from parse_timer_config().

    Each timer in io_timers[] represents one PWM group: all channels sharing
    that timer must run at the same frequency and use the same output protocol
    (PWM, DShot, etc.) because they share the timer's prescaler and period
    registers.

    Matching channels to timers:
    - STM32: channel['timer'] == timer name exactly (e.g. both are 'Timer5')
    - iMXRT: channels use 'PWM1_PWM_A', 'PWM1_PWM_B', …; matched by prefix
      (startswith('PWM1_') or startswith('PWM1P')).

    DShot support is determined at two levels:
    - STM32: timer-level — if the initIOTimer() call included a DMA argument,
      the whole group supports DShot.
    - iMXRT: per-channel — initIOTimerChannelDshot() marks individual channels;
      the group's dshot_outputs / non_dshot_outputs lists reflect this split.

    Returns a list of group dicts, one per timer, each containing:
      group            int  — 1-based group number
      timer            str  — hardware timer name
      outputs          list — 1-based output indices in this group
      dshot            bool — True if any output in the group supports DShot
      dshot_outputs    list — outputs that support DShot
      non_dshot_outputs list — outputs that do not support DShot
    """
    groups = []
    for i, timer in enumerate(timers):
        tname = timer["timer"]
        # Match channels either by exact timer name or by prefix (iMXRT: "PWM1" matches "PWM1_PWM_A")
        group_channels = [
            ch for ch in channels
            if ch["timer"] == tname or ch["timer"].startswith(tname + "_") or ch["timer"].startswith(tname + "P")
        ]
        group_outputs = [ch["output_index"] for ch in group_channels]
        if group_outputs:
            # DShot: for iMXRT it's per-channel; for STM it's per-timer
            # iMXRT channels have is_dshot_channel=True on at least some channels
            has_per_channel_dshot = any(ch.get("is_dshot_channel") for ch in group_channels)
            if has_per_channel_dshot or (not timer["dshot"] and any(ch.get("is_dshot_channel") is not None for ch in group_channels)):
                # iMXRT: use per-channel flag
                dshot_outputs = [ch["output_index"] for ch in group_channels if ch.get("is_dshot_channel")]
                non_dshot_outputs = [ch["output_index"] for ch in group_channels if not ch.get("is_dshot_channel")]
            else:
                # STM: timer-level DShot
                dshot_outputs = [ch["output_index"] for ch in group_channels] if timer["dshot"] else []
                non_dshot_outputs = [] if timer["dshot"] else [ch["output_index"] for ch in group_channels]
            any_dshot = bool(dshot_outputs)
            groups.append({
                "group": i + 1,
                "timer": tname,
                "outputs": group_outputs,
                "dshot": any_dshot,
                "dshot_outputs": dshot_outputs,
                "non_dshot_outputs": non_dshot_outputs,
            })
    return groups


def compute_bdshot(groups: list, channels: list, family: str) -> list:
    """Determine Bidirectional DShot (BDShot) capability per output channel.

    Mechanism
    ---------
    BDShot requires the timer channel to support DMA *capture* (reading eRPM
    telemetry back from the ESC) in addition to the DMA *output* used for
    regular DShot.  Not all timer channels have capture DMA on every chip.

    The capability map ``BDSHOT_CAPTURE_MAP`` encodes which timer channels
    (0-based index within a timer: CH1=0, CH2=1, …) support capture DMA on
    each chip family.  This map was derived from the PX4 hardware abstraction
    layer function ``getTimerChannelDMAMap()`` in
    ``platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.cpp`` (and the
    equivalent for other families).

    Three cases by family:
      '_none': True  — no capture DMA at all on this family (STM32F4/F1);
                       BDShot is entirely unsupported.
      '_all': True   — all DShot outputs support BDShot (iMXRT); the
                       hardware guarantees capture for every DShot channel.
      per-timer dict — STM32H7/F7: each timer lists the 0-based channel
                       indices that have capture DMA.  Channels not listed
                       can still *output* BDShot but cannot *capture* eRPM
                       telemetry (recorded as 'bdshot_output_only').

    For each group the function sets:
      bdshot              bool — True if any output has any BDShot capability
      bdshot_outputs      list — outputs with full BDShot (output + capture)
      bdshot_output_only  list — outputs with output-only BDShot (no eRPM)

    The channel_index values from parse_timer_config() are used to look up
    each output in the capture DMA map.
    """
    cap_map = BDSHOT_CAPTURE_MAP.get(family, {})
    all_capable = cap_map.get("_all", False)
    none_capable = cap_map.get("_none", False)

    result = []
    for group in groups:
        timer = group["timer"]
        if not group["dshot"] or none_capable:
            group["bdshot"] = False
            group["bdshot_outputs"] = []
            group["bdshot_output_only"] = []
        elif all_capable:
            # For all-capable (iMXRT), BDShot is limited to DShot-capable channels
            dshot_outs = group.get("dshot_outputs", group["outputs"] if group["dshot"] else [])
            group["bdshot"] = bool(dshot_outs)
            group["bdshot_outputs"] = dshot_outs[:]
            group["bdshot_output_only"] = []
        elif timer in cap_map:
            supported_ch_indices = cap_map[timer]
            # Find which outputs of this timer have capture-capable channels
            bdshot_full = []
            bdshot_out_only = []
            for ch in channels:
                if ch["timer"] == timer and ch["output_index"] in group["outputs"]:
                    idx = ch["channel_index"]
                    if idx is not None and idx in supported_ch_indices:
                        bdshot_full.append(ch["output_index"])
                    else:
                        bdshot_out_only.append(ch["output_index"])
            group["bdshot"] = True if (bdshot_full or bdshot_out_only) else False
            group["bdshot_outputs"] = bdshot_full
            group["bdshot_output_only"] = bdshot_out_only
        else:
            # Timer has DMA (DShot) but no channel DMA maps → BDShot output only (no capture)
            group["bdshot"] = True
            group["bdshot_outputs"] = []
            group["bdshot_output_only"] = group["outputs"][:]
        result.append(group)
    return result


# ---------------------------------------------------------------------------
# Doc matching: map board vendor/name to flight controller doc file
# ---------------------------------------------------------------------------
BOARD_TO_DOC = {
    # px4 reference designs
    "px4/fmu-v2":   "pixhawk.md",
    "px4/fmu-v3":   "pixhawk-2.md",
    "px4/fmu-v4":   "pixracer.md",
    "px4/fmu-v4pro": "pixhawk3_pro.md",
    "px4/fmu-v5":   "pixhawk4.md",
    "px4/fmu-v5x":  "pixhawk5x.md",
    "px4/fmu-v6c":  "pixhawk6c.md",
    "px4/fmu-v6u":  "pixhawk6x_pro.md",
    "px4/fmu-v6x":  "pixhawk6x.md",
    "px4/fmu-v6xrt": "pixhawk6x-rt.md",
    # holybro
    "holybro/kakutef7":       "kakutef7.md",
    "holybro/kakuteh7":       "kakuteh7.md",
    "holybro/kakuteh7v2":     "kakuteh7v2.md",
    "holybro/kakuteh7mini":   "kakuteh7mini.md",
    "holybro/kakuteh7dualimu": "kakuteh7v2.md",  # Same board
    "holybro/kakuteh7-wing":  "kakuteh7-wing.md",
    "holybro/durandal-v1":   "durandal.md",
    "holybro/pix32v5":       "holybro_pix32_v5.md",
    # cuav
    "cuav/fmu-v6x":  "cuav_pixhawk_v6x.md",
    "cuav/nora":     "cuav_nora.md",
    "cuav/x7pro":    "cuav_x7.md",
    "cuav/x25-evo":  "cuav_x25-evo.md",
    "cuav/x25-super":"cuav_x25-super.md",
    # cubepilot
    "cubepilot/cubeorange":     "cubepilot_cube_orange.md",
    "cubepilot/cubeorangeplus": "cubepilot_cube_orangeplus.md",
    "cubepilot/cubeyellow":     "cubepilot_cube_yellow.md",
    # mro
    "mro/x21":              "mro_x2.1.md",
    "mro/x21-777":          "mro_x2.1.md",
    "mro/ctrl-zero-f7":     "mro_control_zero_f7.md",
    "mro/ctrl-zero-f7-oem": "mro_control_zero_f7.md",
    "mro/pixracerpro":      "pixracer.md",
    # ark
    "ark/fmu-v6x": "ark_v6x.md",
    "ark/fpv":     "ark_fpv.md",
    "ark/pi6x":    "ark_pi6x.md",
    # nxp
    "nxp/fmuk66-v3":       "nxp_rddrone_fmuk66.md",
    "nxp/fmuk66-e":        "nxp_rddrone_fmuk66.md",
    "nxp/mr-tropic":       "nxp_mr_vmu_rt1176.md",
    "nxp/tropic-community": "nxp_mr_vmu_rt1176.md",
    # others
    "omnibus/f4sd":       "omnibus_f4_sd.md",
    "airmind/mindpx-v2":  "mindpx.md",
    "spracing/h7extreme": "spracingh7extreme.md",
    "thepeach/k1":        "thepeach_k1.md",
    "thepeach/r1":        "thepeach_r1.md",
    "radiolink/PIX6":     "radiolink_pix6.md",
    "corvon/743v1":       "corvon_743v1.md",
    "gearup/airbrainh743": "gearup_airbrainh743.md",
    "svehicle/e2":        "svehicle_e2.md",
    "modalai/fc-v1":      "modalai_fc_v1.md",
    "micoair/h743-lite":  "micoair743-lite.md",
    "narinfc/h7":         "vololand_narinfc_h7.md",
    "accton-godwit/ga1":  "accton-godwit_ga1.md",
    "3dr/ctrl-zero-h7-oem-revg": None,
    "auterion/fmu-v6s":   None,
    "auterion/fmu-v6x":   None,
    "av/x-v1":            None,
    "bitcraze/crazyflie": None,
    "bitcraze/crazyflie21": None,
    "cuav/7-nano":        None,
    "diatone/mamba-f405-mk2": None,
    "espressif/esp32":    None,
    "flywoo/gn-f405":     None,
    "hkust/nxt-v1":       None,
    "hkust/nxt-dual":     None,
    "matek/gnss-m9n-f4":  None,
    "matek/h743":         None,
    "matek/h743-mini":    None,
    "matek/h743-slim":    None,
    "micoair/h743":       None,
    "micoair/h743-aio":   None,
    "micoair/h743-v2":    None,
    "modalai/fc-v2":      None,
    "modalai/voxl2-io":   None,
    "mro/ctrl-zero-classic": None,
    "mro/ctrl-zero-h7":   None,
    "mro/ctrl-zero-h7-oem": None,
    "nxp/mr-canhubk3":    None,
    "nxp/ucans32k146":    None,
    "raspberrypi/pico":   None,
    "siyi/n7":            None,
    "sky-drones/smartap-airlink": "airlink.md",
    "uvify/core":         None,
    "x-mav/ap-h743r1":   "x-mav_ap-h743r1.md",
    "x-mav/ap-h743v2":   None,
    "xc-fly/xc-slam":    None,
    "xc-fly/xc-slim":    None,
    "zeroone/x6":         None,
    "ark/cannode":        None,
    "atl/mantis-edu":     None,
}

# Doc URL base
DOC_URL_BASE = "https://docs.px4.io/main/en/flight_controller/"

# Mapping from doc filename to URL slug
def doc_to_url(doc_filename: str) -> str:
    if not doc_filename:
        return None
    slug = doc_filename.replace(".md", "")
    return DOC_URL_BASE + slug


# ---------------------------------------------------------------------------
# Parse flight controller docs for existing PWM Output sections
# ---------------------------------------------------------------------------
PWM_SECTION_PATTERNS = [
    r'##\s+PWM\s+Output',
    r'##\s+PWM\s+Outputs',
    r'##\s+Outputs',
    r'##\s+Motor\s+Outputs',
    r'##\s+Output\s+Ports',
    r'##\s+Actuator\s+Outputs',
    r'##\s+PWM\s+Output\s+&',
    r'##\s+DShot',
    r'##\s+Timer\s+',
]

def parse_fc_doc(doc_path: Path) -> dict:
    """Parse a FC doc and find PWM/output section."""
    if not doc_path.exists():
        return {"exists": False, "has_pwm_section": False, "pwm_section": None}

    text = doc_path.read_text(errors="ignore")
    result = {"exists": True, "has_pwm_section": False, "pwm_section": None}

    # Search for PWM-related headings
    for pattern in PWM_SECTION_PATTERNS:
        m = re.search(pattern, text, re.IGNORECASE)
        if m:
            result["has_pwm_section"] = True
            # Extract the section content (until next ## heading)
            start = m.start()
            rest = text[start:]
            # Find end of this section
            end_m = re.search(r'\n##\s+', rest[3:])
            if end_m:
                section = rest[:end_m.start() + 3]
            else:
                section = rest[:2000]  # Cap at 2000 chars
            result["pwm_section"] = section.strip()
            result["pwm_section_heading"] = m.group(0).strip()
            break

    return result


_MFR_URL_RE = re.compile(r'Contact the \[.*?\]\((https?://[^)]+)\)', re.IGNORECASE)

# Static table of known manufacturer contact URLs, keyed by the manufacturer
# name slug (lowercase, separators stripped).  Checked first so the common
# case requires no filesystem access.  The filesystem scan below acts as a
# fallback for manufacturers added after this table was last updated.
# To update: run grep -rh "Contact the" docs/en/flight_controller/*.md
MANUFACTURER_URLS: dict[str, str] = {
    'ark':              'https://arkelectron.com/contact-us/',
    'arkelectron':      'https://arkelectron.com/contact-us/',
    'arkelectronics':   'https://arkelectron.com/contact-us/',
    'beagleboard':   'https://www.beagleboard.org/boards/beaglebone-blue',
    'cubepilot':     'https://cubepilot.org/#/home',
    'cuav':          'https://store.cuav.net/',
    'emlid':         'https://emlid.com/',
    'gearup':        'https://takeyourgear.com/',
    'holybro':       'https://holybro.com/',
    'micoair':       'https://micoair.com/',
    'mindpx':        'http://mindpx.net',
    'modalai':       'https://forum.modalai.com/',
    'mro':           'https://store.mrobotics.io/',
    'mrobotics':     'https://store.mrobotics.io/',
    'nxp':           'https://www.nxp.com',
    'raccoonlab':    'https://raccoonlab.co',
    'radiolink':     'https://radiolink.com.cn/',
    'skydrones':     'https://sky-drones.com/',
    'spracing':      'https://shop.seriouslypro.com',
    'seriouslypro':  'https://shop.seriouslypro.com',
    'thepeach':      'https://thepeach.kr/',
}


def _find_manufacturer_url(manufacturer: str) -> str | None:
    """Return the manufacturer contact URL, or None if unknown.

    Checks MANUFACTURER_URLS first (O(1)).  Falls back to scanning existing
    FC docs for new manufacturers not yet in the table.
    """
    slug = _make_slug(manufacturer).replace('_', '')   # "holybro", "cuav", …
    if slug in MANUFACTURER_URLS:
        return MANUFACTURER_URLS[slug]
    # Fallback: scan docs whose filenames start with the manufacturer slug
    matches = []
    for doc_path in FC_DOCS.glob('*.md'):
        if not doc_path.stem.replace('_', '').replace('-', '').startswith(slug):
            continue
        m = _MFR_URL_RE.search(doc_path.read_text(errors='ignore'))
        if m:
            matches.append(m.group(1))
    return max(set(matches), key=matches.count) if matches else None


# ---------------------------------------------------------------------------
# Generate proposed PWM Output section text
# ---------------------------------------------------------------------------
def make_outputs_range(outputs: list) -> str:
    """Format a list of ints as compact ranges, e.g. [1,2,3,5] -> '1-3, 5'."""
    if not outputs:
        return ""
    outputs = sorted(outputs)
    ranges = []
    start = outputs[0]
    end = outputs[0]
    for o in outputs[1:]:
        if o == end + 1:
            end = o
        else:
            ranges.append(f"{start}" if start == end else f"{start}-{end}")
            start = end = o
    ranges.append(f"{start}" if start == end else f"{start}-{end}")
    return ", ".join(ranges)


def generate_pwm_section(board_key: str, entry: dict) -> str:
    """Generate a proposed '## PWM Outputs' markdown section from a flat JSON entry."""
    groups = entry.get("groups", [])
    has_io = entry.get("has_io_board", False)
    total = entry.get("total_outputs", None)
    io_outputs = entry.get("io_outputs", 0)

    lines = ["## PWM Outputs {#pwm_outputs}", ""]

    if not groups:
        lines.append("PWM output information could not be determined from source.")
        return "\n".join(lines)

    # Total outputs
    all_outputs = [o for g in groups for o in g["outputs"]]
    n = total or len(all_outputs)
    if has_io:
        lines.append(
            f"This flight controller supports up to {n} FMU PWM outputs (AUX) "
            f"and {io_outputs} IO PWM outputs (MAIN)."
        )
    else:
        lines.append(f"This flight controller supports up to {n} FMU PWM outputs (MAIN).")
    lines.append("")

    # DShot/BDShot summary
    dshot_outputs = sorted(set(o for g in groups for o in g.get("dshot_outputs", g["outputs"] if g.get("dshot") else [])))
    non_dshot_outputs = sorted(set(o for g in groups for o in g.get("non_dshot_outputs", [] if g.get("dshot") else g["outputs"])))
    bdshot_full = sorted(set(o for g in groups for o in g.get("bdshot_outputs", [])))
    bdshot_partial = sorted(set(o for g in groups for o in g.get("bdshot_output_only", [])))

    def plural_outputs(lst, verb_plural="support", verb_singular="supports"):
        r = make_outputs_range(lst)
        if len(lst) == 1:
            return f"Output {r} {verb_singular}"
        return f"Outputs {r} {verb_plural}"

    DSHOT_LINK = "[DShot](../peripherals/dshot.md)"
    BDSHOT_LINK = "[Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry)"
    dshot_linked = False
    bdshot_linked = False

    def dshot_ref():
        nonlocal dshot_linked
        if not dshot_linked:
            dshot_linked = True
            return DSHOT_LINK
        return "DShot"

    def bdshot_ref():
        nonlocal bdshot_linked
        if not bdshot_linked:
            bdshot_linked = True
            return BDSHOT_LINK
        return "Bidirectional DShot"

    all_outputs_set = sorted(set(o for g in groups for o in g["outputs"]))
    all_dshot = dshot_outputs == all_outputs_set
    all_bdshot = bdshot_full == all_outputs_set
    outputs_word = "FMU outputs" if has_io else "outputs"

    # Uniform cases: single statement, no label/bullet needed
    if not dshot_outputs:
        lines.append(f"{dshot_ref()} is not supported.")
        lines.append("")
    elif all_dshot and all_bdshot:
        lines.append(f"All {outputs_word} support {dshot_ref()} and {bdshot_ref()}.")
        lines.append("")
    elif all_dshot and not bdshot_full and not bdshot_partial:
        lines.append(f"All {outputs_word} support {dshot_ref()} ({bdshot_ref()} not supported).")
        lines.append("")
    else:
        # Mixed capabilities — use label + bullets
        outputs_label = "FMU Outputs:" if has_io else "Outputs:"
        lines.append(outputs_label)
        lines.append("")
        if dshot_outputs:
            lines.append(f"- {plural_outputs(dshot_outputs)} {dshot_ref()}.")
        if non_dshot_outputs:
            lines.append(f"- {plural_outputs(non_dshot_outputs, 'do not support', 'does not support')} {dshot_ref()}.")
        if bdshot_full:
            lines.append(f"- {plural_outputs(bdshot_full)} {bdshot_ref()}.")
        if bdshot_partial:
            lines.append(f"- {plural_outputs(bdshot_partial)} {bdshot_ref()} output only (no eRPM capture).")
        lines.append("")

    # Groups
    lines.append(f"The {n} outputs are in {len(groups)} groups:")
    lines.append("")
    for g in groups:
        o = g["outputs"]
        r = make_outputs_range(o)
        plural = "s" if len(o) > 1 else ""
        lines.append(f"- Output{plural} {r} in group{g['group']} ({g['timer']})")
    lines.append("")
    lines.append("All outputs within the same group must use the same output protocol and rate.")

    return "\n".join(lines)


def _gps_ports_from_entry(entry: dict) -> list:
    """Return serial_ports entries whose label looks like GPS1, GPS2, GPS, etc."""
    return [p for p in entry.get('serial_ports', [])
            if re.match(r'^GPS\d*$', p.get('label', ''))]


def _gps_source_json(board_key: str, entry: dict) -> str:
    """Build the JSON object embedded in the GPS & Compass section's HTML comment."""
    gps_ports = _gps_ports_from_entry(entry)
    data = {
        'board': board_key,
        'source': {
            'gps_ports': [
                {'label': p['label'], 'device': p['device'], 'uart': p['uart']}
                for p in gps_ports
            ],
            'has_pps_capture':   entry.get('has_pps_capture', False),
            'has_safety_switch': entry.get('has_safety_switch', False),
            'has_safety_led':    entry.get('has_safety_led', False),
            'has_buzzer':        entry.get('has_buzzer', False),
        },
        'gps_ports_wizard': entry.get('gps_ports_wizard'),
    }
    return json.dumps(data, indent=2)


GPS_CHECKLIST = [
    "- [ ] Confirm physical label(s) of GPS port(s) as printed on board",
    "- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)",
    "- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins",
    "- [ ] Note whether an external compass is required or integrated in GPS module",
    "- [ ] Add a connection diagram image showing GPS port location",
]


def generate_gps_section(board_key: str, entry: dict) -> str:
    """Generate a '### GPS & Compass' markdown section from parsed GPS config."""
    heading = "### GPS & Compass {#gps_compass}\n\n"
    intro = (
        "PX4 supports GPS modules connected to the GPS port(s) listed below.\n"
        "The module should be [mounted on the frame](../assembly/mount_gps_compass.md) "
        "as far away from other electronics as possible, "
        "with the direction marker pointing towards the front of the vehicle.\n"
    )

    has_safety_switch = entry.get('has_safety_switch', False)
    gps_ports_wizard = entry.get('gps_ports_wizard')

    # Auto-detect full 10-pin Pixhawk connector: all three GPS-connector features present
    has_full_connector = (
        entry.get('has_safety_switch', False)
        and entry.get('has_safety_led', False)
        and entry.get('has_buzzer', False)
    )

    if gps_ports_wizard:
        ports_to_render = gps_ports_wizard
    else:
        # Synthesise from serial_ports; only the primary (first) GPS port gets
        # the full 10-pin connector description — secondary ports use 6-pin basic.
        raw_ports = _gps_ports_from_entry(entry)
        if not raw_ports:
            raw_ports = [{'label': 'TODO: GPS port label', 'device': None, 'uart': None}]
        ports_to_render = []
        for i, p in enumerate(raw_ports):
            is_primary = (i == 0)
            ports_to_render.append({
                'port_key':          p['label'],
                'label':             p['label'],
                'pixhawk_standard':  has_full_connector and is_primary,
                'full_port':         has_full_connector and is_primary,
            })

    PIXHAWK_STD_URL = (
        "[Pixhawk Connector Standard]"
        "(https://github.com/pixhawk/Pixhawk-Standards/blob/master/"
        "DS-009%20Pixhawk%20Connector%20Standard.pdf)"
    )
    lines = ['\nThe GPS ports are:\n']
    for port in ports_to_render:
        label = port.get('label') or f"TODO: {port.get('port_key', 'GPS')} port label"
        if port.get('pixhawk_standard') and port.get('full_port'):
            connector = f"10-pin JST GH ({PIXHAWK_STD_URL})"
            features = "GPS, compass (I2C), safety switch, buzzer, LED"
        elif port.get('pixhawk_standard'):
            connector = f"6-pin JST GH ({PIXHAWK_STD_URL})"
            features = "GPS, compass (I2C)"
        else:
            connector = "TODO: connector type"
            features = "GPS, compass (I2C)"
        lines.append(f'- `{label}` (FMU): {connector} — {features}')

    body = '\n'.join(lines)

    safety_note = ''
    if has_safety_switch:
        safety_note = (
            "\n\nThe GPS module's integrated safety switch is enabled _by default_ "
            "(when enabled, PX4 will not let you arm the vehicle).\n"
            "To disable the safety switch press and hold it for 1 second.\n"
            "You can press the safety switch again to enable safety and disarm the vehicle."
        )

    doc_file = entry.get('doc_file', '')
    doc_name = doc_file.replace('.md', '') if doc_file else board_key.replace('/', '_')
    image_link = f"\n\n![GNSS Connection](../../assets/flight_controller/{doc_name}/gnss_connection.png)"
    checklist_comment = "\n\n<!-- checklist\n" + "\n".join(GPS_CHECKLIST) + "\n-->"
    source_comment = f"\n\n<!-- gps-source-data\n{_gps_source_json(board_key, entry)}\n-->"
    return heading + intro + body + safety_note + image_link + checklist_comment + source_comment


def generate_serial_section(board_key: str, entry: dict) -> str:
    """Generate a '## Serial Port Mapping' markdown table from parsed serial data."""
    ports = entry.get('serial_ports', [])

    lines = ['## Serial Port Mapping', '']

    if not ports:
        lines.append('Serial port mapping could not be determined from source.')
        return '\n'.join(lines)

    lines += [
        '| UART | Device | Port | Flow Control |',
        '|------|--------|------|:---:|',
    ]
    for p in ports:
        fc = 'Yes' if p.get('flow_control') else '-'
        lines.append(f"| {p['uart']} | {p['device']} | {p['label']} | {fc} |")

    return '\n'.join(lines)


def _rc_serial_protocols(entry: dict) -> list:
    """Ordered protocol list for the serial RC port, derived from source config.

    PPM is prepended only when the PPM pin is wired to the same physical pin
    as the serial UART RX (RC_SERIAL_PORT_SHARED_PPM_PIN_GPIO_RX).  When the
    board has a dedicated PPM pin that is NOT shared, PPM appears on a separate
    physical connector and must be listed as its own port entry.
    """
    ppm = ['PPM'] if (entry.get('has_ppm_pin') and entry.get('ppm_shared_with_rc_serial')) else []
    if entry.get('has_rc_input'):
        return ppm + ['SBUS', 'DSM/DSMX', 'ST24', 'SUMD', 'CRSF', 'GHST']
    if entry.get('has_common_rc'):
        return ppm + ['SBUS', 'DSM/DSMX', 'CRSF', 'GHST']
    if entry.get('has_io_board'):
        # IO board without rc_input/common_rc: basic serial via IO firmware
        return ppm + ['SBUS', 'DSM/DSMX', 'ST24', 'SUMD']
    return ppm


def _and_list(items: list) -> str:
    """['A', 'B', 'C'] → 'A, B, and C'.  ['A', 'B'] → 'A and B'.  ['A'] → 'A'."""
    if not items:
        return ''
    if len(items) == 1:
        return items[0]
    if len(items) == 2:
        return f'{items[0]} and {items[1]}'
    return ', '.join(items[:-1]) + ', and ' + items[-1]


def _derive_rc_label(entry: dict) -> str:
    """Best-effort RC port label from parsed serial_ports (used when wizard data absent)."""
    rc_dev = entry.get('rc_serial_device')
    for port in entry.get('serial_ports', []):
        if rc_dev and port.get('device') == rc_dev:
            return port.get('label', '')
        if port.get('label') in ('RC', 'PX4IO/RC'):
            return port.get('label', '')
    return ''


def _infer_rc_side(entry: dict) -> str:
    """Infer IO/FMU for the serial RC port from source data."""
    rc_dev = entry.get('rc_serial_device')
    for port in entry.get('serial_ports', []):
        if (rc_dev and port.get('device') == rc_dev) or port.get('label') in ('RC', 'PX4IO/RC'):
            return 'IO' if 'PX4IO' in port.get('label', '') else 'FMU'
    return 'IO' if entry.get('has_io_board') else 'FMU'


def _rc_source_json(board_key: str, entry: dict) -> str:
    """Build the JSON object embedded in the RC section's HTML comment."""
    has_rc_input = entry.get('has_rc_input', False)
    has_common_rc = entry.get('has_common_rc', False)
    has_io_board = entry.get('has_io_board', False)
    rc_serial_device = entry.get('rc_serial_device')

    # Locate RC port and IO port entries in serial_ports
    rc_port_entry = None
    io_port_entry = None
    for port in entry.get('serial_ports', []):
        label = port.get('label', '')
        dev = port.get('device')
        if dev == rc_serial_device and rc_port_entry is None:
            rc_port_entry = port
        elif label in ('RC', 'PX4IO/RC') and rc_port_entry is None:
            rc_port_entry = port
        if 'PX4IO' in label and io_port_entry is None:
            io_port_entry = port

    rc_serial = None
    if rc_port_entry:
        label = rc_port_entry.get('label', '')
        side = 'IO' if 'PX4IO' in label else 'FMU'
        rc_serial = {
            'device': rc_port_entry['device'],
            'uart':   rc_port_entry['uart'],
            'label':  label,
            'side':   side,
        }
    elif rc_serial_device:
        rc_serial = {'device': rc_serial_device, 'uart': None, 'label': None, 'side': 'FMU'}

    io_serial = None
    if io_port_entry:
        io_serial = {
            'device': io_port_entry['device'],
            'uart':   io_port_entry['uart'],
            'label':  io_port_entry.get('label', ''),
        }

    has_ppm_pin = entry.get('has_ppm_pin', False)
    ppm_shared = entry.get('ppm_shared_with_rc_serial', False)

    data = {
        'board': board_key,
        'modules': {
            'rc_input':               has_rc_input,
            'common_rc':              has_common_rc,
            'px4io':                  has_io_board,
            'has_ppm_pin':            has_ppm_pin,
            'ppm_shared_with_serial': ppm_shared,
        },
        'rc_serial':     rc_serial,
        'io_serial':     io_serial,
        'rc_ports_wizard': entry.get('rc_ports_wizard'),
    }
    return json.dumps(data, indent=2)


RC_CHECKLIST = [
    "- [ ] List all RC port names exactly as labeled on the board",
    "- [ ] State which protocols each port supports",
    "- [ ] State the connector type used for each RC port",
    "- [ ] Confirm which protocols have been tested with this board",
    "- [ ] Add a connection diagram image showing where to plug in the RC receiver",
]


def generate_radio_control_section(board_key: str, entry: dict) -> str:
    """Generate a '### Radio Control' markdown section from parsed RC config."""
    intro = (
        "### Radio Control {#radio_control}\n\n"
        "A remote control (RC) radio system is required if you want to _manually_ control your "
        "vehicle (PX4 does not require a radio system for autonomous flight modes).\n\n"
        "You will need to [select a compatible transmitter/receiver]"
        "(../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they "
        "communicate (read the instructions that come with your specific transmitter/receiver).\n"
    )

    serial_protos = _rc_serial_protocols(entry)
    has_ppm_pin = entry.get('has_ppm_pin', False)
    ppm_shared = entry.get('ppm_shared_with_rc_serial', False)

    rc_ports = entry.get('rc_ports_wizard')

    has_io_board = entry.get('has_io_board', False)

    if not rc_ports:
        # No wizard data — synthesise a single-port entry from source so the output
        # is still in bullet format and has the right protocols.
        label = _derive_rc_label(entry) or 'TODO: RC port label'
        side = _infer_rc_side(entry)
        rc_ports = [{'label': label, 'side': side}]
        # Add a dedicated PPM placeholder only when:
        #  - there is a separate PPM GPIO (not sharing the serial UART RX), AND
        #  - there is no IO board (IO boards handle PPM internally via the IO RC port)
        if has_ppm_pin and not ppm_shared and not has_io_board:
            rc_ports.append({'label': 'TODO: PPM port label', 'side': side, 'ppm_only': True})

    lines = ['\nThe ports and supported protocols are:\n']
    for port in rc_ports:
        label = port['label']
        side = port.get('side', 'FMU')
        if port.get('ppm_only'):
            proto_str = 'PPM'
        elif serial_protos:
            proto_str = _and_list(serial_protos) + ' receivers'
        else:
            proto_str = 'TODO: list supported protocols'
        lines.append(f'- `{label}` ({side}): {proto_str}')

    # Note about multi-channel PWM receivers requiring a PPM encoder
    lines.append(
        '\nFor PPM and S.Bus receivers, a single signal wire carries all channels. '
        'If your receiver outputs individual PWM signals (one wire per channel) '
        'it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).'
    )

    body = '\n'.join(lines)
    doc_file = entry.get('doc_file', '')
    doc_name = doc_file.replace('.md', '') if doc_file else board_key.replace('/', '_')
    image_link = f"\n\n![RC Connection Diagram](../../assets/flight_controller/{doc_name}/rc_connection_diagram.png)"
    checklist_comment = "\n\n<!-- checklist\n" + "\n".join(RC_CHECKLIST) + "\n-->"
    source_comment = f"\n\n<!-- rc-source-data\n{_rc_source_json(board_key, entry)}\n-->"
    return intro + body + image_link + checklist_comment + source_comment


def _power_source_json(board_key: str, entry: dict) -> str:
    """Return the JSON blob embedded in the power section source comment."""
    return json.dumps({
        'board': board_key,
        'source': {
            'num_power_inputs':          entry.get('num_power_inputs', 1),
            'has_redundant_power':       entry.get('has_redundant_power', False),
            'has_dual_battery_monitoring': entry.get('has_dual_battery_monitoring', False),
            'has_dronecan_power_input':  entry.get('has_dronecan_power_input', False),
            'power_monitor_type':        entry.get('power_monitor_type'),
        },
        'power_ports_wizard': entry.get('power_ports_wizard'),
    }, indent=2)


def _render_power_port(port: dict, board_level_type: str | None) -> str:
    """Render a single power port list item, with DroneCAN-specific text if needed."""
    label = port.get('label') or 'TODO: POWER port label'
    ctype = port.get('connector_type') or 'TODO: connector type'
    mtype = port.get('monitor_type') or board_level_type or 'analog'

    if mtype == 'dronecan':
        return (
            f'- `{label}`: {ctype} — DroneCAN battery monitoring.\n'
            f'  Connect a [DroneCAN power module](../power_module/index.md) to the CAN bus.\n'
            f'  Set `UAVCAN_ENABLE = 2` (Sensors Automatic Config) '
            f'and `UAVCAN_SUB_BAT = 1` (Raw data).'
        )
    else:
        return f'- `{label}`: {ctype}'


def generate_power_section(board_key: str, entry: dict) -> str:
    """Generate the ## Power {#power} section for an FC doc page.

    Content is driven by:
      - Parsed power config (num_power_inputs, has_redundant_power, etc.)
      - power_ports_wizard override list (label + connector_type + optional monitor_type per port)
    """
    doc_name = board_key.split('/', 1)[1] if '/' in board_key else board_key

    power_ports_wizard = entry.get('power_ports_wizard')
    num_inputs = entry.get('num_power_inputs', 1)
    has_redundant = entry.get('has_redundant_power', False)
    board_level_type = entry.get('power_monitor_type')

    # Build the port description
    if power_ports_wizard:
        ports = power_ports_wizard
    else:
        if has_redundant:
            ports = [
                {'label': f'TODO: POWER {i+1} port label',
                 'connector_type': 'TODO: connector type'}
                for i in range(num_inputs)
            ]
        else:
            ports = [{'label': 'TODO: POWER port label',
                      'connector_type': 'TODO: connector type'}]

    # Check for mixed port types (DroneCAN + non-DroneCAN)
    port_types = {p.get('monitor_type') for p in ports if p.get('monitor_type')}
    has_dronecan_port = 'dronecan' in port_types or board_level_type == 'dronecan'
    has_mixed_types = 'dronecan' in port_types and len(port_types) > 1

    # Introduction sentence — single vs redundant vs mixed
    if has_mixed_types:
        non_dronecan = [p for p in ports if p.get('monitor_type') != 'dronecan']
        dronecan = [p for p in ports if p.get('monitor_type') == 'dronecan']
        nc_labels = ' and '.join(f'**{p["label"]}**' for p in non_dronecan)
        dc_labels = ' and '.join(f'**{p["label"]}**' for p in dronecan)
        intro_ports = (
            f'The flight controller supports two power inputs: '
            f'{nc_labels} for a standard power module and {dc_labels} for a DroneCAN power module.'
        )
    elif has_redundant and len(ports) >= 2:
        port_labels = ' or '.join(f'**{p["label"]}**' for p in ports)
        intro_ports = (
            f'The flight controller can be powered from a power module connected to the '
            f'{port_labels} port, providing redundant power inputs.\n'
            f'If one power module fails the controller automatically switches to the other.'
        )
    else:
        intro_ports = (
            f'The flight controller can be powered from a power module connected to the '
            f'**{ports[0]["label"]}** port.'
        )

    # Port list (label + connector type, with DroneCAN-specific text)
    port_lines = [_render_power_port(p, board_level_type) for p in ports]
    port_block = '\n'.join(port_lines)

    warning = (
        ':::warning\n'
        'The PWM output ports are not powered by the POWER port.\n'
        'The output rail must be [separately powered](../assembly/servo_power.md) '
        'if it needs to power servos or other hardware.\n'
        'This is generally true for VTOL and fixed-wing vehicles, '
        'but not necessarily true for MC vehicles.\n'
        ':::'
    )

    body = (
        f'## Power {{#power}}\n\n'
        f'{intro_ports}\n\n'
        f'The power module must supply a regulated **5V** at a minimum of **3A continuous**.\n\n'
        f'Power ports:\n\n'
        f'{port_block}\n\n'
        f'{warning}\n\n'
        f'For battery and power module configuration see '
        f'[Battery and Power Module Setup](../config/battery.md).'
    )

    image_link = (
        f'\n\n![Power Connection](../../assets/flight_controller/{doc_name}/power_connection.png)'
    )
    checklist = (
        '\n\n<!-- checklist\n'
        '- [ ] Confirm power port label(s) as printed on board\n'
        '- [ ] Confirm connector type(s)\n'
        '- [ ] Confirm voltage/current ratings of power module used\n'
        '- [ ] Add a connection diagram image showing power port location\n'
        '-->'
    )
    source_comment = f'\n\n<!-- power-source-data\n{_power_source_json(board_key, entry)}\n-->'

    return body + image_link + checklist + source_comment


def generate_telemetry_section(board_key: str, entry: dict) -> str:
    """Generate the ## Telemetry Radios section for an FC doc page.

    Content is driven by the TELEM* entries in serial_ports.
    TELEM1 is always the default (no configuration needed when connected there).
    """
    doc_name = board_key.split('/', 1)[1] if '/' in board_key else board_key

    # Extract and sort TELEM ports from serial_ports
    all_ports = entry.get('serial_ports', [])
    telem_ports = sorted(
        [p for p in all_ports if p.get('label', '').startswith('TELEM')],
        key=lambda p: p['label'],
    )
    telem_labels = [p['label'] for p in telem_ports]

    # Connection sentence depends on number of TELEM ports
    if not telem_labels:
        connection = (
            'The vehicle-based radio should be connected to the '
            '**TODO: TELEM port label** port '
            '(if connected to this port, no further configuration is required).'
        )
    elif len(telem_labels) == 1:
        connection = (
            f'The vehicle-based radio should be connected to the '
            f'**{telem_labels[0]}** port '
            f'(if connected to this port, no further configuration is required).'
        )
    else:
        port_list = ', '.join(f'**{l}**' for l in telem_labels)
        connection = (
            f'The vehicle-based radio should be connected to a TELEM port — '
            f'{port_list} '
            f'(if connected to **{telem_labels[0]}**, no further configuration is required).'
        )

    body = (
        f'## Telemetry Radios (Optional) {{#telemetry}}\n\n'
        f'[Telemetry radios](../telemetry/index.md) may be used to communicate and control '
        f'a vehicle in flight from a ground station (for example, you can direct the UAV to '
        f'a particular position, or upload a new mission).\n\n'
        f'{connection}\n'
        f'The other radio is connected to your ground station computer or mobile device '
        f'(usually by USB).'
    )

    image_link = (
        f'\n\n![Telemetry Radio]'
        f'(../../assets/flight_controller/{doc_name}/telemetry_radio.jpg)'
    )
    checklist = (
        '\n\n<!-- checklist\n'
        '- [ ] Confirm TELEM port label(s) as printed on board\n'
        f'- [ ] Add a wiring photo showing telemetry radio connected to '
        f'{telem_labels[0] if telem_labels else "TELEM1"}\n'
        '-->'
    )
    source_data = json.dumps({
        'board': board_key,
        'source': {'telem_ports': telem_labels},
    }, indent=2)
    source_comment = f'\n\n<!-- telemetry-source-data\n{source_data}\n-->'

    return body + image_link + checklist + source_comment


def generate_sd_card_section(board_key: str, entry: dict) -> str:
    """Generate the ## SD Card section for an FC doc page."""
    doc_name = board_key.split('/', 1)[1] if '/' in board_key else board_key
    has_sd = entry.get('has_sd_card', False)

    source_data = json.dumps({
        'board': board_key,
        'source': {'has_sd_card': has_sd},
    }, indent=2)
    source_comment = f'\n\n<!-- sd-source-data\n{source_data}\n-->'

    if not has_sd:
        return (
            '## SD Card\n\n'
            'This board does not have an SD card slot.'
            + source_comment
        )

    body = (
        '## SD Card (Optional) {#sd_card}\n\n'
        'SD cards are highly recommended as they are needed to '
        '[log and analyse flight details](../getting_started/flight_reporting.md), '
        'to run missions, and to use UAVCAN-bus hardware.\n'
        f'Insert the card into _TODO: {doc_name} board name_ as shown below.'
    )
    image_link = (
        f'\n\n![SD Card Slot]'
        f'(../../assets/flight_controller/{doc_name}/sd_card_slot.jpg)'
    )
    tip = (
        '\n\n:::tip\n'
        'For more information see '
        '[Basic Concepts > SD Cards (Removable Memory)]'
        '(../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).\n'
        ':::'
    )
    checklist = (
        '\n\n<!-- checklist\n'
        '- [ ] Confirm SD card slot location\n'
        '- [ ] Add photo showing SD card insertion\n'
        '-->'
    )

    return body + image_link + tip + checklist + source_comment


# ---------------------------------------------------------------------------
# Specifications section generator
# ---------------------------------------------------------------------------

def _fmt_list(items: list) -> str:
    """Format a list of sensor/chip names as a comma-separated string."""
    return ', '.join(str(i) for i in items if i)


def _fmt_list_counted(items: list) -> str:
    """Format a list of names, collapsing consecutive duplicates with a count suffix.

    E.g. ['MS5611', 'MS5611'] → 'MS5611 (2)'
         ['ICM-42688P', 'MS5611'] → 'ICM-42688P, MS5611'
    """
    from collections import Counter
    counts = Counter(str(i) for i in items if i)
    return ', '.join(
        f'{name} ({n})' if n > 1 else name
        for name, n in counts.items()
    )


def generate_specifications_section(board_key: str, entry: dict) -> str:
    """Generate a '## Specifications {#specifications}' markdown section.

    Auto-detected fields (from board source files):
      - Processor model + specs (via chip_model → CHIP_SPECS lookup)
      - IO processor (if has_io_board)
      - PWM output count and breakdown (FMU / IO)
      - SD card, Ethernet, onboard heater
      - Number of serial ports

    Wizard-supplied fields (from overview_wizard in entry):
      - IMU(s), barometer(s), magnetometer(s), OSD chip
      - Dimensions, weight, input voltage range
    """
    lines = ['## Specifications {#specifications}', '']

    ow = entry.get('overview_wizard') or {}

    # Variant info (may be None for entries loaded from old metadata JSON)
    _sv_info = entry.get('sensor_variant_info') or {}
    _has_variants = _sv_info.get('has_variants', False)
    _sv_unconditional = _sv_info.get('unconditional') or {}
    _sv_variants = _sv_info.get('variants') or {}
    _variant_labels: dict[str, str] = (ow.get('sensor_variant_labels') or {})

    # Build a lookup from display name → list of bus entries (for annotation).
    # When variant data is available, use the unconditional sensors as the source so
    # that variant-specific sensors are not counted in the "always-present" bus list.
    _bus_source = _sv_unconditional if _has_variants else (entry.get('sensor_bus_info') or {})
    _bus_info: dict[str, list] = {}
    for _cat, _entries in _bus_source.items():
        for _e in _entries:
            _bus_info.setdefault(_e['name'], []).append(_e)

    def _fmt_bus_annotation(bus_entry: dict) -> str:
        """Return ' (SPI)', ' (I2C, internal)', ' (I2C bus 1, external)', or ''."""
        bt = bus_entry.get('bus_type')
        if not bt:
            return ''
        if bt == 'SPI':
            return ' (SPI)'
        # I2C
        parts = ['I2C']
        bn = bus_entry.get('bus_num')
        if bn is not None:
            parts.append(f'bus {bn}')
        if bus_entry.get('external'):
            parts.append('external')
        else:
            parts.append('internal')
        return f' ({", ".join(parts)})'

    def _annotate_name(display_name: str, bus_lookup: dict | None = None) -> str:
        """Annotate a sensor name with bus info if available.

        If the name maps to multiple bus instances (e.g. internal + external IST8310),
        expand to a comma-separated list of annotated names.
        """
        lookup = bus_lookup if bus_lookup is not None else _bus_info
        instances = lookup.get(display_name)
        if not instances:
            return display_name
        if len(instances) == 1:
            return display_name + _fmt_bus_annotation(instances[0])
        return ', '.join(display_name + _fmt_bus_annotation(e) for e in instances)

    def _fmt_variant_label(code: str) -> str:
        """Return a human-readable variant label for use in annotations."""
        custom = _variant_labels.get(code)
        if custom:
            return f'hardware revision {custom}'
        if code == '__other__':
            return 'other variants'
        return f'variant {code}'

    def _resolve_sensor(wizard_key: str, driver_key: str, label: str,
                        category: str, required: bool = False) -> str | None:
        """Return a formatted sensor line, or None to omit.

        Priority:
          1. Wizard value (authoritative) — always used when set
          2. Variant-aware display — when sensor_variant_info is available
          3. Multiple drivers detected — listed as unconfirmed candidates (with bus annotations)
          4. Single driver detected — used directly (with bus annotation)
          5. Nothing detected — 'TODO' placeholder (only when required=True)
        """
        wizard_val = ow.get(wizard_key)
        if wizard_val:
            display = _fmt_list_counted(wizard_val) if isinstance(wizard_val, list) else wizard_val
            sensor_line = f'- **{label}**: {display}'
            # Note any detected drivers not confirmed by the wizard — may be unnecessary firmware bloat
            confirmed_upper = {w.upper() for w in (wizard_val if isinstance(wizard_val, list) else [wizard_val])}
            detected_raw = entry.get(driver_key) or []
            unused = [d for d in detected_raw if d.upper() not in confirmed_upper]
            if unused:
                sensor_line += f' <!-- TODO: Unnecessary driver(s) in firmware?: {", ".join(unused)}. -->'
            return sensor_line

        if _has_variants:
            # Build unconditional base set for this category
            uncond_entries = _sv_unconditional.get(category, [])
            # Compute names present in unconditional (for dedup check)
            uncond_names = {e['name'] for e in uncond_entries}

            # Annotate unconditional entries using the unconditional bus lookup
            base_parts = []
            seen_base = set()
            for _e in uncond_entries:
                annotated = _annotate_name(_e['name'])
                if annotated not in seen_base:
                    seen_base.add(annotated)
                    base_parts.append(annotated)

            # Collect variant-unique sensors (in a variant but NOT in unconditional)
            variant_parts = []
            for code, vdata in _sv_variants.items():
                v_entries = vdata.get(category, [])
                for _ve in v_entries:
                    if _ve['name'] not in uncond_names:
                        # Build a per-variant bus lookup for this chip
                        v_bus_lookup: dict[str, list] = {}
                        for _vc, _vdata in _sv_variants.items():
                            for _vve in _vdata.get(category, []):
                                v_bus_lookup.setdefault(_vve['name'], []).append(_vve)
                        bus_ann = _fmt_bus_annotation(_ve)  # e.g. ' (SPI)' or ''
                        label_str = _fmt_variant_label(code)
                        if bus_ann:
                            # Merge into a single parenthetical: '(SPI, variant HW000001)'
                            combined_ann = bus_ann.rstrip(')') + f', {label_str})'
                        else:
                            combined_ann = f' ({label_str})'
                        variant_parts.append(_ve['name'] + combined_ann)

            # Deduplicate variant_parts while preserving order
            seen_vp: set = set()
            unique_variant_parts = []
            for vp in variant_parts:
                if vp not in seen_vp:
                    seen_vp.add(vp)
                    unique_variant_parts.append(vp)

            if base_parts and unique_variant_parts:
                combined = _fmt_list(base_parts) + '; ' + _fmt_list(unique_variant_parts)
                return f'- **{label}**: {combined}'
            if base_parts:
                return f'- **{label}**: {_fmt_list(base_parts)}'
            if unique_variant_parts:
                return f'- **{label}**: {_fmt_list(unique_variant_parts)}'
            if required:
                return f'- **{label}**: TODO: list {label.lower()}(s)'
            return None

        # No variant info — fall back to old driver-list behaviour
        drivers = entry.get(driver_key) or []
        if len(drivers) == 1:
            return f'- **{label}**: {_annotate_name(drivers[0])}'
        if len(drivers) > 1:
            candidates = _fmt_list([_annotate_name(d) for d in drivers])
            return f'- **{label}**: TODO: confirm which is installed — {candidates}'
        if required:
            return f'- **{label}**: TODO: list {label.lower()}(s)'
        return None

    # --- Processor ---
    lines.append('- **Processor**')
    chip_model = entry.get('chip_model')
    specs = CHIP_SPECS.get(chip_model) if chip_model else None
    if chip_model and specs:
        lines.append(
            f'  - **Main FMU Processor**: {chip_model} '
            f'(32-bit Arm\u00ae {specs["core"]}, {specs["mhz"]} MHz, '
            f'{specs["flash"]} flash, {specs["ram"]} RAM)'
        )
    elif chip_model:
        lines.append(f'  - **Main FMU Processor**: {chip_model}')
    else:
        lines.append('  - **Main FMU Processor**: TODO: chip model')

    if entry.get('has_io_board'):
        lines.append(f'  - **IO Processor**: {_IO_PROC_SPEC}')

    # --- Sensors ---
    lines.append('- **Sensors**')
    imu_line  = _resolve_sensor('imu',  'sensor_imu_drivers',  'IMU',          category='imu',  required=True)
    baro_line = _resolve_sensor('baro', 'sensor_baro_drivers', 'Barometer',    category='baro', required=True)
    mag_line  = _resolve_sensor('mag',  'sensor_mag_drivers',  'Magnetometer', category='mag',  required=True)
    osd_line  = _resolve_sensor('osd',  'sensor_osd_drivers',  'OSD',          category='osd',  required=False)
    if imu_line:  lines.append('  ' + imu_line)
    if baro_line: lines.append('  ' + baro_line)
    if mag_line:  lines.append('  ' + mag_line)
    if osd_line:  lines.append('  ' + osd_line)

    # --- Interfaces ---
    lines.append('- **Interfaces**')
    # Use group-derived count — DIRECT_PWM_OUTPUT_CHANNELS in board_config.h
    # counts all FMU timer channels, which matches groups since
    # initIOTimerChannelCapture entries are also counted as outputs.
    groups = entry.get('groups') or []
    fmu_out = sum(len(g.get('outputs', [])) for g in groups)
    io_out = entry.get('io_outputs', 0) or 0
    has_io = entry.get('has_io_board', False)
    if has_io and io_out:
        lines.append(f'  - **PWM outputs**: {fmu_out + io_out} ({fmu_out} FMU + {io_out} IO)')
    elif fmu_out:
        lines.append(f'  - **PWM outputs**: {fmu_out} (FMU)')

    num_serial = len(entry.get('serial_ports') or [])
    if num_serial:
        lines.append(f'  - **Serial ports**: {num_serial}')

    _BUS_CAT_LABEL = {
        'imu': 'IMU', 'baro': 'barometer', 'mag': 'magnetometer',
        'osd': 'OSD', 'power_monitor': 'power monitor',
    }
    # For bus sub-bullets, use unconditional sensors when variant data is available,
    # so variant-specific sensors don't appear in the "always-present" bus list.
    _bus_sub_source = _sv_unconditional if _has_variants else (entry.get('sensor_bus_info') or {})
    spi_sensors = [(cat, e) for cat, entries in _bus_sub_source.items()
                   for e in entries if e.get('bus_type') == 'SPI']
    i2c_sensors = [(cat, e) for cat, entries in _bus_sub_source.items()
                   for e in entries if e.get('bus_type') == 'I2C']

    num_i2c = entry.get('num_i2c_buses', 0)

    # Build per-bus maps from sensor data
    _ext_bus_map = {}   # bus_num(int) → [(cat, sensor_name), ...]
    _int_sensors  = []  # (cat, sensor_name) for internal I2C sensors (bus_num=None)
    for _cat, _e in i2c_sensors:
        _bn = _e.get('bus_num')
        if _e.get('external') and _bn is not None:
            _ext_bus_map.setdefault(_bn, []).append((_cat, _e['name']))
        elif not _e.get('external'):
            _int_sensors.append((_cat, _e['name']))
    _n_ext = len(_ext_bus_map)

    _i2c_wizard = entry.get('i2c_buses_wizard') or []
    _wizard_by_bus = {b['bus_num']: b['label'] for b in _i2c_wizard}
    # Auto-detected port labels from source comments (e.g. "# External compass on GPS1/I2C1:")
    _src_port_labels: dict[int, str] = {}
    for _cat_entries in _bus_sub_source.values():
        for _e in _cat_entries:
            _bn2 = _e.get('bus_num')
            _pl = _e.get('port_label')
            if _pl and _bn2 is not None and _bn2 not in _src_port_labels:
                _src_port_labels[_bn2] = _pl

    _i2c_bus_config = entry.get('i2c_bus_config')   # {'external': [...], 'internal': [...]} or None

    if _i2c_bus_config:
        # DETAILED PATH: authoritative per-bus routing from src/i2c.cpp
        _ext_buses_cfg = set(_i2c_bus_config.get('external', []))
        _int_buses_cfg = set(_i2c_bus_config.get('internal', []))
        _all_buses = sorted(_ext_buses_cfg | _int_buses_cfg)
        _n_ext_cfg = len(_ext_buses_cfg)
        _n_int_cfg = len(_int_buses_cfg)
        _total = len(_all_buses)

        if not _total:
            lines.append('  - **I2C ports**: TODO: number of I2C ports')
        elif _n_ext_cfg > 0 and _n_int_cfg > 0:
            lines.append(f'  - **I2C ports**: {_total} ({_n_ext_cfg} external, {_n_int_cfg} internal)')
        elif _n_ext_cfg > 0:
            lines.append(f'  - **I2C ports**: {_total} ({_n_ext_cfg} external)')
        else:
            lines.append(f'  - **I2C ports**: {_total} ({_n_int_cfg} internal)')

        for _bn in _all_buses:
            _is_ext = _bn in _ext_buses_cfg
            _routing = 'external' if _is_ext else 'internal'
            _label = _wizard_by_bus.get(_bn) or _src_port_labels.get(_bn)
            _paren = f'{_routing}, {_label}' if _label else _routing
            if _is_ext:
                _sensor_parts = _ext_bus_map.get(_bn, [])
                if _sensor_parts:
                    _notes = ', '.join(
                        f'{_nm} ({_BUS_CAT_LABEL.get(_c, _c)})' for _c, _nm in _sensor_parts
                    )
                    _bus_str = f'I2C{_bn} ({_paren}): {_notes}'
                    if _label and re.match(r'^GPS\d+$', _label):
                        _bus_str += ' — on GPS connector'
                else:
                    _bus_str = f'I2C{_bn} ({_paren}): free (no sensor detected)'
            else:
                # Internal bus — try to attach known internal sensors when only one internal bus
                _bus_str = f'I2C{_bn} ({_paren})'
            lines.append(f'    - {_bus_str}')

        # For internal buses: internal sensors have bus_num=None so we can't map per-bus.
        # When exactly one internal bus, retrofit sensor list onto its bullet.
        # Otherwise add a grouped summary line.
        if _int_sensors:
            _notes = ', '.join(f'{_nm} ({_BUS_CAT_LABEL.get(_c, _c)})' for _c, _nm in _int_sensors)
            if _n_int_cfg == 1:
                _int_bn = sorted(_int_buses_cfg)[0]
                _label = _wizard_by_bus.get(_int_bn) or _src_port_labels.get(_int_bn)
                _paren = f'internal, {_label}' if _label else 'internal'
                _target = f'    - I2C{_int_bn} (internal)'
                for _i in range(len(lines) - 1, -1, -1):
                    if lines[_i] == _target:
                        lines[_i] = f'    - I2C{_int_bn} ({_paren}): {_notes}'
                        break
            else:
                lines.append(f'    - Internal buses: {_notes}')

    else:
        # FALLBACK PATH: existing format when i2c_bus_config is absent
        if not num_i2c:
            lines.append('  - **I2C ports**: TODO: number of I2C ports')
        elif _n_ext > 0 and _n_ext <= num_i2c:
            _n_int = num_i2c - _n_ext
            lines.append(f'  - **I2C ports**: {num_i2c} ({_n_int} internal, {_n_ext} external)')
        else:
            lines.append(f'  - **I2C ports**: {num_i2c}')

        _all_ext_bus_nums = sorted(set(list(_ext_bus_map) + list(_wizard_by_bus)))
        for _bn in _all_ext_bus_nums:
            _label = _wizard_by_bus.get(_bn) or _src_port_labels.get(_bn) or f'TODO: label for I2C bus {_bn}'
            _sensor_parts = _ext_bus_map.get(_bn, [])
            _bus_str = f'{_label} (I2C{_bn}, external)'
            if _sensor_parts:
                _notes = ', '.join(f'{_nm} ({_BUS_CAT_LABEL[_c]})' for _c, _nm in _sensor_parts)
                _bus_str += f': {_notes}'
            if re.match(r'^GPS\d+$', _label):
                _bus_str += ' — on GPS connector'
            lines.append(f'    - {_bus_str}')

        if _int_sensors:
            _notes = ', '.join(f'{_nm} ({_BUS_CAT_LABEL[_c]})' for _c, _nm in _int_sensors)
            lines.append(f'    - Internal: {_notes}')

    num_spi = entry.get('num_spi_buses', 0)
    lines.append(f'  - **SPI buses**: {num_spi}' if num_spi else '  - **SPI buses**: TODO: number of SPI buses')
    for _cat, _e in spi_sensors:
        lines.append(f'    - {_e["name"]} ({_BUS_CAT_LABEL[_cat]})')

    num_can = entry.get('num_can_buses', 0)
    if num_can:
        lines.append(f'  - **CAN buses**: {num_can}')
    # CAN omitted (no TODO) when not detected — not all FCs have CAN

    if entry.get('has_usb'):
        # Support both new usb_connectors list and legacy usb_connector string
        connectors = ow.get('usb_connectors') or (
            [ow['usb_connector']] if ow.get('usb_connector') else []
        )
        labels = ow.get('usb_labels') or []
        non_default_labels = any(l != 'USB' for l in labels)
        if non_default_labels:
            # Pair each label with its connector (if available)
            parts = []
            for i, lbl in enumerate(labels):
                conn = connectors[i] if i < len(connectors) else None
                parts.append(f'{lbl} ({conn})' if conn else lbl)
            lines.append(f'  - **USB**: {", ".join(parts)}')
        elif connectors:
            from collections import Counter
            parts = [f'{c} (\u00d72)' if n > 1 else c for c, n in Counter(connectors).items()]
            lines.append(f'  - **USB**: {", ".join(parts)}')
        else:
            lines.append('  - **USB**: Yes')
    else:
        lines.append('  - **USB**: TODO: confirm USB connector type')

    # RC input
    has_rc = entry.get('has_rc_input', False)
    has_common_rc = entry.get('has_common_rc', False)
    has_ppm = entry.get('has_ppm_pin', False)
    has_io = entry.get('has_io_board', False)
    if has_io:
        # IO processor handles multi-protocol RC; list all supported protocols
        if has_ppm:
            lines.append('  - **RC input**: SBUS, DSM/DSMX, ST24, SUMD, CRSF, GHST, PPM (via IO)')
        else:
            lines.append('  - **RC input**: SBUS, DSM/DSMX, ST24, SUMD, CRSF, GHST (via IO)')
    elif has_common_rc and has_ppm:
        lines.append('  - **RC input**: DSM/SRXL2, S.Bus/CPPM')
    elif has_common_rc:
        lines.append('  - **RC input**: DSM/SRXL2, S.Bus')
    elif has_ppm:
        lines.append('  - **RC input**: PPM')
    elif has_rc:
        lines.append('  - **RC input**: Yes')

    # Analog battery monitoring
    num_power = entry.get('num_power_inputs', 0) or 0
    if num_power:
        lines.append(f'  - **Analog battery inputs**: {num_power}')

    # Additional analog inputs (wizard, always TODO if not set)
    num_adc = ow.get('num_additional_adc_inputs')
    if num_adc is not None:
        lines.append(f'  - **Additional analog inputs**: {num_adc}')
    else:
        lines.append('  - **Additional analog inputs**: TODO: number of additional analog inputs')

    if entry.get('has_ethernet'):
        lines.append('  - **Ethernet**: Yes')

    # --- Electrical Data (always emitted; TODO when wizard data absent) ---
    min_v = ow.get('min_voltage')
    max_v = ow.get('max_voltage')
    # Backward compat: old JSONs may have voltage_range as a single string
    if not min_v and not max_v and ow.get('voltage_range'):
        max_v = ow['voltage_range']
    lines.append('- **Electrical Data**')
    if min_v and max_v:
        lines.append(f'  - **Operating voltage**: {min_v}\u2013{max_v} (V)')
    elif max_v:
        lines.append(f'  - **Operating voltage**: TODO\u2013{max_v} (V)')
    elif min_v:
        lines.append(f'  - **Operating voltage**: {min_v}\u2013TODO (V)')
    else:
        lines.append('  - **Operating voltage**: TODO: supply voltage range')
    # USB power input line
    if ow.get('usb_powers_fc') and ow.get('usb_pwr_min_v') and ow.get('usb_pwr_max_v'):
        lines.append(f'  - **USB-C power input**: {ow["usb_pwr_min_v"]}\u2013{ow["usb_pwr_max_v"]} (V)')
    elif ow.get('usb_powers_fc'):
        lines.append('  - **USB-C power input**: TODO: USB-C voltage range')
    # Servo rail line
    if ow.get('has_servo_rail'):
        srv_max = ow.get('servo_rail_max_v')
        _srv_suffix = ' (servo rail must be separately powered and does not power the autopilot)'
        if srv_max:
            try:
                _srv_high = float(str(srv_max).rstrip('Vv').strip()) > 12
            except ValueError:
                _srv_high = False
            if _srv_high:
                lines.append(f'  - **Servo rail**: High-voltage capable, up to {srv_max} (V){_srv_suffix}')
            else:
                lines.append(f'  - **Servo rail**: Up to {srv_max} (V){_srv_suffix}')
        else:
            lines.append(f'  - **Servo rail**: TODO: servo rail max voltage{_srv_suffix}')
    # Power supply redundancy line
    _usb_pwr = ow.get('usb_powers_fc', False)
    _n_pwr = entry.get('num_power_inputs', 1)
    _total_sources = _n_pwr + (1 if _usb_pwr else 0)
    if _total_sources >= 2:
        _pwr_wizard = entry.get('power_ports_wizard') or []
        if _pwr_wizard:
            _src_names = [p['label'] for p in _pwr_wizard]
        else:
            _src_names = [f'POWER{i+1}' if _n_pwr > 1 else 'POWER' for i in range(_n_pwr)]
        if _usb_pwr:
            _src_names.append('USB-C')
        _level = {2: 'Dual', 3: 'Triple'}.get(_total_sources, f'{_total_sources}-way')
        lines.append(f'  - **Power supply**: {_level} redundant ({", ".join(_src_names)})')

    # --- Mechanical Data (always emitted; TODO when wizard data absent) ---
    dim_w = ow.get('width_mm')
    dim_l = ow.get('length_mm')
    dim_h = ow.get('height_mm')
    # Backward compat: parse old dimensions_mm string "WxLxH"
    if not any([dim_w, dim_l, dim_h]) and ow.get('dimensions_mm'):
        _parts = re.split(r'\s*[xX]\s*', ow['dimensions_mm'].strip())
        if len(_parts) >= 3:
            dim_w, dim_l, dim_h = _parts[0], _parts[1], _parts[2]
        elif len(_parts) == 2:
            dim_w, dim_l = _parts[0], _parts[1]
    weight = ow.get('weight_g')
    lines.append('- **Mechanical Data**')
    if any([dim_w, dim_l, dim_h]):
        _dim_parts = [str(p) if p else 'TODO' for p in [dim_w, dim_l, dim_h]]
        lines.append(f'  - **Dimensions**: {" \u00d7 ".join(_dim_parts)} (mm)')
    else:
        lines.append('  - **Dimensions**: TODO: dimensions (mm)')
    lines.append(f'  - **Weight**: {weight} (g)' if weight is not None else '  - **Weight**: TODO: weight (g)')

    # --- Other (conditional, omit section if nothing applies) ---
    other_lines = []
    if entry.get('has_sd_card'):
        other_lines.append('  - **SD card**: Yes')
    if entry.get('has_heater'):
        other_lines.append('  - **Onboard heater**: Yes')
    if other_lines:
        lines.append('- **Other**')
        lines.extend(other_lines)

    lines.append('')

    # Source data comment for traceability / future re-runs
    source_data = {
        'board': board_key,
        'chip_model': chip_model,
        'has_io_board': entry.get('has_io_board', False),
        'total_outputs': entry.get('total_outputs'),
        'fmu_servo_outputs': fmu_out,
        'io_outputs': io_out,
        'has_sd_card': entry.get('has_sd_card', False),
        'has_ethernet': entry.get('has_ethernet', False),
        'has_heater': entry.get('has_heater', False),
        'num_i2c_buses': entry.get('num_i2c_buses', 0),
        'num_spi_buses': entry.get('num_spi_buses', 0),
        'num_can_buses': entry.get('num_can_buses', 0),
        'has_usb': entry.get('has_usb', False),
        'sensor_drivers': {
            'imu':  entry.get('sensor_imu_drivers', []),
            'baro': entry.get('sensor_baro_drivers', []),
            'mag':  entry.get('sensor_mag_drivers', []),
            'osd':  entry.get('sensor_osd_drivers', []),
        },
        'overview_wizard': ow or None,
    }
    source_comment = (
        '<!-- overview-source-data\n'
        + json.dumps(source_data, indent=2)
        + '\n-->'
    )
    lines.append(source_comment)

    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Section generator registry
# Register each section: key -> generator function(board_key, entry) -> str
#
# SECTION_ORDER  — sections included in new stub pages (generate_full_template)
#                  and available via --section <key>
# APPLY_SECTIONS — sections applied by --apply (without --section).
#                  serial_ports is intentionally excluded so it is not pushed
#                  to existing docs automatically; use --section serial_ports
#                  to apply it explicitly.
# ---------------------------------------------------------------------------
SECTION_GENERATORS = {
    "specifications": generate_specifications_section,
    "power":          generate_power_section,
    "pwm_outputs":    generate_pwm_section,
    "radio_control":  generate_radio_control_section,
    "gps_compass":    generate_gps_section,
    "telemetry":      generate_telemetry_section,
    "sd_card":        generate_sd_card_section,
    "serial_ports":   generate_serial_section,
}

SECTION_ORDER = [
    "specifications", # first: overview/specs near the top of each doc
    "power",          # fundamental hardware connection
    "pwm_outputs",
    "radio_control",
    "gps_compass",     # after RC, before serial ports
    "telemetry",
    "sd_card",
    "serial_ports",
]

APPLY_SECTIONS = [
    "pwm_outputs",
    # specifications excluded — has TODO items for sensors; apply with --section specifications
    # power excluded — apply explicitly with --section power
    # radio_control excluded — apply explicitly with --section radio_control
    # gps_compass excluded — apply explicitly with --section gps_compass
    # telemetry excluded — apply explicitly with --section telemetry
    # sd_card excluded — apply explicitly with --section sd_card
]

SECTION_PATTERNS = {
    "specifications": [r'##\s+Specifications?', r'##\s+Key\s+Features',
                       r'##\s+Key\s+Design\s+Points', r'##\s+Quick\s+Summary',
                       r'##\s+Overview'],
    "power":          [r'##\s+Power\b', r'###\s+Power\b'],
    "pwm_outputs":    PWM_SECTION_PATTERNS,
    "radio_control":  [r'###\s+Radio\s+Control', r'##\s+Radio\s+Control'],
    "gps_compass":    [r'###\s+GPS\s*[&/]\s*Compass', r'###\s+GPS/Compass', r'###\s+GPS\b',
                       r'##\s+GPS\s*[&/]\s*Compass', r'##\s+GPS/Compass', r'##\s+GPS\b'],
    "telemetry":      [r'##\s+Telemetry\b', r'###\s+Telemetry\b'],
    "sd_card":        [r'##\s+SD\s+Card\b', r'###\s+SD\s+Card\b'],
    "serial_ports":   [r'##\s+Serial\s+Port\s+Mapping', r'##\s+Serial\s+Ports'],
}

# Section keys that belong under the ## Assembly {#assembly} parent heading
ASSEMBLY_SECTION_KEYS = {"radio_control", "gps_compass"}


# ---------------------------------------------------------------------------
# Section apply logic (insert/replace sections in existing FC doc files)
# ---------------------------------------------------------------------------
# Assembly sections go near the end, just before Further Information / Credits.
_ASSEMBLY_INSERT_BEFORE = [
    r'## Further [Ii]nfo',
    r'## Further Information',
    r'## Credits',
]

_INSERT_BEFORE = [
    r'## Where to Buy',
    r'## Serial Port Mapping',
    r'## Serial Ports',
    r'## Pinouts?',
    r'## Debug Port',
    r'## Dimensions',
    r'## Voltage Ratings',
    r'## Building Firmware',
    r'## Further info',
    r'## Further Information',
    r'## Credits',
]


def _find_section_end(text: str, start: int, level: int = 2) -> int:
    """Return the position where the section ends (start of the next same-or-higher heading)."""
    if level >= 3:
        # Stop at ## or ### (any heading at H2 or H3 level)
        pattern = r'\n(?:#{2,3})(?!#)\s+'
    else:
        # Stop at ## only
        pattern = r'\n##(?!#)\s+'
    m = re.search(pattern, text[start + 1:])
    if m:
        return start + 1 + m.start()
    return len(text)


def _apply_section(doc_text: str, proposed: str, section_key: str) -> str:
    patterns = SECTION_PATTERNS.get(section_key, [])
    # Check if an existing section matching this key is present
    for pattern in patterns:
        m = re.search(pattern, doc_text, re.IGNORECASE)
        if m:
            start = m.start()
            hm = re.match(r'^(#+)', doc_text[start:])
            level = len(hm.group(1)) if hm else 2
            end = _find_section_end(doc_text, start, level)
            if section_key in ASSEMBLY_SECTION_KEYS and level == 2:
                # Old ## heading — remove it and fall through to Assembly insertion
                doc_text = doc_text[:start] + doc_text[end:]
                break
            elif section_key == 'specifications':
                # Special behaviour: preserve hand-written content.
                # 1. Inject {#specifications} anchor into heading if absent.
                # 2. Append generated content as a comment after the section body
                #    so authors can cherry-pick from it without it rendering.
                heading_line_end = doc_text.index('\n', start)
                heading_line = doc_text[start:heading_line_end]
                if '{#specifications}' not in heading_line:
                    # Strip any existing anchor and append the correct one
                    heading_line = re.sub(r'\s*\{#[^}]+\}', '', heading_line).rstrip()
                    heading_line += ' {#specifications}'
                    doc_text = doc_text[:start] + heading_line + doc_text[heading_line_end:]
                    # Recalculate end after heading edit (length may have changed)
                    end = _find_section_end(doc_text, start, level)
                # Append the proposed section as a comment after the existing body.
                # Strip the inner <!-- overview-source-data ... --> block to avoid
                # nested comments (which HTML parsers terminate at the first -->).
                proposed_for_comment = re.sub(
                    r'\n*<!-- overview-source-data[\s\S]*?-->', '', proposed
                ).rstrip()
                comment = (
                    '\n<!-- specifications-proposed\n'
                    + proposed_for_comment
                    + '\n-->'
                )
                existing_body_end = doc_text[:end].rstrip('\n')
                return existing_body_end + comment + '\n' + doc_text[end:]
            else:
                # ### heading (or non-assembly section) — direct replacement
                return doc_text[:start] + proposed + '\n' + doc_text[end:]

    # Assembly sections: find or create ## Assembly {#assembly}
    if section_key in ASSEMBLY_SECTION_KEYS:
        assembly_m = re.search(r'^##\s+Assembly', doc_text, re.MULTILINE | re.IGNORECASE)
        if assembly_m:
            assembly_end = _find_section_end(doc_text, assembly_m.start(), 2)
            return doc_text[:assembly_end].rstrip('\n') + '\n\n' + proposed + '\n' + doc_text[assembly_end:]
        else:
            assembly_block = '## Assembly {#assembly}\n\n' + proposed + '\n'
            for insert_pattern in _ASSEMBLY_INSERT_BEFORE:
                im = re.search(insert_pattern, doc_text, re.IGNORECASE)
                if im:
                    return doc_text[:im.start()] + assembly_block + '\n\n' + doc_text[im.start():]
            return doc_text.rstrip('\n') + '\n\n' + assembly_block + '\n'

    # Non-assembly section — insert before the first matching anchor heading
    for insert_pattern in _INSERT_BEFORE:
        m = re.search(insert_pattern, doc_text, re.IGNORECASE)
        if m:
            return doc_text[:m.start()] + proposed + '\n\n' + doc_text[m.start():]
    return doc_text.rstrip('\n') + '\n\n' + proposed + '\n'


def _has_no_rc_data(entry: dict) -> bool:
    """True when no RC configuration was found for this board."""
    return (
        not entry.get('has_rc_input')
        and not entry.get('has_common_rc')
        and not entry.get('has_io_board')
    )


def apply_sections_to_docs(data: dict, sections: list = None, doc_filter: str = None) -> tuple:
    """Apply generated sections to FC doc files. Returns (updated, skipped) lists.

    doc_filter: if given, only process the board whose doc_file matches this
    filename (e.g. 'cuav_x25-evo.md').  Stem-only ('cuav_x25-evo') also accepted.
    """
    sections_to_apply = sections or APPLY_SECTIONS
    updated, skipped = [], []

    # Normalise filter to bare filename with extension
    if doc_filter:
        p = Path(doc_filter)
        doc_filter_name = p.name if p.suffix else p.name + ".md"

    for key, entry in data.items():
        doc_filename = entry.get('doc_file')
        if not doc_filename:
            skipped.append((key, 'no doc mapping'))
            continue
        if doc_filter and doc_filename != doc_filter_name:
            skipped.append((key, f'filtered (not {doc_filter_name})'))
            continue
        doc_path = FC_DOCS / doc_filename
        if not doc_path.exists():
            skipped.append((key, f'doc not found: {doc_filename}'))
            continue

        doc_text = doc_path.read_text(encoding='utf-8')
        # Read wizard overrides embedded in the doc (port labels, connector types, etc.)
        # so section generators use them even without the wizard.json cache file.
        wizard_doc_data = _read_wizard_data_from_doc(doc_path)
        if wizard_doc_data:
            entry = dict(entry)  # shallow copy — don't mutate shared data
            for _wf in ('rc_ports_wizard', 'gps_ports_wizard',
                        'power_ports_wizard', 'overview_wizard', 'i2c_buses_wizard'):
                if wizard_doc_data.get(_wf) is not None:
                    entry[_wf] = wizard_doc_data[_wf]

        new_text = doc_text
        for section_key in sections_to_apply:
            generator = SECTION_GENERATORS.get(section_key)
            if not generator:
                continue
            # Per-section skip guards
            if section_key == 'pwm_outputs' and not entry.get('groups'):
                continue
            if section_key == 'radio_control' and _has_no_rc_data(entry):
                continue
            proposed = generator(key, entry)
            new_text = _apply_section(new_text, proposed, section_key)

        if new_text == doc_text:
            skipped.append((key, 'no change'))
            continue

        doc_path.write_text(new_text, encoding='utf-8')
        updated.append((key, doc_filename, 'updated'))
        print(f'  [updated] {key} -> {doc_filename}')

    return updated, skipped


# ---------------------------------------------------------------------------
# Full template page generator
# ---------------------------------------------------------------------------
DEFAULT_SINCE_VERSION = "vX.XX"


def _wizard_prompt(label: str, default: str = "", required: bool = False) -> str:
    """Interactive prompt; returns entered value or default on empty input."""
    hint = f" [{default}]" if default else (" (required)" if required else " (optional, press Enter to skip)")
    while True:
        try:
            val = input(f"  {label}{hint}: ").strip()
            # Strip lone surrogate code points that can result from terminal encoding issues
            val = ''.join(c for c in val if not ('\ud800' <= c <= '\udfff'))
        except EOFError:
            return default
        if val:
            return val
        if default:
            return default
        if not required:
            return ""
        print("    (this field is required)")


_WIZARD_CACHE_FIELDS = (
    "manufacturer", "product", "board", "fmu_version", "since_version",
    "manufacturer_url", "rc_ports_wizard", "gps_ports_wizard", "power_ports_wizard",
    "overview_wizard", "i2c_buses_wizard",
)


def _wizard_cache_key(manufacturer: str, product: str) -> str:
    """Return the stem used for the wizard cache filename (manufacturer+product slug)."""
    return f"{_make_slug(manufacturer)}_{_make_slug(product)}"


def _wizard_cache_path(manufacturer: str, product: str, meta_dir: Path = None) -> Path:
    stem = _wizard_cache_key(manufacturer, product)
    return (meta_dir or METADATA_DIR) / f"{stem}_wizard.json"


def _load_wizard_cache(manufacturer: str, product: str,
                       meta_dir: Path = None) -> dict | None:
    path = _wizard_cache_path(manufacturer, product, meta_dir)
    if path.exists():
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None
    return None


def _save_wizard_cache(manufacturer: str, product: str,
                       args=None, meta_dir: Path = None) -> None:
    path = _wizard_cache_path(manufacturer, product, meta_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {f: getattr(args, f, None) for f in _WIZARD_CACHE_FIELDS}
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def _embed_wizard_data_comment(content: str, wizard_data: dict) -> str:
    """Append a <!-- wizard-data {JSON} --> comment at the end of the document."""
    comment = '<!-- wizard-data\n' + json.dumps(wizard_data, indent=2) + '\n-->\n'
    return content.rstrip('\n') + '\n\n' + comment


def _collect_and_strip_source_comments(content: str) -> tuple[str, str]:
    """Extract all <!-- *-source-data … --> blocks from the document body.

    Returns (stripped_content, comments_block) so callers can append the
    collected comments at the end of the document rather than inline.
    Checklist comments (<!-- checklist … -->) are left in place.
    """
    pattern = re.compile(r'<!-- [a-z-]+-source-data\n.*?-->\n?', re.DOTALL)
    blocks = pattern.findall(content)
    stripped = pattern.sub('', content)
    # Collapse triple-or-more blank lines left behind by removal
    stripped = re.sub(r'\n{3,}', '\n\n', stripped)
    comments_block = '\n'.join(b.rstrip('\n') for b in blocks)
    return stripped, comments_block


def _read_wizard_data_from_doc(doc_path: Path) -> dict | None:
    """Extract wizard data from a <!-- wizard-data ... --> comment in a .md file."""
    try:
        text = doc_path.read_text(encoding='utf-8')
        m = re.search(r'<!--\s*wizard-data\s*\n(.*?)\n-->', text, re.DOTALL)
        if m:
            return json.loads(m.group(1))
    except (OSError, json.JSONDecodeError):
        pass
    return None


def _run_wizard(args) -> None:
    """Interactively fill in missing --new-doc values on args in-place."""
    print("\n=== New FC Doc Wizard ===\n")

    if not args.manufacturer:
        args.manufacturer = _wizard_prompt("Manufacturer name", required=True)
    else:
        print(f"  Manufacturer: {args.manufacturer}")

    if not args.product:
        # Suggest the most-recently cached product for this manufacturer
        _mfr_slug = _make_slug(args.manufacturer)
        _prior = sorted(METADATA_DIR.glob(f"{_mfr_slug}_*_wizard.json"),
                        key=lambda p: p.stat().st_mtime, reverse=True)
        _product_default = ""
        for _c in _prior:
            try:
                _cd = json.loads(_c.read_text(encoding="utf-8"))
                if _cd.get("product"):
                    _product_default = _cd["product"]
                    break
            except Exception:
                pass
        args.product = _wizard_prompt("Product name", required=True,
                                      default=_product_default)
    else:
        print(f"  Product: {args.product}")

    # Load cached wizard data right after manufacturer+product are known.
    # Values from the cache are used as defaults for each prompt below — the user
    # can accept by pressing Enter or type a new value to override.
    cached = _load_wizard_cache(args.manufacturer, args.product) or {}
    if cached:
        print("  (Cached wizard data found — used as defaults below)")

    if not args.board:
        suggested = _find_board_key(args.manufacturer, args.product)
        val = _wizard_prompt("Board key (e.g. holybro/kakuteh7 or px4/fmu-v6x)",
                             default=suggested or cached.get('board') or "")
        args.board = val or None
    else:
        print(f"  Board: {args.board}")

    # FMU version: infer from board name when it is a px4 reference design;
    # ask only for manufacturer-specific boards where the version is a separate concept.
    if not args.fmu_version:
        if args.board and args.board.startswith('px4/'):
            args.fmu_version = args.board.split('/', 1)[1]   # "px4/fmu-v6x" → "fmu-v6x"
            print(f"  FMU version: {args.fmu_version} (inferred from px4 board)")
        else:
            val = _wizard_prompt("FMU version suffix (e.g. fmu-v6x, optional)",
                                 default=cached.get('fmu_version') or '')
            args.fmu_version = val or None
    else:
        print(f"  FMU version: {args.fmu_version}")

    args.since_version = _wizard_prompt("PX4 version introduced in",
                                        default=cached.get('since_version') or args.since_version)

    # RC port labels — load board data once for hints and capability detection
    _rc_entry = {}
    if args.board:
        try:
            _rc_entry = gather_board_data().get(args.board, {})
        except Exception:
            pass

    _has_ppm = _rc_entry.get('has_ppm_pin', False)
    _ppm_shared = _rc_entry.get('ppm_shared_with_rc_serial', False)
    _serial_hint = _derive_rc_label(_rc_entry)
    _serial_side = _infer_rc_side(_rc_entry)

    print()
    print("  RC capabilities detected from source:")
    _driver = ('rc_input' if _rc_entry.get('has_rc_input')
               else 'common_rc' if _rc_entry.get('has_common_rc') else 'none')
    print(f"    Serial RC driver : {_driver}")
    _ppm_desc = ('yes — shared with serial UART (one connector)' if _ppm_shared
                 else 'yes — dedicated hardware pin' if _has_ppm else 'no')
    print(f"    PPM hardware pin : {_ppm_desc}")
    if _rc_serial_protocols(_rc_entry):
        print(f"    Serial protocols : {', '.join(_rc_serial_protocols(_rc_entry))}")

    # Prepare cached RC defaults — non-PPM entry is serial, PPM-only is the PPM port
    _cached_rc = cached.get('rc_ports_wizard') or []
    _cached_serial = next((p for p in _cached_rc if not p.get('ppm_only')), {})
    _cached_ppm    = next((p for p in _cached_rc if p.get('ppm_only')), {})

    print()
    serial_label = _wizard_prompt(
        "Serial RC port label as printed on board",
        default=_cached_serial.get('label') or _serial_hint,
    ) or 'TODO: RC port label'

    rc_ports = [{'label': serial_label, 'side': _serial_side}]

    # Dedicated (non-shared) PPM pin — may be on a separate physical connector
    if _has_ppm and not _ppm_shared:
        sep_raw = _wizard_prompt(
            "PPM has a dedicated pin. Is it on a separate connector from the serial RC port?",
            default='y',
        )
        if sep_raw.strip().lower() in ('y', 'yes', ''):
            _ppm_side = 'IO' if _rc_entry.get('has_io_board') else 'FMU'
            ppm_label = _wizard_prompt("PPM port label as printed on board",
                                       default=_cached_ppm.get('label') or 'PPM')
            rc_ports.append({'label': ppm_label or 'TODO: PPM port label',
                             'side': _ppm_side, 'ppm_only': True})

    args.rc_ports_wizard = rc_ports

    # --- Power ---
    _num_inputs = _rc_entry.get('num_power_inputs', 1)
    _has_redundant = _rc_entry.get('has_redundant_power', False)
    _has_dronecan = _rc_entry.get('has_dronecan_power_input', False)
    _pattern_a = _has_dronecan and _rc_entry.get('num_power_inputs', 1) >= 2

    print()
    print("  Power capabilities detected from source:")
    print(f"    Power inputs     : {_num_inputs}"
          f"{' (redundant)' if _has_redundant else ''}")
    if _rc_entry.get('power_monitor_type'):
        print(f"    Monitor type     : {_rc_entry['power_monitor_type']}")
    if _has_dronecan:
        print("    DroneCAN power   : detected")

    print()
    _cached_power = cached.get('power_ports_wizard') or []
    power_ports_wizard = []
    for i in range(_num_inputs):
        cp = _cached_power[i] if i < len(_cached_power) else {}
        default_label = cp.get('label') or (f'POWER {i + 1}' if _num_inputs > 1 else 'POWER')
        label = _wizard_prompt(
            f"  POWER port {i + 1} label as printed on board" if _num_inputs > 1
            else "  POWER port label as printed on board",
            default=default_label,
        ) or default_label
        connector = _wizard_prompt(
            f"  Connector type for {label} (e.g. 6-pin JST GH, 6-pin Molex CLIK-Mate)",
            default=cp.get('connector_type') or '',
        )
        port = {
            'label': label,
            'connector_type': connector or 'TODO: connector type',
        }
        # When DroneCAN is detected, prompt for per-port monitor type
        if _has_dronecan:
            # Default: Pattern A → brick 1=analog, brick 2=dronecan; Pattern B → dronecan
            if _pattern_a:
                default_mtype = 'analog' if i == 0 else 'dronecan'
            else:
                default_mtype = 'dronecan'
            mtype = _wizard_prompt(
                f"  Monitor type for {label} (analog/i2c/dronecan)",
                default=cp.get('monitor_type') or default_mtype,
            ) or default_mtype
            port['monitor_type'] = mtype.strip().lower()
        power_ports_wizard.append(port)

    args.power_ports_wizard = power_ports_wizard

    # --- GPS & Compass ---
    _gps_serial = _gps_ports_from_entry(_rc_entry)
    _has_safety = _rc_entry.get('has_safety_switch', False)

    if _gps_serial:
        print()
        print("  GPS capabilities detected from source:")
        print(f"    GPS ports        : {', '.join(p['label'] for p in _gps_serial)}")
        print(f"    PPS capture      : {'yes' if _rc_entry.get('has_pps_capture') else 'no'}")
        print(f"    Safety switch HW : {'yes' if _has_safety else 'no'}")
        print()

        _cached_gps = cached.get('gps_ports_wizard') or []
        gps_ports_wizard = []
        for gps_port in _gps_serial:
            port_key = gps_port['label']   # e.g. 'GPS1', 'GPS2'
            cg = next((g for g in _cached_gps if g.get('port_key') == port_key), {})
            label = _wizard_prompt(
                f"  {port_key} label as printed on board",
                default=cg.get('label') or port_key,
            ) or port_key

            if port_key == 'GPS1' and _has_safety:
                # Safety switch hardware present — ask if it lives on the GPS1 connector
                raw = _wizard_prompt(
                    f"  Safety switch detected. Is it on the {port_key} connector\n"
                    f"  (i.e. a Pixhawk 10-pin full GPS port with GPS + compass +\n"
                    f"  safety switch + buzzer + LED), or a separate port?",
                    default='y' if cg.get('full_port', True) else 'n',
                )
                if raw.strip().lower() in ('y', 'yes', ''):
                    gps_ports_wizard.append({'port_key': port_key, 'label': label,
                                             'pixhawk_standard': True, 'full_port': True})
                else:
                    # Safety switch is on a separate connector; GPS1 is a basic port
                    raw2 = _wizard_prompt(
                        f"  Is {port_key} a Pixhawk Connector Standard 6-pin basic port?",
                        default='y' if cg.get('pixhawk_standard', True) else 'n',
                    )
                    is_std = raw2.strip().lower() in ('y', 'yes', '')
                    gps_ports_wizard.append({'port_key': port_key, 'label': label,
                                             'pixhawk_standard': is_std, 'full_port': False})
            else:
                # No safety switch hardware (or GPS2/GPS3) — ask if Pixhawk 6-pin basic
                raw = _wizard_prompt(
                    f"  Is {port_key} a Pixhawk Connector Standard port (6-pin basic)?",
                    default='y' if cg.get('pixhawk_standard', True) else 'n',
                )
                is_std = raw.strip().lower() in ('y', 'yes', '')
                gps_ports_wizard.append({'port_key': port_key, 'label': label,
                                         'pixhawk_standard': is_std, 'full_port': False})

        args.gps_ports_wizard = gps_ports_wizard
    else:
        args.gps_ports_wizard = None

    # --- I2C external bus labels ---
    _bus_info_raw = _rc_entry.get('sensor_bus_info') or {}
    _ext_buses = {}       # bus_num → [(cat, name)] for buses with detected external sensors
    _detected_labels = {} # bus_num → port_label auto-detected from source comments
    for _cat, _entries in _bus_info_raw.items():
        for _e in _entries:
            _bn = _e.get('bus_num')
            if _e.get('bus_type') == 'I2C' and _e.get('external') and _bn is not None:
                _ext_buses.setdefault(_bn, []).append((_cat, _e['name']))
            if _e.get('port_label') and _bn is not None and _bn not in _detected_labels:
                _detected_labels[_bn] = _e['port_label']

    _STANDARD_I2C_LABELS = ['GPS1', 'GPS2', 'I2C', 'I2C1', 'I2C2', 'POWER']
    _cached_i2c = {b['bus_num']: b['label'] for b in (cached.get('i2c_buses_wizard') or [])}
    _num_i2c = _rc_entry.get('num_i2c_buses', 0)
    # Load authoritative bus routing from i2c.cpp if available in cached entry or board path
    _wiz_i2c_cfg = _rc_entry.get('i2c_bus_config') or {}
    if not _wiz_i2c_cfg and args.board:
        _wiz_i2c_cfg = parse_i2c_bus_config(BOARDS / args.board)
    _wiz_ext_buses_cfg = set(_wiz_i2c_cfg.get('external', []))
    _wiz_int_buses_cfg = set(_wiz_i2c_cfg.get('internal', []))

    i2c_buses_wizard = []
    # Ask about all enabled I2C buses (not just sensor-bearing ones) so standalone
    # external ports (e.g. I2C3, I2C4) can be labelled even without detected sensors.
    _buses_to_ask = sorted(set(
        list(_ext_buses) + list(_cached_i2c) +
        list(_wiz_ext_buses_cfg) + list(_wiz_int_buses_cfg) +
        (list(range(1, _num_i2c + 1)) if _num_i2c > 0 else [])
    ))
    if _buses_to_ask:
        print()
        if _ext_buses:
            print("  External I2C buses detected from source:")
            for _bn in sorted(_ext_buses):
                _sensor_desc = ', '.join(_nm for _, _nm in _ext_buses[_bn])
                _pl = _detected_labels.get(_bn)
                _pl_str = f' [detected port: {_pl}]' if _pl else ''
                print(f"    I2C bus {_bn}: {_sensor_desc}{_pl_str}")
        if _wiz_i2c_cfg:
            if _wiz_ext_buses_cfg:
                print(f"  External buses (from src/i2c.cpp): {sorted(_wiz_ext_buses_cfg)}")
            if _wiz_int_buses_cfg:
                print(f"  Internal buses — no external connector: {sorted(_wiz_int_buses_cfg)}")
        elif _num_i2c > 0:
            print(f"  I2C buses enabled: 1–{_num_i2c}")
        print(f"  Common port labels: {', '.join(_STANDARD_I2C_LABELS)}")
        print()
        for _bn in _buses_to_ask:
            _cached_label = _cached_i2c.get(_bn, '')
            _detected_label = _detected_labels.get(_bn, '')
            _default = _cached_label or _detected_label or ''
            _hint = f' [detected: {_detected_label}]' if _detected_label and not _cached_label else ''
            _sensor_info = ', '.join(_nm for _, _nm in _ext_buses.get(_bn, []))
            _sensor_hint = f' (sensor: {_sensor_info})' if _sensor_info else ''
            if _bn in _wiz_int_buses_cfg:
                _routing_note = ' [internal bus — no external connector, Enter to skip]'
            elif _bn in _wiz_ext_buses_cfg and not _sensor_info:
                _routing_note = ' [external — may be a free connector]'
            else:
                _routing_note = ''
            _label = _wizard_prompt(
                f"  Label for I2C bus {_bn} port as printed on board{_sensor_hint}{_hint}{_routing_note}",
                default=_default
            )
            if _label:
                i2c_buses_wizard.append({'bus_num': _bn, 'label': _label})

    args.i2c_buses_wizard = i2c_buses_wizard or None

    # --- Sensors & Physical Specs (for ## Key Features section) ---
    print()
    print("  === Sensors & Physical Specs ===")

    # Show which sensor drivers were auto-detected from source
    # Use both parsers (default.px4board + rc.board_sensors) to match gather_board_data()
    if args.board:
        _board_path = BOARDS / args.board
        _sensor_cfg = parse_sensor_config(_board_path)
        _rc_sensor_cfg = parse_rc_board_sensors(_board_path)
        for _cat in ('imu', 'baro', 'mag', 'osd'):
            for _chip in _rc_sensor_cfg.get(_cat, []):
                if _chip not in _sensor_cfg.setdefault(_cat, []):
                    _sensor_cfg[_cat].append(_chip)
    else:
        _sensor_cfg = {}
    _det_imu  = _sensor_cfg.get('imu', [])
    _det_baro = _sensor_cfg.get('baro', [])
    _det_mag  = _sensor_cfg.get('mag', [])
    _det_osd  = _sensor_cfg.get('osd', [])

    if any([_det_imu, _det_baro, _det_mag, _det_osd]):
        print()
        print("  Sensors detected from source drivers:")
        if _det_imu:  print(f"    IMU         : {', '.join(_det_imu)}")
        if _det_baro: print(f"    Barometer   : {', '.join(_det_baro)}")
        if _det_mag:  print(f"    Magnetometer: {', '.join(_det_mag)}")
        if _det_osd:  print(f"    OSD         : {', '.join(_det_osd)}")
        print("  (Multiple drivers = hardware revision variants; confirm which is installed)")

    # --- Sensor hardware variants ---
    _sv_info = _rc_entry.get('sensor_variant_info') or {}
    _sv_has_variants = _sv_info.get('has_variants', False)
    _sv_variants = _sv_info.get('variants') or {}
    _CAT_DISPLAY = {'imu': 'IMU', 'baro': 'barometer', 'mag': 'magnetometer', 'osd': 'OSD'}

    cached_ov = cached.get('overview_wizard') or {}
    _cached_variant_labels = cached_ov.get('sensor_variant_labels') or {}

    variant_labels: dict[str, str] = {}
    if _sv_has_variants and _sv_variants:
        print()
        print("  Hardware sensor variants detected:")
        for _code, _vdata in _sv_variants.items():
            _display_code = 'other variants' if _code == '__other__' else _code
            _sensor_parts = []
            for _cat, _vent in _vdata.items():
                for _ve in _vent:
                    _sensor_parts.append(f'{_ve["name"]} ({_CAT_DISPLAY.get(_cat, _cat)})')
            print(f"    {_display_code}: {', '.join(_sensor_parts) or '(none)'}")
        print()
        print("  Enter optional labels for each variant (press Enter to skip):")
        for _code in _sv_variants:
            _display_code = 'other variants' if _code == '__other__' else _code
            _lbl = _wizard_prompt(
                f"  Label for '{_display_code}' (e.g. 'Rev 1.0', 'Standard')",
                default=_cached_variant_labels.get(_code) or "",
            )
            if _lbl:
                variant_labels[_code] = _lbl

    print()

    def _wizard_list(prompt_label: str, detected: list = None,
                     cached_list: list = None, max_items: int = None,
                     item_hints: list = None) -> list:
        """Prompt for a multi-value list; blank entry ends input.

        Displays detected drivers as a numbered menu (1-N). Each prompt
        defaults to the corresponding cached value so the user can accept
        previous selections by pressing Enter.

        Entering 0 stops immediately without applying any cache fallback,
        letting the user clear previously entered items.

        max_items: stop after this many items (no trailing open-ended prompt).
        item_hints: per-item hint strings shown in parentheses before the default.
        """
        items = []
        explicitly_done = False
        print(f"  {prompt_label} (one per line, blank to finish):")
        if detected:
            print(f"    {0:2}. (done — stop here / clear remaining)")
            for i, name in enumerate(detected, 1):
                print(f"    {i:2}. {name}")
            print("    (enter a number, a custom name, or press Enter to accept default)")
        elif cached_list:
            print("     0. (done — stop here / clear remaining)")
            print("    (press Enter to accept each default, or 0 to clear all)")
        idx = 1
        while True:
            # Stop once we've reached the maximum allowed items
            if max_items is not None and idx > max_items:
                break
            # Show the corresponding cached entry as the default for this position
            cached_default = str(cached_list[idx - 1]) if cached_list and idx <= len(cached_list) else ""
            hint = item_hints[idx - 1] if item_hints and idx <= len(item_hints) else None
            label = f"    item {idx} ({hint})" if hint else f"    item {idx}"
            val = _wizard_prompt(label, default=cached_default)
            # 0 = explicit stop (skips cache fallback)
            if val == "0":
                explicitly_done = True
                break
            if not val:
                break
            # Resolve numeric entry to detected name
            if detected and val.strip().isdigit():
                num = int(val.strip())
                if 1 <= num <= len(detected):
                    val = detected[num - 1]
                    print(f"      → {val}")
            items.append(val)
            idx += 1
        # Safety fallback: only reached when no items were entered AND no defaults
        # fired (e.g. piped/non-interactive input with no cached defaults available).
        if not items and not explicitly_done and cached_list:
            items = list(cached_list)
            print(f"    (using cached: {', '.join(str(v) for v in items)})")
        return items

    imu_list  = _wizard_list("IMU(s)", _det_imu, cached_list=cached_ov.get('imu'))
    baro_list = _wizard_list("Barometer(s)", _det_baro, cached_list=cached_ov.get('baro'))
    mag_list  = _wizard_list("Magnetometer/Compass(es) (optional)", _det_mag,
                             cached_list=cached_ov.get('mag'))
    osd_default = _det_osd[0] if len(_det_osd) == 1 else ""
    osd       = _wizard_prompt("OSD chip (optional)",
                               default=cached_ov.get('osd') or osd_default)
    # Pre-populate from cached split fields, or parse old dimensions_mm string
    _dims_cached = cached_ov.get('dimensions_mm') or ''
    _dims_parts = re.split(r'\s*[xX]\s*', _dims_cached.strip()) if _dims_cached else []
    dim_w     = _wizard_prompt("Width (mm)",
                               default=str(cached_ov.get('width_mm') or '')
                                       or (_dims_parts[0] if len(_dims_parts) > 0 else ''))
    dim_l     = _wizard_prompt("Length (mm)",
                               default=str(cached_ov.get('length_mm') or '')
                                       or (_dims_parts[1] if len(_dims_parts) > 1 else ''))
    dim_h     = _wizard_prompt("Height (mm)",
                               default=str(cached_ov.get('height_mm') or '')
                                       or (_dims_parts[2] if len(_dims_parts) > 2 else ''))
    weight_raw = _wizard_prompt("Weight (g)",
                                default=str(cached_ov.get('weight_g') or ''))
    min_v     = _wizard_prompt("Minimum input voltage (V)",
                               default=cached_ov.get('min_voltage') or '')
    max_v     = _wizard_prompt("Maximum input voltage (V)",
                               default=cached_ov.get('max_voltage') or '')
    _usb_pwr_cached = 'y' if cached_ov.get('usb_powers_fc') else 'n'
    _usb_pwr_raw  = _wizard_prompt("Can USB power the flight controller? (y/n)",
                                   default=_usb_pwr_cached)
    usb_powers_fc = _usb_pwr_raw.strip().lower() in ('y', 'yes')
    if usb_powers_fc:
        usb_pwr_min = _wizard_prompt("USB minimum power voltage (V)",
                                     default=cached_ov.get('usb_pwr_min_v') or '4.75')
        usb_pwr_max = _wizard_prompt("USB maximum power voltage (V)",
                                     default=cached_ov.get('usb_pwr_max_v') or '5.25')
    else:
        usb_pwr_min = None
        usb_pwr_max = None
    _servo_cached = 'y' if cached_ov.get('has_servo_rail', True) else 'n'
    _servo_raw    = _wizard_prompt("Does the board have a servo/output rail? (y/n)",
                                   default=_servo_cached)
    has_servo_rail = _servo_raw.strip().lower() in ('y', 'yes')
    if has_servo_rail:
        servo_rail_max_v = _wizard_prompt("Maximum servo rail voltage (V)",
                                          default=cached_ov.get('servo_rail_max_v') or '')
    else:
        servo_rail_max_v = None
    _usb_cached = (cached_ov.get('usb_connectors')
                   or ([cached_ov['usb_connector']] if cached_ov.get('usb_connector') else None))
    usb_list  = _wizard_list("USB connector type(s) (e.g. USB-C, Micro-USB)",
                              [], cached_list=_usb_cached)
    _usb_labels_default = (cached_ov.get('usb_labels')
                           or (['USB'] * len(usb_list) if usb_list else ['USB']))
    usb_labels = _wizard_list("USB port label(s) (e.g. USB, USB2)",
                               [], cached_list=_usb_labels_default,
                               max_items=len(usb_list) if usb_list else None,
                               item_hints=usb_list or None)
    adc_raw   = _wizard_prompt("Additional analog inputs beyond battery monitoring (optional, integer)",
                               default=str(cached_ov.get('num_additional_adc_inputs') or ''))

    weight_g = None
    if weight_raw:
        try:
            weight_g = float(weight_raw)
        except ValueError:
            weight_g = weight_raw   # keep as string if not numeric

    num_additional_adc = None
    if adc_raw:
        try:
            num_additional_adc = int(adc_raw)
        except ValueError:
            pass   # ignore non-integer input

    args.overview_wizard = {
        'imu':                       imu_list or None,
        'baro':                      baro_list or None,
        'mag':                       mag_list or None,
        'osd':                       osd or None,
        'width_mm':                  dim_w or None,
        'length_mm':                 dim_l or None,
        'height_mm':                 dim_h or None,
        'weight_g':                  weight_g,
        'min_voltage':               min_v or None,
        'max_voltage':               max_v or None,
        'usb_powers_fc':             usb_powers_fc,
        'usb_pwr_min_v':             usb_pwr_min or None,
        'usb_pwr_max_v':             usb_pwr_max or None,
        'has_servo_rail':            has_servo_rail,
        'servo_rail_max_v':          servo_rail_max_v or None,
        'usb_connectors':            usb_list or None,
        'usb_labels':                usb_labels or None,
        'num_additional_adc_inputs': num_additional_adc,
        'sensor_variant_labels':     variant_labels or None,
    }

    _url_hint = (args.manufacturer_url
                 or cached.get('manufacturer_url')
                 or _find_manufacturer_url(args.manufacturer)
                 or "TODO")
    if _url_hint != "TODO":
        print(f"  (Manufacturer URL inferred from existing docs: {_url_hint})")
    while True:
        url = _wizard_prompt("Manufacturer URL", default=_url_hint)
        if url == "TODO" or url.startswith("https://"):
            args.manufacturer_url = url
            break
        if url.startswith("http://"):
            print("    Warning: HTTP URL entered; HTTPS is strongly recommended.")
            args.manufacturer_url = url
            break
        print("    URL must start with https:// or http://")

    # Save non-parseable wizard data to metadata for future re-use
    _save_wizard_cache(args.manufacturer, args.product, args)
    cache_stem = _wizard_cache_key(args.manufacturer, args.product)
    print(f"  (Wizard data cached → {cache_stem}_wizard.json)")

    print()


def generate_full_template(board_key: str, entry: dict,
                           manufacturer: str = None, product: str = None,
                           since_version: str = DEFAULT_SINCE_VERSION,
                           manufacturer_url: str = None,
                           doc_name: str = None) -> str:
    """
    Generate a complete stub FC documentation page for a board.
    Boilerplate sections contain placeholder text; auto-generated sections
    are filled in from registered SECTION_GENERATORS.

    If manufacturer/product are provided they are used for the H1 title and
    manufacturer warning link text. Otherwise the title is derived from board_key.
    When board_key is None (no board data), the make target line is omitted.
    """
    if board_key and board_key != "unknown/unknown" and "/" in board_key:
        vendor, board = board_key.split("/", 1)
        make_target = f"make {vendor}_{board}_default"
    else:
        vendor, board = None, None
        make_target = None

    if manufacturer and product:
        title = f"{manufacturer} {product}"
    elif board:
        title = board.replace("-", " ").replace("_", " ").title()
    else:
        title = "TODO Flight Controller"

    mfr_display = manufacturer if manufacturer else (vendor.title() if vendor else "TODO")
    mfr_url = manufacturer_url or "TODO"

    # doc_name is the target markdown filename stem (no .md) used for asset paths.
    # Callers that know the exact filename should pass it explicitly; otherwise
    # derive from manufacturer + product as a reasonable fallback.
    if doc_name is None:
        if manufacturer and product:
            doc_name = _doc_filename(manufacturer, product).removesuffix('.md')
        else:
            doc_name = 'TODO'

    lines = [
        f"# {title}",
        "",
        f'<Badge type="tip" text="PX4 {since_version}" />',
        "",
        ":::warning",
        "PX4 does not manufacture this (or any) autopilot.",
        f"Contact the [{mfr_display}]({mfr_url}) for hardware support or compliance issues.",
        ":::",
        "",
        "TODO: brief description of the flight controller.",
        "",
        f"![{title}](../../assets/flight_controller/{doc_name}/{doc_name}_hero.jpg)",
        "",
        "## Where to Buy {#store}",
        "",
        "TODO: link to purchase page.",
        "",
        "## Pinouts",
        "",
        "TODO: connector pinout tables.",
        "",
    ]

    # Insert non-assembly auto-generated sections (Specifications, PWM, Serial, etc.)
    # specifications is inserted before "## Where to Buy" by the _INSERT_BEFORE logic,
    # but for the template we prepend it directly so it appears at the top.
    _spec_gen = SECTION_GENERATORS.get("specifications")
    if _spec_gen:
        lines.insert(lines.index("## Where to Buy {#store}"),
                     _spec_gen(board_key or "unknown/unknown", entry))
        lines.insert(lines.index("## Where to Buy {#store}"), "")

    for section_key in SECTION_ORDER:
        generator = SECTION_GENERATORS.get(section_key)
        if not generator or section_key in ASSEMBLY_SECTION_KEYS:
            continue
        if section_key == "specifications":
            continue   # already inserted above
        lines.append(generator(board_key or "unknown/unknown", entry))
        lines.append("")

    lines += [
        "## Building Firmware",
        "",
        ":::tip",
        "Most users will not need to build this firmware!",
        "It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.",
        ":::",
        "",
        "To [build PX4](../dev_setup/building_px4.md) for this target:",
        "",
        "```sh",
    ]
    if make_target:
        lines.append(make_target)
    else:
        lines.append("make <vendor>_<board>_default  # TODO: replace with correct target")
    lines += [
        "```",
        "",
        "## Debug Port {#debug_port}",
        "",
        "<!--",
        "- What port the PX4 System Console runs on",
        "- What port the SWD interface runs on",
        "- Whether it follows a Pixhawk Debug Standard, and if so full or small form",
        "- Pinout and connector information if NOT pixhawk standard",
        "-->",
        "<!--",
        "The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the **[FMU | IO] Debug** port.",
        "",
        "For Pixhawk Standard Debug: The pinouts and connector comply with the [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full)|[Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) interface defined in the [Pixhawk Connector Standard](...) interface (JST SM10B connector).",
        "",
        "For other ports: Insert Pin / Signal / Volt layout...",
        "-->",
        "",
    ]

    # Assembly section (RC + GPS) near the end, before Further Information
    lines.append("## Assembly {#assembly}")
    lines.append("")
    for section_key in SECTION_ORDER:
        if section_key not in ASSEMBLY_SECTION_KEYS:
            continue
        generator = SECTION_GENERATORS.get(section_key)
        if generator:
            lines.append(generator(board_key or "unknown/unknown", entry))
            lines.append("")

    lines += [
        "## Further Information",
        "",
        "TODO: links to additional resources.",
    ]

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Markdown generation from gathered data
# ---------------------------------------------------------------------------
def generate_markdown(data: dict) -> str:
    """Generate the proposed-sections markdown file from gathered board data."""
    out_lines = []
    out_lines.append("# PX4 Board FC Doc Sections")
    out_lines.append("")
    out_lines.append("Auto-generated from source. See `fc_doc_generator.py`.")
    out_lines.append("")
    out_lines.append("---")
    out_lines.append("")

    for key, v in data.items():
        out_lines.append(f"### {key}")
        out_lines.append("")

        if v.get("doc_url"):
            out_lines.append(f"**Doc:** [{v['doc_url']}]({v['doc_url']})")
            out_lines.append(f"**Documented:** {'Yes' if v['documented'] else 'No'}")
        else:
            out_lines.append("**Doc:** No flight controller doc found")
        out_lines.append("")

        for section_key in SECTION_ORDER:
            generator = SECTION_GENERATORS.get(section_key)
            if not generator:
                continue
            proposed = generator(key, v)
            out_lines.append(f"**Proposed `{section_key}` section:**")
            out_lines.append("")
            out_lines.append("```markdown")
            out_lines.append(proposed)
            out_lines.append("```")
            out_lines.append("")

        if v.get("existing_pwm_section"):
            out_lines.append("**Existing doc section (for reference):**")
            out_lines.append("")
            out_lines.append("<!--")
            out_lines.append(v["existing_pwm_section"])
            out_lines.append("-->")
            out_lines.append("")

        out_lines.append("---")
        out_lines.append("")

    return "\n".join(out_lines)


# ---------------------------------------------------------------------------
# Board data gatherer
# ---------------------------------------------------------------------------
def gather_board_data() -> dict:
    """
    Parse all boards and return a flat JSON-serialisable results dict.

    To add data from a new source, call your parser here and merge the result
    into the entry dict before it is stored in results.
    """
    results = {}

    board_dirs = sorted([
        p.parent.parent
        for p in BOARDS.rglob("src/timer_config.cpp")
    ])

    for board_path in board_dirs:
        vendor = board_path.parent.name
        board = board_path.name
        key = f"{vendor}/{board}"

        chip_family = get_chip_family(board_path)
        chip_variant = parse_chip_variant(board_path)
        board_cfg = parse_board_config(board_path)
        timer_cfg = parse_timer_config(board_path)
        serial_cfg = parse_serial_config(board_path)
        rc_cfg = parse_rc_config(board_path)
        gps_cfg = parse_gps_config(board_path)
        power_cfg = parse_power_config(board_path)
        sd_cfg = parse_sd_card_config(board_path)
        sensor_cfg = parse_sensor_config(board_path)
        rc_sensor_cfg = parse_rc_board_sensors(board_path)
        # Supplement defconfig results with rc.board_sensors chip names (deduplicated)
        for _cat in ('imu', 'baro', 'mag', 'osd'):
            for _chip in rc_sensor_cfg[_cat]:
                if _chip not in sensor_cfg[_cat]:
                    sensor_cfg[_cat].append(_chip)
        iface_cfg = parse_interface_config(board_path)
        i2c_bus_cfg = parse_i2c_bus_config(board_path)
        sensor_bus_cfg = parse_rc_board_sensor_bus(board_path)
        sensor_variant_info = parse_sensor_variant_blocks(board_path)

        timers = timer_cfg.get("timers", [])
        channels = timer_cfg.get("channels", [])
        groups = compute_groups(timers, channels)
        groups = compute_bdshot(groups, channels, chip_family)

        doc_filename = BOARD_TO_DOC.get(key)
        doc_url = doc_to_url(doc_filename)

        doc_info = {"exists": False, "has_pwm_section": False}
        if doc_filename:
            doc_path = FC_DOCS / doc_filename
            doc_info = parse_fc_doc(doc_path)

        results[key] = {
            "board": key,
            "chip_family": chip_family,
            "chip_model":   chip_variant.get('chip_model'),
            "total_outputs": board_cfg.get("total_outputs"),
            "has_io_board": board_cfg.get("has_io_board", False),
            "io_outputs": board_cfg.get("io_outputs", 0),
            "groups": [
                {
                    "group": g["group"],
                    "timer": g["timer"],
                    "outputs": g["outputs"],
                    "dshot": g["dshot"],
                    "dshot_outputs": g.get("dshot_outputs", g["outputs"] if g["dshot"] else []),
                    "non_dshot_outputs": g.get("non_dshot_outputs", [] if g["dshot"] else g["outputs"]),
                    "bdshot_outputs": g.get("bdshot_outputs", []),
                    "bdshot_output_only": g.get("bdshot_output_only", []),
                }
                for g in groups
            ],
            "serial_ports": serial_cfg.get("serial_ports", []),
            "has_rc_input": rc_cfg.get('has_rc_input', False),
            "has_common_rc": rc_cfg.get('has_common_rc', False),
            "rc_serial_device": rc_cfg.get('rc_serial_device'),
            "has_ppm_pin": rc_cfg.get('has_ppm_pin', False),
            "ppm_shared_with_rc_serial": rc_cfg.get('ppm_shared_with_rc_serial', False),
            "has_pps_capture":          gps_cfg.get('has_pps_capture', False),
            "has_safety_switch":        gps_cfg.get('has_safety_switch', False),
            "has_safety_led":           gps_cfg.get('has_safety_led', False),
            "has_buzzer":               gps_cfg.get('has_buzzer', False),
            "num_power_inputs":         power_cfg.get('num_power_inputs', 1),
            "has_redundant_power":      power_cfg.get('has_redundant_power', False),
            "has_dual_battery_monitoring": power_cfg.get('has_dual_battery_monitoring', False),
            "has_dronecan_power_input": power_cfg.get('has_dronecan_power_input', False),
            "power_monitor_type":       power_cfg.get('power_monitor_type'),
            "has_sd_card":              sd_cfg.get('has_sd_card', False),
            "has_ethernet":             sd_cfg.get('has_ethernet', False),
            "has_heater":               sd_cfg.get('has_heater', False),
            "sensor_imu_drivers":       sensor_cfg.get('imu', []),
            "sensor_baro_drivers":      sensor_cfg.get('baro', []),
            "sensor_mag_drivers":       sensor_cfg.get('mag', []),
            "sensor_osd_drivers":       sensor_cfg.get('osd', []),
            "sensor_bus_info":          sensor_bus_cfg,
            "sensor_variant_info":      sensor_variant_info,
            "i2c_bus_config":           i2c_bus_cfg or None,
            "num_i2c_buses":            (
                len(i2c_bus_cfg.get('external', [])) + len(i2c_bus_cfg.get('internal', []))
                if i2c_bus_cfg else iface_cfg.get('num_i2c_buses', 0)
            ),
            "num_spi_buses":            iface_cfg.get('num_spi_buses', 0),
            "num_can_buses":            iface_cfg.get('num_can_buses', 0),
            "has_usb":                  iface_cfg.get('has_usb', False),
            "doc_url": doc_url,
            "doc_file": doc_filename,
            "doc_exists": doc_info.get("exists", False),
            "documented": doc_info.get("has_pwm_section", False),
            "existing_pwm_section": doc_info.get("pwm_section"),
        }

    return results


def _find_board_key(manufacturer: str, product: str) -> str | None:
    """
    Try to match manufacturer+product to a boards/<vendor>/<board> key.

    Searches both the manufacturer's own vendor directory AND boards/px4/
    (Pixhawk reference designs), since many third-party boards are built on
    px4/fmu-v* hardware and that is the correct board target to use.

    Returns the key if a single confident fuzzy match is found, else None.
    Always prints the candidate list so the user can choose.
    """
    vendor_slug = _make_slug(manufacturer).replace('_', '')   # "holybro"
    product_slug = _make_slug(product).replace('_', '')        # "kakuteh7"

    # Vendor dirs that fuzzy-match the manufacturer
    vendor_dirs = [d for d in BOARDS.iterdir() if d.is_dir()
                   and vendor_slug in d.name.replace('-', '')]

    # Always include boards/px4/ (reference designs: fmu-v5x, fmu-v6x, …)
    px4_dir = BOARDS / 'px4'
    if px4_dir.is_dir() and px4_dir not in vendor_dirs:
        vendor_dirs.append(px4_dir)

    def _boards_in(vd):
        return [bd for bd in sorted(vd.iterdir())
                if bd.is_dir() and (bd / "src/timer_config.cpp").exists()]

    # Fuzzy-match product name against board names across all candidate dirs
    candidates = []
    for vd in vendor_dirs:
        for bd in _boards_in(vd):
            board_slug = bd.name.replace('-', '').replace('_', '')
            if product_slug in board_slug or board_slug in product_slug:
                candidates.append(f"{vd.name}/{bd.name}")

    if len(candidates) == 1:
        print(f"  Auto-matched board: {candidates[0]}")
        return candidates[0]

    if candidates:
        print(f"  Multiple board matches found:")
        for c in candidates:
            print(f"    {c}")
    else:
        # No fuzzy match — list every available board from all searched dirs
        print(f"  No auto-match found for '{product}'. Available boards:")
        for vd in vendor_dirs:
            for bd in _boards_in(vd):
                print(f"    {vd.name}/{bd.name}")

    return None


def create_stub_doc(manufacturer: str, product: str,
                    fmu_version: str = None, board_key: str = None,
                    force: bool = False,
                    since_version: str = DEFAULT_SINCE_VERSION,
                    manufacturer_url: str = None,
                    rc_ports_wizard: list = None,
                    gps_ports_wizard: list = None,
                    power_ports_wizard: list = None,
                    overview_wizard: dict = None,
                    i2c_buses_wizard: list = None) -> Path | None:
    """Create a stub FC documentation page in docs/en/flight_controller/."""
    # Auto-discover board key first so fmu_version can be inferred before the
    # filename is built (px4/fmu-v6x → fmu_version="fmu-v6x" → filename suffix).
    if not board_key:
        board_key = _find_board_key(manufacturer, product)

    # For px4 reference designs the board name IS the fmu version (fmu-v6x, etc.)
    if not fmu_version and board_key and board_key.startswith('px4/'):
        fmu_version = board_key.split('/', 1)[1]

    filename = _doc_filename(manufacturer, product, fmu_version)
    doc_path = FC_DOCS / filename

    if doc_path.exists() and not force:
        print(f"ERROR: {doc_path} already exists. Use --force to overwrite.")
        return None

    entry = {}
    if board_key:
        data = gather_board_data()
        entry = data.get(board_key, {})
        if not entry:
            print(f"WARNING: board '{board_key}' not found in parsed data; PWM section will be empty.")

    if rc_ports_wizard or gps_ports_wizard or power_ports_wizard or overview_wizard or i2c_buses_wizard:
        entry = dict(entry)   # shallow copy — don't mutate cached data
    if rc_ports_wizard:
        entry['rc_ports_wizard'] = rc_ports_wizard
    if gps_ports_wizard:
        entry['gps_ports_wizard'] = gps_ports_wizard
    if power_ports_wizard:
        entry['power_ports_wizard'] = power_ports_wizard
    if overview_wizard:
        entry['overview_wizard'] = overview_wizard
    if i2c_buses_wizard:
        entry['i2c_buses_wizard'] = i2c_buses_wizard

    # Infer manufacturer URL from existing docs when not explicitly provided
    if not manufacturer_url:
        manufacturer_url = _find_manufacturer_url(manufacturer)

    content = generate_full_template(board_key, entry, manufacturer, product, since_version, manufacturer_url,
                                     doc_name=Path(filename).stem)

    # Move all <!-- *-source-data --> comments from inline positions to end of doc
    content, _source_comments = _collect_and_strip_source_comments(content)
    if _source_comments:
        content = content.rstrip('\n') + '\n\n' + _source_comments + '\n'

    # Embed all wizard-supplied data as a hidden comment so --apply can read it back
    # later without needing the wizard.json cache file to be present.
    wizard_data = {
        'manufacturer': manufacturer, 'product': product,
        'fmu_version': fmu_version, 'since_version': since_version,
        'manufacturer_url': manufacturer_url,
        'rc_ports_wizard': rc_ports_wizard,
        'gps_ports_wizard': gps_ports_wizard,
        'power_ports_wizard': power_ports_wizard,
        'overview_wizard': overview_wizard,
        'i2c_buses_wizard': i2c_buses_wizard,
    }
    if any(v is not None for v in wizard_data.values()):
        content = _embed_wizard_data_comment(content, wizard_data)

    doc_path.write_text(content, encoding='utf-8')
    print(f"Created: {doc_path}")

    # Print the SUMMARY.md line the user needs to add manually
    display = f"{manufacturer} {product}"
    if fmu_version:
        display += f" ({fmu_version.upper()})"
    print(f"\nAdd to docs/en/SUMMARY.md:")
    print(f"      - [{display}](flight_controller/{filename})")
    return doc_path


# ---------------------------------------------------------------------------
# Document quality checks
# ---------------------------------------------------------------------------
# Each SectionSpec defines one expected section in an FC doc and the checks
# to run against it.  DOC_SECTION_SPECS mirrors generate_full_template() —
# update both together when the template changes.
# ---------------------------------------------------------------------------

class SectionSpec:
    """Expected properties of one section in an FC doc."""

    def __init__(self, name, heading_pattern, heading_checks=None, body_checks=None):
        self.name = name
        self.heading_pattern = re.compile(heading_pattern, re.IGNORECASE | re.MULTILINE)
        self.heading_checks = heading_checks or []
        self.body_checks = body_checks or []


# --- individual check functions -------------------------------------------
# Each accepts (text: str | None) and returns (passed: bool, message: str).
# text is None when the section was not found in the document.

def _chk_no_todo(text):
    """Section body should not contain bare TODO placeholders."""
    if text and re.search(r'\bTODO\b', text):
        return (False, "contains TODO placeholder")
    return (True, "no TODOs")


def _chk_no_html_comments(text):
    """Section body should not contain HTML guidance comments (<!-- ... -->)."""
    if text and '<!--' in text:
        return (False, "contains <!-- comment (guidance not yet replaced)")
    return (True, "no HTML comments")


def _chk_has_badge(text):
    """Document preamble should contain a <Badge> PX4 version component."""
    if text and '<Badge' in text:
        return (True, "has <Badge> version component")
    return (False, "missing <Badge> version component")


def _chk_has_sh_make(text):
    """Building Firmware section should have a ```sh block with a make …_default command."""
    if not text:
        return (True, "n/a (section absent)")
    if re.search(r'```sh[^`]*\bmake\b[^`]*_default', text, re.DOTALL):
        return (True, "has make command in ```sh block")
    return (False, "missing ```sh block with `make …_default` target")


def _chk_anchor(anchor_id):
    """Return a check that verifies the heading line contains {#anchor_id}."""
    expected = "{#" + anchor_id + "}"

    def _check(heading_line):
        if heading_line and expected in heading_line:
            return (True, f"has anchor {expected}")
        return (False, f"missing anchor {expected}")

    _check.__name__ = f"anchor_{anchor_id}"
    return _check


def _chk_title(expected_title):
    """Return a check that verifies the heading text matches expected_title exactly."""

    def _check(heading_line):
        if heading_line is None:
            return (True, "n/a (section absent)")
        # Strip leading hashes and trailing anchor suffix
        text = re.sub(r'\s*\{#[^}]+\}', '', heading_line).strip()
        text = re.sub(r'^#+\s*', '', text).strip()
        if text == expected_title:
            return (True, f"title is '{expected_title}'")
        return (False, f"title is '{text}', expected '{expected_title}'")

    _check.__name__ = f"title_{_make_slug(expected_title)}"
    return _check


# --- debug port content checks --------------------------------------------
# These run on the Debug Port section body.
# Where a check is only required for non-standard ports, it first detects
# whether a Pixhawk Debug Full/Mini standard is cited and returns n/a if so.

_PIXHAWK_STD_RE = re.compile(r'Pixhawk Debug\s+(Full|Mini)', re.IGNORECASE)
_PIXHAWK_STD_NA = (True, "n/a (Pixhawk standard port)")


def _chk_debug_fmu_io(text):
    """Section should identify whether it is the FMU or IO debug port."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if re.search(r'\b(FMU|IO)\b', text):
        return (True, "identifies FMU or IO port")
    return (False, "does not specify which port (FMU or IO)")


def _chk_debug_standard(text):
    """Section should state whether port follows Pixhawk Debug Full or Mini standard."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    m = _PIXHAWK_STD_RE.search(text)
    if m:
        return (True, f"references Pixhawk Debug {m.group(1)}")
    return (False, "no Pixhawk Debug Full/Mini reference — non-standard pinout details required")


def _chk_debug_pinout(text):
    """Non-standard port: section must include a pinout table."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if _PIXHAWK_STD_RE.search(text):
        return _PIXHAWK_STD_NA
    # Accept a markdown table (≥2 data rows with |) or a definition list
    data_rows = [l for l in text.splitlines()
                 if re.match(r'\s*\|', l) and not re.match(r'\s*\|[-| ]+\|', l)]
    if len(data_rows) >= 2:
        return (True, "has pinout table")
    return (False, "non-standard port: no pinout table found")


def _chk_debug_connector(text):
    """Non-standard port: section must name the connector."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if _PIXHAWK_STD_RE.search(text):
        return _PIXHAWK_STD_NA
    if re.search(r'\b(JST|Molex|Hirose|DF\d+|GH|SH|PicoBlade|connector)\b', text, re.IGNORECASE):
        return (True, "names the connector")
    return (False, "non-standard port: connector name not specified")


def _chk_debug_cable(text):
    """Non-standard port: section should indicate whether a specialist cable is provided."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if _PIXHAWK_STD_RE.search(text):
        return _PIXHAWK_STD_NA
    if re.search(r'\bcable\b', text, re.IGNORECASE):
        return (True, "mentions cable")
    return (False, "non-standard port: no mention of specialist cable")


def _chk_debug_links(text):
    """Section should link to both the SWD debug guide and the System Console guide."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    has_swd = bool(re.search(r'swd_debug\.md|SWD interface', text, re.IGNORECASE))
    has_console = bool(re.search(r'system_console\.md|System Console', text, re.IGNORECASE))
    missing = []
    if not has_swd:
        missing.append("SWD debug")
    if not has_console:
        missing.append("System Console")
    if not missing:
        return (True, "links to SWD debug and System Console")
    return (False, "missing link(s): " + ", ".join(missing))


def _chk_gps_port_named(text):
    """Section should reference at least one GPS port name in backticks."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if re.search(r'`[A-Za-z0-9]+`', text):
        return (True, "names a port in backticks")
    return (False, "no port name in backticks — add GPS port label(s)")


def _chk_gps_compass_link(text):
    """Section should link to the GPS mounting or gps_compass guide."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if re.search(r'mount_gps_compass|gps_compass|gps/README', text, re.IGNORECASE):
        return (True, "links to GPS/compass guide")
    return (False, "missing link to GPS mounting or compass guide")


def _chk_rc_link(text):
    """Section should link to rc_transmitter_receiver.md."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if re.search(r'rc_transmitter_receiver\.md', text):
        return (True, "links to RC transmitter/receiver guide")
    return (False, "missing link to rc_transmitter_receiver.md")


def _chk_rc_port_named(text):
    """Section should name at least one RC port as labeled on the board (bold text)."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if re.search(r'\*\*[A-Za-z0-9][^*]+\*\*', text):
        return (True, "names a port in bold")
    return (False, "no bold port name found — add port label(s) as printed on the board")


def _chk_rc_connector(text):
    """Section should state the connector type used for RC port(s)."""
    if not text or not text.strip():
        return (True, "n/a (section absent)")
    if re.search(r'\bJST[- ]?[A-Z0-9]+\b|connector|\bplug\b|\bsocket\b|\bpin\b', text, re.IGNORECASE):
        return (True, "mentions connector type")
    return (False, "no connector type mentioned — state the connector used for the RC port(s)")


# --- canonical section spec list ------------------------------------------
# Keep in sync with generate_full_template().  Add a new SectionSpec here
# whenever a new required heading is added to the template.

DOC_SECTION_SPECS = [
    SectionSpec(
        name="Title (H1)",
        heading_pattern=r'^#\s+\S',
        body_checks=[_chk_no_todo, _chk_has_badge],
    ),
    SectionSpec(
        name="Specifications",
        heading_pattern=r'^##\s+(Specifications?|Key Features|Key Design Points|Quick Summary|Overview)',
        heading_checks=[_chk_anchor("specifications")],
        body_checks=[_chk_no_todo],
    ),
    SectionSpec(
        name="Where to Buy",
        heading_pattern=r'^##\s+Where to Buy',
        heading_checks=[_chk_title("Where to Buy"), _chk_anchor("store")],
        body_checks=[_chk_no_todo],
    ),
    SectionSpec(
        name="Pinouts",
        heading_pattern=r'^##\s+Pinouts?',
        body_checks=[_chk_no_todo],
    ),
    SectionSpec(
        name="PWM Outputs",
        heading_pattern=r'^##\s+PWM Outputs',
        heading_checks=[_chk_title("PWM Outputs"), _chk_anchor("pwm_outputs")],
        body_checks=[_chk_no_todo],
    ),
    SectionSpec(
        name="Assembly",
        heading_pattern=r'^##\s+Assembly',
        heading_checks=[_chk_title("Assembly"), _chk_anchor("assembly")],
        body_checks=[],
    ),
    SectionSpec(
        name="Radio Control",
        heading_pattern=r'^#{2,3}\s+Radio Control',
        heading_checks=[_chk_title("Radio Control"), _chk_anchor("radio_control")],
        body_checks=[_chk_no_todo, _chk_rc_link, _chk_rc_port_named, _chk_rc_connector],
    ),
    SectionSpec(
        name="GPS & Compass",
        heading_pattern=r'^#{2,3}\s+GPS',
        heading_checks=[_chk_title("GPS & Compass"), _chk_anchor("gps_compass")],
        body_checks=[_chk_no_todo, _chk_gps_port_named, _chk_gps_compass_link],
    ),
    SectionSpec(
        name="Building Firmware",
        heading_pattern=r'^##\s+Building Firmware',
        body_checks=[_chk_no_todo, _chk_has_sh_make],
    ),
    SectionSpec(
        name="Debug Port",
        heading_pattern=r'^##\s+Debug Port',
        heading_checks=[_chk_title("Debug Port"), _chk_anchor("debug_port")],
        body_checks=[
            _chk_no_todo,
            _chk_no_html_comments,
            _chk_debug_fmu_io,
            _chk_debug_standard,
            _chk_debug_pinout,
            _chk_debug_connector,
            _chk_debug_cable,
            _chk_debug_links,
        ],
    ),
    SectionSpec(
        name="Further Information",
        heading_pattern=r'^##\s+Further Inf',
        body_checks=[_chk_no_todo],
    ),
]


def _has_quickstart_link(text: str) -> bool:
    """Return True if text contains a link to a quickstart or assembly guide."""
    return bool(re.search(r'quickstart|quick_start|assembly_setup|assembly_guide', text, re.IGNORECASE))


def _chk_no_url_images(doc_text: str) -> list:
    """Return list of URL image sources found in doc_text (should be empty)."""
    return [m.group(1) for m in re.finditer(r'!\[[^\]]*\]\((https?://[^)]+)\)', doc_text)]


def _chk_image_filenames(doc_text: str, doc_name: str) -> list:
    """Return list of image path issues: non-lowercase filenames or wrong folder."""
    issues = []
    expected_prefix = f'../../assets/flight_controller/{doc_name}/'
    for m in re.finditer(r'!\[[^\]]*\]\(([^)]+)\)', doc_text):
        src = m.group(1)
        if src.startswith('http://') or src.startswith('https://'):
            continue
        filename = Path(src).name
        if filename != filename.lower():
            issues.append(f"uppercase in filename: {filename}")
        if 'assets/flight_controller/' in src and not src.startswith(expected_prefix):
            issues.append(f"wrong folder (expected {expected_prefix}...): {src}")
    return issues


# Section names that defer to a quickstart guide (body checks become NA when link present)
_QUICKSTART_SKIP_SECTIONS = {"Radio Control", "GPS & Compass"}


def _parse_doc_sections(doc_text):
    """
    Split an FC doc into (preamble, [(heading_line, body_text), ...]).

    preamble — text before the first heading (contains Badge, intro, etc.)
    heading_line — raw heading string, e.g. "## Debug Port {#debug_port}"
    body_text — content between this heading and the start of the next
    """
    heading_re = re.compile(r'(?m)^(#{1,6}\s+.+)$')
    matches = list(heading_re.finditer(doc_text))
    if not matches:
        return doc_text, []
    preamble = doc_text[:matches[0].start()]
    pairs = []
    for i, m in enumerate(matches):
        heading = m.group(1).rstrip()
        body_start = m.end()
        body_end = matches[i + 1].start() if i + 1 < len(matches) else len(doc_text)
        pairs.append((heading, doc_text[body_start:body_end]))
    return preamble, pairs


def check_doc(doc_path):
    """
    Run all DOC_SECTION_SPECS checks against one FC doc file.

    Returns a list of result dicts, each with keys:
      section  — SectionSpec.name
      check    — check function name
      passed   — bool
      message  — short description of the result
    """
    doc_path = Path(doc_path)
    text = doc_path.read_text(encoding="utf-8")
    preamble, sections = _parse_doc_sections(text)

    results = []
    for spec in DOC_SECTION_SPECS:
        # Find first heading that matches this spec
        heading_line = None
        body_text = None
        for h, b in sections:
            if spec.heading_pattern.match(h):
                heading_line = h
                body_text = b
                break

        # For H1, fold the preamble into the body so Badge is visible to checks
        if spec.name == "Title (H1)":
            body_text = preamble + (body_text or "")

        # Implicit "present" check
        results.append({
            "section": spec.name,
            "check": "present",
            "passed": heading_line is not None,
            "message": "found" if heading_line is not None else "MISSING — heading not found",
        })

        for chk in spec.heading_checks:
            passed, msg = chk(heading_line)
            results.append({"section": spec.name,
                            "check": chk.__name__.lstrip("_").removeprefix("chk_"),
                            "passed": passed, "message": msg})

        has_quickstart = (
            spec.name in _QUICKSTART_SKIP_SECTIONS
            and body_text is not None
            and _has_quickstart_link(body_text)
        )
        if has_quickstart:
            results.append({
                "section": spec.name,
                "check": "quickstart_present",
                "passed": True,
                "message": "delegates to quickstart/assembly guide",
            })

        for chk in spec.body_checks:
            if has_quickstart:
                passed, msg = True, "n/a (quickstart/assembly link present)"
            else:
                passed, msg = chk(body_text)
            results.append({"section": spec.name,
                            "check": chk.__name__.lstrip("_").removeprefix("chk_"),
                            "passed": passed, "message": msg})

    # Global document checks (not tied to a specific section)
    doc_name = doc_path.stem
    url_issues = _chk_no_url_images(text)
    if url_issues:
        for issue in url_issues[:3]:
            results.append({
                "section": "(document)",
                "check": "no_url_images",
                "passed": False,
                "message": f"image uses URL: {issue[:80]}",
            })
    else:
        results.append({
            "section": "(document)",
            "check": "no_url_images",
            "passed": True,
            "message": "no URL images found",
        })

    filename_issues = _chk_image_filenames(text, doc_name)
    if filename_issues:
        for issue in filename_issues[:3]:
            results.append({
                "section": "(document)",
                "check": "image_filenames",
                "passed": False,
                "message": issue,
            })
    else:
        results.append({
            "section": "(document)",
            "check": "image_filenames",
            "passed": True,
            "message": "all image filenames OK",
        })

    return results


def print_check_results(doc_path, results):
    """Print check results for one doc as a human-readable table."""
    passes = sum(1 for r in results if r["passed"])
    fails = sum(1 for r in results if not r["passed"])
    status = "OK" if fails == 0 else f"FAIL ({fails} issue{'s' if fails != 1 else ''})"
    print(f"\nChecking: {doc_path}  [{status}]")

    col_sec = max(len(r["section"]) for r in results)
    col_chk = max(len(r["check"]) for r in results)
    divider = "  " + "-" * (col_sec + col_chk + 30)
    print(f"  {'Section':<{col_sec}}  {'Check':<{col_chk}}  Result")
    print(divider)

    prev_sec = None
    for r in results:
        sec_label = r["section"] if r["section"] != prev_sec else ""
        prev_sec = r["section"]
        mark = "✓" if r["passed"] else "✗"
        print(f"  {sec_label:<{col_sec}}  {r['check']:<{col_chk}}  {mark} {r['message']}")

    print(f"\n  {passes} passed, {fails} failed")


def check_all_docs():
    """Run DOC_SECTION_SPECS checks on every .md file in FC_DOCS and print a summary."""
    doc_paths = sorted(FC_DOCS.glob("*.md"))
    if not doc_paths:
        print(f"No FC docs found in {FC_DOCS}")
        return

    all_results = [(p, check_doc(p)) for p in doc_paths]

    total = len(all_results)
    fully_passing = sum(1 for _, r in all_results if all(x["passed"] for x in r))
    print(f"\nFC Doc Quality Report — {total} docs checked\n")

    col_file = max(len(p.name) for p, _ in all_results)
    print(f"  {'File':<{col_file}}  Pass  Fail  Issues")
    print("  " + "-" * (col_file + 50))

    for p, results in all_results:
        passes = sum(1 for r in results if r["passed"])
        fails = sum(1 for r in results if not r["passed"])
        issues = "; ".join(
            f"{r['section']}: {r['message']}" for r in results if not r["passed"]
        )
        issues_str = (issues[:70] + "…") if len(issues) > 70 else issues
        print(f"  {p.name:<{col_file}}  {passes:>4}  {fails:>4}  {issues_str}")

    print(f"\n  {fully_passing}/{total} docs fully pass all checks")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate FC documentation sections from PX4 board source files.",
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--apply", "-a", action="store_true",
        help="Apply generated sections to FC doc files (in addition to writing JSON/MD).",
    )
    mode_group.add_argument(
        "--new-doc", action="store_true",
        help="Create a new stub FC doc in docs/en/flight_controller/.",
    )
    mode_group.add_argument(
        "--check-doc", metavar="FILE",
        help="Check a single FC doc against DOC_SECTION_SPECS and print a report.",
    )
    mode_group.add_argument(
        "--check-all", action="store_true",
        help="Check all FC docs in docs/en/flight_controller/ and print a summary.",
    )
    parser.add_argument(
        "--section", choices=list(SECTION_GENERATORS.keys()), default=None,
        help="Apply only this section key (implies --apply).",
    )
    parser.add_argument(
        "--doc", default=None,
        metavar="FILENAME",
        help="Apply only to this doc file, e.g. cuav_x25-evo.md (implies --apply). "
             "Stem without extension is also accepted.",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        metavar="DIR",
        help="Base output directory: fc_sections.md and per-board JSON go here (default: tests/snapshots/ for fc_sections.md, metadata/ for JSON).",
    )
    parser.add_argument(
        "--manufacturer", "-m", default=None,
        help="Manufacturer display name, e.g. 'Holybro' (required with --new-doc).",
    )
    parser.add_argument(
        "--product", "-p", default=None,
        help="Product display name, e.g. 'Pixhawk 6X' (required with --new-doc).",
    )
    parser.add_argument(
        "--board", "-b", default=None,
        help="Board key e.g. holybro/kakuteh7 (optional; auto-discovered if omitted).",
    )
    parser.add_argument(
        "--fmu-version", default=None,
        help="Optional FMU version suffix e.g. fmu-v6x.",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Overwrite existing doc file (used with --new-doc).",
    )
    parser.add_argument(
        "--since-version", default=DEFAULT_SINCE_VERSION,
        metavar="VERSION",
        help=f"PX4 version the board was introduced in, e.g. v1.18 (default: {DEFAULT_SINCE_VERSION}).",
    )
    parser.add_argument(
        "--manufacturer-url", default=None,
        metavar="URL",
        help="Manufacturer website URL for the hardware support warning.",
    )
    parser.add_argument(
        "--wizard", "-w", action="store_true",
        help="Interactively prompt for any missing --new-doc values (only valid with --new-doc).",
    )
    args = parser.parse_args()

    if args.wizard and not args.new_doc:
        parser.error("--wizard can only be used with --new-doc")

    if args.check_doc:
        check_path = Path(args.check_doc)
        if not check_path.exists():
            parser.error(f"File not found: {args.check_doc}")
        results = check_doc(check_path)
        print_check_results(check_path, results)
    elif args.check_all:
        check_all_docs()
    elif args.new_doc:
        if args.wizard:
            _run_wizard(args)
        elif not args.manufacturer or not args.product:
            parser.error("--new-doc requires --manufacturer and --product (or use --wizard)")
        _wiz_cache = {}
        if args.manufacturer and args.product and not args.wizard:
            # Load cached wizard data so --new-doc --force re-runs pick up prior answers
            _wiz_cache = _load_wizard_cache(args.manufacturer, args.product) or {}
            if _wiz_cache:
                print("  (Using cached wizard data)")
        create_stub_doc(args.manufacturer, args.product,
                        fmu_version=args.fmu_version,
                        board_key=args.board or _wiz_cache.get('board'),
                        force=args.force,
                        since_version=args.since_version or _wiz_cache.get('since_version'),
                        manufacturer_url=args.manufacturer_url or _wiz_cache.get('manufacturer_url'),
                        rc_ports_wizard=getattr(args, 'rc_ports_wizard', None) or _wiz_cache.get('rc_ports_wizard'),
                        gps_ports_wizard=getattr(args, 'gps_ports_wizard', None) or _wiz_cache.get('gps_ports_wizard'),
                        power_ports_wizard=getattr(args, 'power_ports_wizard', None) or _wiz_cache.get('power_ports_wizard'),
                        overview_wizard=getattr(args, 'overview_wizard', None) or _wiz_cache.get('overview_wizard'),
                        i2c_buses_wizard=getattr(args, 'i2c_buses_wizard', None) or _wiz_cache.get('i2c_buses_wizard'))
    else:
        data = gather_board_data()
        meta_dir = args.output_dir / "metadata" if args.output_dir else METADATA_DIR
        snapshots_dir = args.output_dir or (SCRIPT_DIR / "tests" / "snapshots")
        meta_dir.mkdir(parents=True, exist_ok=True)
        snapshots_dir.mkdir(parents=True, exist_ok=True)

        for k, v in data.items():
            board_json = {ek: ev for ek, ev in v.items() if ek != "existing_pwm_section"}
            safe_key = k.replace("/", "_")
            board_json_path = meta_dir / f"{safe_key}_data.json"
            board_json_path.write_text(json.dumps(board_json, indent=2), encoding="utf-8")
        print(f"Written: {len(data)} board JSON files to {meta_dir}/")

        md_path = snapshots_dir / "fc_sections.md"
        md_path.write_text(generate_markdown(data), encoding="utf-8")
        print(f"Written: {md_path}")

        print(f"Boards processed: {len(data)}")
        has_doc = sum(1 for v in data.values() if v["doc_exists"])
        documented = sum(1 for v in data.values() if v["documented"])
        dshot_boards = sum(1 for v in data.values() if any(g["dshot"] for g in v["groups"]))
        print(f"Boards with FC doc: {has_doc}")
        print(f"Boards with existing PWM section: {documented}")
        print(f"Boards with any DShot: {dshot_boards}")

        if args.apply or args.section or args.doc:
            sections = [args.section] if args.section else None
            print()
            updated, skipped = apply_sections_to_docs(data, sections, doc_filter=args.doc)
            print(f"\nApply done. Updated: {len(updated)}, Skipped: {len(skipped)}")
