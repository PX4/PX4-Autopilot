#!/usr/bin/env bash
#
# brick_ecc.sh - plant an uncorrectable flash ECC error (STM32H7)
#
# Builds brick_ecc.c, loads it into RAM over ST-Link, and runs it to plant an
# uncorrectable ECC word in the parameter sector. Requires the board on SWD
# and st-tools + an ARM gdb. See README.md for usage and adaptation.
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

GCC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
# st-link/st-util cannot write the Cortex-M7 TCMs (ITCM/DTCM) over SWD, so the
# stub lives in unused AXI SRAM (the firmware uses only the low ~54 KiB of the
# 512 KiB region). We reset the board afterwards, so clobbering RAM is fine.
LOAD_ADDR=0x24040000           # AXI SRAM, well past the firmware's RAM use
STACK=0x24048000               # 32 KiB above the stub

# Pick an ARM-capable gdb that actually runs. The 2020-q4 arm-none-eabi-gdb is
# linked against libncurses.so.5 which modern distros do not ship, so prefer
# gdb-multiarch (it auto-detects the Cortex-M target from st-util).
GDB=""
for cand in gdb-multiarch arm-none-eabi-gdb gdb; do
	if command -v "$cand" >/dev/null 2>&1 && "$cand" --version >/dev/null 2>&1; then
		GDB="$cand"; break
	fi
done
if [ -z "$GDB" ]; then
	echo "No working ARM gdb found. Install one: sudo apt-get install gdb-multiarch" >&2
	exit 1
fi
echo "using gdb: $GDB"

"$GCC" -mcpu=cortex-m7 -mthumb -nostdlib -nostartfiles -ffreestanding -Os \
	-Wl,-Ttext=$LOAD_ADDR -Wl,-e_start -o "$HERE/brick_ecc.elf" "$HERE/brick_ecc.c"
"$OBJCOPY" -O binary "$HERE/brick_ecc.elf" "$HERE/brick_ecc.bin"

# entry = address of _start in the loaded image (should equal LOAD_ADDR)
ENTRY=0x$(arm-none-eabi-nm "$HERE/brick_ecc.elf" | awk '/ _start$/{print $1}')
echo "stub _start = $ENTRY (load $LOAD_ADDR)"

st-util >/tmp/brick_stutil.log 2>&1 &
STUTIL_PID=$!
trap 'kill "$STUTIL_PID" 2>/dev/null || true' EXIT
sleep 1

# st-util has no "monitor reset" - it halts on attach, so we run the stub from
# the halted state (it masks interrupts itself) and reset via SCB AIRCR after
# the stub's bkpt to reboot into the firmware.
"$GDB" -nx -q -batch \
	-ex "target extended-remote :4242" \
	-ex "monitor halt" \
	-ex "restore $HERE/brick_ecc.bin binary $LOAD_ADDR" \
	-ex "set \$control = 0" \
	-ex "set \$sp = $STACK" \
	-ex "set \$xpsr = 0x01000000" \
	-ex "set \$pc = $ENTRY" \
	-ex "continue" \
	-ex "printf \"\\n=== stub stopped at pc=0x%08x (want inside $LOAD_ADDR..) ===\\n\", \$pc" \
	-ex "printf \"reading planted word at 0x081E0000 (a bad-ECC word should FAIL):\\n\"" \
	-ex "x/1xw 0x081E0000" \
	-ex "printf \"resetting board into the firmware...\\n\"" \
	-ex "set *(unsigned int *)0xE000ED0C = 0x05FA0004" \
	-ex "quit" || true

echo
echo "Done. If the read above said 'Cannot access memory at 0x081e0000', the"
echo "bad-ECC word is planted. The board was reset; if it didn't reboot on its"
echo "own, power-cycle it and watch the console:"
echo "  non-hardened bootloader: hard faults every boot (BFAR=0x081e0000, find_entry)."
echo "  with ECC scrub:          boots clean, parameter sector erased, params re-seed."
