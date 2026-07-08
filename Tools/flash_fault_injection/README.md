# Flash ECC fault injection (STM32H7)

A bench tool that deterministically plants an **uncorrectable (double-bit) flash ECC error** on an STM32H7 board, to test the bootloader's ECC scrub and the parameter-store recovery path.

## Why this exists

On STM32H7, a torn write to a flash row (power lost mid program/erase) can leave a 256-bit flash word with inconsistent ECC. A CPU read of such a word raises an uncorrectable ECC error, which becomes a precise bus fault. On a board that stores parameters in flash, this hits `find_entry()` during `parameter_flashfs_init()` very early in boot, so the board hard-faults on *every* boot. Reflashing the app or bootloader does not help (neither image touches the parameter sector); only a mass erase recovers it.

The bootloader ECC scrub (`platforms/nuttx/src/bootloader/stm/stm32h7/ecc_scrub.c`) DMA-scans the application and parameter flash before booting the app and erases any sector with an uncorrectable error. This tool plants the exact fault that scrub is meant to recover from, so the scrub and the param re-seed can be validated on hardware.

## How it works

A complete double-program of a flash word does **not** always yield an uncorrectable codeword. The recipe that does (verified on H743): program one 256-bit word twice with no erase between, clearing a *different* single bit each time — `0xFFFFFFFE` then `0xFFFFFFFD`. The stored data becomes `0xFFFFFFFC` but the stored ECC is `ecc(P1) & ecc(P2)`, inconsistent with the data → uncorrectable. (A complementary pair such as `0x55…`/`0xAA…` collapses to data `0` with a *valid* ECC and does not fault.)

`brick_ecc.c` is a tiny freestanding stub: it masks interrupts, unlocks flash bank 2, erases the target sector, double-programs the target word, then halts on a breakpoint. It runs from AXI SRAM because st-link cannot write the Cortex-M7 TCMs over SWD; the board is reset afterwards so clobbering RAM is harmless.

## Requirements

- Target board connected over SWD (ST-Link).
- `stlink-tools` (provides `st-util`).
- `gcc-arm-none-eabi` (provides the compiler, `objcopy`, `nm`).
- An ARM-capable gdb. `gdb-multiarch` is preferred — the 2020-q4 `arm-none-eabi-gdb` links against `libncurses.so.5`, which modern distros do not ship.

```
sudo apt-get install stlink-tools gcc-arm-none-eabi gdb-multiarch
```

## Usage

The defaults target the **ARK_FPV** parameter sector (STM32H743, bank 2, sector 7 @ `0x081E0000`).

```
./brick_ecc.sh
```

The script builds the stub, loads it into RAM, runs it, reads back the planted word (a bad-ECC word reports `Cannot access memory` — that is the expected success indicator), and resets the board. Then power-cycle and watch the NSH console:

- **Non-hardened bootloader:** hard-faults on every boot (`BFAR=0x081e0000`, fault in `find_entry`).
- **Bootloader with the ECC scrub:** boots clean — the scrub erased the parameter sector, and the app re-seeds parameters to defaults.

## Adapting to another board

The defaults are ARK_FPV-specific. For a different H7 target, edit `brick_ecc.c`:

- `ADDR` — a word inside the sector you want to corrupt (e.g. the parameter sector base for that board, from its `sector_map` in `boards/<vendor>/<board>/src/init.c`).
- `SNB` — the sector number within the bank for that address.
- The `FLASH_*2` register addresses target **bank 2**; for a word in bank 1 use the bank-1 registers (`FLASH_KEYR1` `0x52002004`, `FLASH_CR1` `0x5200200C`, `FLASH_SR1` `0x52002010`, `FLASH_CCR1` `0x52002014`).

`LOAD_ADDR`/`STACK` in `brick_ecc.sh` assume free AXI SRAM at `0x24040000`; adjust if your target's RAM map differs.

> ⚠️ This tool **intentionally corrupts flash**. Use it only on a development board you are willing to mass-erase. It is a bench/test helper and is not built or shipped as part of any firmware image.
