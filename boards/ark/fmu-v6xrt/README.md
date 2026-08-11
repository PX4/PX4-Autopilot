# ARK FMU-v6X-RT

NXP i.MX RT1176: Cortex-M7 at 996 MHz, plus a Cortex-M4 that PX4 does not use. **No internal flash** — it runs XIP out of a 64 MB external octal NOR (Macronix MX25UM51345G) on FlexSPI1.

## Architecture, if you are coming from an STM32

| | STM32H7 / F4 | i.MX RT1176 |
|---|---|---|
| Code store | internal flash at `0x08000000` | **external** NOR, XIP-mapped at `0x30000000` over FlexSPI1 |
| Boot | core fetches the vector table from the start of flash | boot ROM reads an **FCB** at `+0x400` to set FlexSPI up, then an **IVT** at `+0x1000` to find the entry point |
| Erase unit | 128 KB sectors (H7) | 4 KB sectors |
| RAM | SRAM banks at fixed sizes | ITCM `0x00000000` and DTCM `0x20000000` (256 KB each) carved out of FlexRAM, plus OCRAM at `0x20240000` |
| Clocks | RCC | CCM, one configurable root clock per peripheral |
| Parameters | an internal flash sector | FRAM (FM25V02A, 32 KB) on **FlexSPI2** |
| Flashing | `st-flash`, DFU, ROM bootloader | needs a target-side flash algorithm — nothing writes the NOR without one |

Three consequences worth internalising:

**The NOR is the only code store.** There is no internal flash to fall back on and no ROM DFU over the debug port. If the NOR ends up in a mode the boot ROM cannot read, the board looks completely dead — no console, no USB, about a third of normal idle current — while SWD still works fine. That is recoverable, not bricked.

**FlexSPI is configured twice, differently.** The boot ROM brings the NOR up in plain 1-pad SPI from the FCB. PX4 then re-initialises it into octal DDR at 200 MHz for XIP speed (`imxrt_octl_flash_initialize()` in `src/init.c`, on in both defconfigs), which includes pointing the read sampler at the flash's DQS pad. A debugger's flash algorithm expects the reset-state controller and hangs against the octal-DDR one, which is why flashing has a preconditioning step.

**Both images carry their own FCB and IVT**, so each is a standalone bootable blob written at its base address — there is no separate header to flash.

### Flash layout

| region | address | size |
|---|---|---|
| bootloader | `0x30000000` | 128 KB |
| application | `0x30020000` | 4 MB − 128 KB (of 64 MB; the rest is unused) |

## Dependencies

- **Toolchain:** `arm-none-eabi-gcc` 13.2.1, the standard PX4 v1.18 toolchain. Nothing board-specific.
- **Probe:** an NXP MCU-Link (`1fc9:0143`). It ships with a JST-GH DCD-LZ cable that mates with the FC's debug connector directly, and its VCOM is the NSH console at 57600.
- **pyOCD**, from pip — *not* a distro package, which tends to omit `cmsis-pack-manager` and then cannot install device packs:

  ```bash
  pip install pyocd
  ```

- **NXP's device family pack**, which is where the RT1176 flash algorithm lives (~27 MB, cached under `~/.local/share/cmsis-pack-manager`):

  ```bash
  pyocd pack install mimxrt1176
  ```

  pyOCD's *built-in* `mimxrt1170_cm7` target is not an alternative for flashing — its algorithm hangs in `Init` on this part. It is still needed for the preconditioning step below, so both get used.

- **Non-root access to the probe:**

  ```
  # /etc/udev/rules.d/60-mcu-link.rules
  ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="0143", MODE="0666", GROUP="plugdev"
  ```

## Building

```bash
make ark_fmu-v6xrt_bootloader     # build/ark_fmu-v6xrt_bootloader/ark_fmu-v6xrt_bootloader.bin
make ark_fmu-v6xrt_default        # build/ark_fmu-v6xrt_default/ark_fmu-v6xrt_default.bin (and .px4)
```

Flash the raw `.bin`. The `.px4` is a packaged container for the USB uploader and is not what SWD wants.

## Flashing over SWD

Once a bootloader is on the board, the usual `make ark_fmu-v6xrt_default upload` over USB works and is faster. SWD is what you need for a bare board, a wiped NOR, or replacing the bootloader itself.

**1. Precondition — every session, before any flash.**

```bash
pyocd cmd -t mimxrt1170_cm7 -f 4000k -O resume_on_disconnect=false \
  -c "read32 0x400CC000" -c "write32 0x400CC000 0xffffa700"
```

`0x400CC000` is FlexSPI1's `MCR0`. Reading `0xffffa030` means PX4 has the NOR in octal DDR with reads sampled off the DQS pad; the write clears that sampling source, which is what the flash algorithm cannot survive. This also leaves the core **halted**, separately required — the pack target's CM4 reset-catch fails `No ACK` against a running PX4.

**2. Flash.**

```bash
# bootloader, ~3 s
pyocd flash -t mimxrt1176dvmaa -f 4000k -O resume_on_disconnect=false \
  --format bin --base-address 0x30000000 \
  build/ark_fmu-v6xrt_bootloader/ark_fmu-v6xrt_bootloader.bin

# application, ~44 s
pyocd flash -t mimxrt1176dvmaa -f 4000k -O resume_on_disconnect=false \
  --format bin --base-address 0x30020000 \
  build/ark_fmu-v6xrt_default/ark_fmu-v6xrt_default.bin
```

**3. Boot it.**

```bash
pyocd reset -t mimxrt1170_cm7 -f 4000k -O reset_type=hw
```

### Gotchas

- **Two targets, on purpose.** `mimxrt1170_cm7` for preconditioning and reset, the pack's `mimxrt1176dvmaa` for flashing. Neither does both.
- **`reset_type=hw` is not optional.** pyOCD's NXP connect leaves boot-mode bits set in `SRC_SBMR` that a core reset does not clear, and the board then sits halted instead of booting. Only NRST reloads them from the boot pins.
- **`programmed 0 bytes … identical` is success**, not a no-op failure — pyOCD compared the image against the NOR and found it already there. Add `-O smart_flash=false` to force a real erase and program.
- **No `--script` on the pack target.** A user-script delegate makes its connect fail with `Unexpected ACK '0'`.
- **A flaky USB cable to the probe looks exactly like a target that hangs.** One that dropped roughly 1 packet in 35 000 — still enumerating at 480 Mbit/s, nothing in `dmesg` — presented as random sector erases wedging the target, and cost weeks of chasing FlexSPI and the second core. If erases start failing at random, soak the probe's link before suspecting the board.
