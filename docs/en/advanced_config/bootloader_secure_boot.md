# Bootloader Secure Boot

Secure boot is a feature that ensures that only cryptographically authorized PX4 firmware is executed.
This is used by OEMs to ensure that only their validated, tested firmware runs on the vehicle — protecting safety, brand integrity, and regulatory compliance, and stopping others from running unauthorized software on their hardware.

The _PX4 Bootloader_ can verify a cryptographic signature over the PX4 firmware before it is run.
When enabled, only a firmware image signed with a private key whose matching public key is baked into the bootloader will boot.
For any unsigned image, tampered image, or image signed by the wrong key, the device will wait in the bootloader screen (where it can be recovered safely over USB), and will not start the rest of the PX4 flight stack.

::: warning
This feature is intended for OEMs.

If you flash a bootloader that trusts a public key whose private counterpart you have lost, you can no longer sign new firmware for that device and will need a debug probe to recover it.
Keep private keys backed up and never commit them to source control.
:::

## How It Works

PX4's secure boot uses [ed25519](https://en.wikipedia.org/wiki/EdDSA#Ed25519) signatures (via [monocypher](https://monocypher.org)).
The signing key is 32 bytes and the resulting signature is 64 bytes.
Verification is fast (no random number generator is required on the device), and the implementation is small enough to fit in a 128 KB bootloader sector alongside the rest of the bootloader.

A signed firmware image lays out in flash like this:

```txt
+---------------------------+  APP_LOAD_ADDRESS              ─┐
| Vector table              |                                 │
+---------------------------+  APP_LOAD_ADDRESS               │
| Image TOC                 |    + BOARD_IMAGE_TOC_OFFSET     │ BOOT region
+---------------------------+                                 │ (hashed and signed)
| .text / .rodata / .data   |                                 │
+---------------------------+  &_boot_signature              ─┤
| 64-byte ed25519 signature |                                 │ SIG1
+---------------------------+                                ─┘
```

The **Table of Contents (TOC)** is a small data structure compiled into the firmware that tells the bootloader which region to hash and which key slot to verify against.
Its format is defined in [`src/include/image_toc.h`](https://github.com/PX4/PX4-Autopilot/blob/main/src/include/image_toc.h).
For px4_fmu-v6x the TOC declares two entries: **BOOT** (the firmware bytes to verify) and **SIG1** (the 64-byte ed25519 signature to verify them against).

On reset the bootloader reads the TOC at a fixed offset, computes an ed25519 signature over the BOOT region, and compares it to the SIG1 entry.
If the signature verifies it jumps to the app; otherwise it stays in the bootloader and waits for a new upload.

The host-side uploader (`Tools/px4_uploader.py`) also asks the bootloader to verify the signature before sending the final reboot, via a dedicated `VERIFY_SIG` opcode.
This means a signature mismatch is reported as a clean error from `px_uploader.py` instead of a silent "device stays in bootloader after reboot".

## Trying It Out

PX4 ships a secure-boot example for **px4_fmu-v6x**.
This consists of two build variants:

- `px4_fmu-v6x_bootloader_secureboot` — the secure bootloader, with ed25519 verification enabled, and the upstream _test public key_ baked in.
- `px4_fmu-v6x_secureboot` — PX4 firmware, with TOC and automatic signing.

The steps are:

1. Build and flash the secure bootloader (one-time, via SWD)
   - Build the bootloader:

     ```sh
     make px4_fmu-v6x_bootloader_secureboot
     ```

   - Flash the resulting `build/px4_fmu-v6x_bootloader_secureboot/px4_fmu-v6x_bootloader_secureboot.elf` via a debug probe.
     See [Bootloader Update](../advanced_config/bootloader_update).

     ::: tip
     This is the only step that needs SWD — once the secure bootloader is in place, all firmware updates go over USB.
     :::

2. Build and upload signed firmware

   ```sh
   make px4_fmu-v6x_secureboot upload
   ```

   The build produces an unsigned `.bin`, signs it with the upstream test key (`Tools/test_keys/test_keys.json`), wraps it in a `.px4` envelope marked `image_signed: true`, and uploads it.
   You will see something like:

   ```sh
   Verify   ▕██████████████████████████████▏ 100%
   Verifying image signature... passed
   Uploaded in 18s
   ```

If you upload an image that the bootloader can't verify (e.g. an unsigned `.px4`, a tampered `.bin`, or one signed with a different key), `px_uploader.py` reports the failure before reboot:

```sh
Upload failed: Signature verification failed: image will not boot.
The bootloader computed a signature over the flashed image that does not
match any public key it trusts.
```

## Generating Your Own Keys

The default test key is committed to the PX4 tree, so a real deployment must replace it (in order to protect the public key).
Generate a new ed25519 key pair with:

```shell
python3 Tools/secure_bootloader/generate_signing_keys.py /path/to/my_key
```

This writes:

- `my_key.json` — the private key used by `sign_firmware.py`. **Keep this private. Do not commit it.**
- `my_key.pub` — the public key as a C-array fragment, suitable for `#include` in the bootloader's keystore.

Make the following changes use your new keys in the build:

1. **Update the bootloader to trust your public key.**

   Edit `boards/px4/fmu-v6x/bootloader_secureboot.px4board` and change `CONFIG_PUBLIC_KEY0` to point at `my_key.pub`. Rebuild and reflash the bootloader via SWD.

2. **Tell the app build to sign with the matching private key.**

   Either edit `boards/px4/fmu-v6x/secureboot.px4board` and change `CONFIG_BOARD_SECUREBOOT_KEY` to point at `my_key.json`, or set the environment variable at build time:

   ```sh
   BOARD_SECUREBOOT_KEY=/path/to/my_key.json make px4_fmu-v6x_secureboot upload
   ```

The keys used must always be the corresponding cryptographic pair — if you flash a bootloader that trusts a key whose private half you don't have, you can only recover via SWD.

## Enabling Secure Boot on a New Board

To enable secure boot up on a board that doesn't already have a `secureboot` variant, you'll need:

- a `toc.c` file placed in the board's `src/` (modeled on `boards/px4/fmu-v6x/src/toc.c`),
- a linker script with `_main_toc` reserved at a fixed offset past the vector table and a `.signature` section at the end of FLASH (see `boards/px4/fmu-v6x/nuttx-config/scripts/secureboot-script.ld`)
- a `secureboot.px4board` setting `CONFIG_BOARD_SECUREBOOT=y` and the linker prefix,
- a `bootloader_secureboot.px4board` enabling `CONFIG_BOARD_CRYPTO`, `CONFIG_DRIVERS_SW_CRYPTO`, `CONFIG_DRIVERS_STUB_KEYSTORE` and the public-key path,
- `BOOTLOADER_USE_SECURITY` + `BOOTLOADER_SIGNING_ALGORITHM` + `BOARD_IMAGE_TOC_OFFSET` defines in the board's `hw_config.h`, gated on `PX4_CRYPTO`.

The fmu-v6x variant files are kept small and self-contained for exactly this reason — they are intended to be copied as a starting point.

## Multi-Part Signed Images

The layout shown above assumes the whole signed image is flashed to memory-mapped flash at `APP_LOAD_ADDRESS`, where the bootloader can hash it in place.

That assumption doesn't hold for boards that host images on media such as an SD card or eMMC, because the data isn't stored as a contiguous block, and the CPU can't address it directly.
Loading the whole (potentially large) image into RAM so that it can be authenticated is potentially problematic, and certainly inefficient.

To support these kinds of boards, PX4 can instead build a small, **separately signed** TOC block that is prepended to the signed app image on the media.
The bootloader can then load this standalone TOC into a small RAM buffer and authenticate it before anything else is trusted, and without having to load the whole image.
Once verified, the TOC entries tell the bootloader exactly what the rest of the image contains and how each part should be located, loaded, and verified.
In other words, this approach allows the image payload to be staged into memory and verified piece by piece, instead of all at once.

### On-media layout

```txt
+---------------------------+  media offset 0                ─┐
| Standalone TOC block      |                                 │ signed
| + signature               |                                 │ separately
+---------------------------+                                ─┤
| Signed PX4 app image      |                                 │
| (BOOT region + SIG1)      |                                 │ signed app
+---------------------------+                                ─┘
```

The standalone TOC is signed with the **same key** as the app (`CONFIG_BOARD_SECUREBOOT_KEY`, or the `BOARD_SECUREBOOT_KEY` env var).

### Relevant TOC flag

TOC entries normally describe their payload with an absolute address — the address the image was signed at.
That doesn't work for the standalone TOC once it's loaded into a RAM buffer, because the buffer's location is chosen at runtime by the board's bootloader startup code, not fixed at build/sign time.
[`src/include/image_toc.h`](https://github.com/PX4/PX4-Autopilot/blob/main/src/include/image_toc.h) adds a flag so an entry can describe its payload relative to that buffer instead of with an absolute address:

- `TOC_FLAG2_RELATIVE_ADDRESSES` — the entry's `start`/`end` are byte offsets from the base of the buffer the TOC was found in, not absolute addresses.

Set this in your board's `toc.c` on any entry whose payload is staged into a RAM buffer rather than executed in place.

### Bootloader integration

The flag above only helps if the bootloader also knows where that RAM buffer is and how to fill it.
The common PX4 bootloader ([`platforms/nuttx/src/bootloader/common/bl.c`](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/nuttx/src/bootloader/common/bl.c)) exposes two integration points for board-specific startup code to provide that — neither is wired up automatically; a board using non-XIP media must call/override these itself:

- `bl_set_toc_buffer(const void *buf, size_t len)` — tell the bootloader where board startup code staged the TOC block, before entering `bootloader()` / `jump_to_app()` (which then locate and verify it via `find_toc()`).
- Boards where the TOC still sits at `APP_LOAD_ADDRESS` in XIP flash don't need to call this — that's the default `find_toc()` falls back to.
- `BL_TOC_ENTRY_COPY(dst, src, len)` — macro used to copy a verified entry's payload to its target address.
  Defaults to a plain `memcpy()` (evaluating to `0` on success), which is sufficient when the image is memory-mapped.
  Override it in `hw_config.h` when `src` isn't a directly addressable memory location (e.g. a byte offset on SD card/eMMC), so the copy goes through the appropriate storage driver instead.
  The macro must return non-zero on failure.

### Enabling the standalone TOC on a board

Steps 1–3 below are one-time setup for a given board.
Once `src/toc.c` and `nuttx-config/scripts/toc.ld` exist and `CONFIG_BOARD_SECUREBOOT` is enabled, every subsequent build re-runs the build/sign/prepend pipeline below automatically (no separate command is needed):

1. Enable secure boot: `CONFIG_BOARD_SECUREBOOT=y`.
2. Add [`boards/<vendor>/<board>/src/toc.c`](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6x/src/toc.c) declaring the `image_toc_entry_t` table. Entries whose payloads are staged into a RAM buffer should set `TOC_FLAG2_RELATIVE_ADDRESSES` (see [Relevant TOC flag](#relevant-toc-flag) above).
3. Add `boards/<vendor>/<board>/nuttx-config/scripts/toc.ld` — a standalone linker script for the TOC block.
   It should use the `_app_start` / `_app_end` symbols to size the payload; those symbols are computed from the unsigned app `.bin` that is pulled in via `.incbin` by [`platforms/nuttx/toc/fw_image.c`](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/nuttx/toc/fw_image.c).

The build rules in [`platforms/nuttx/toc/CMakeLists.txt`](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/nuttx/toc/CMakeLists.txt) then automatically:

1. Compile `toc.c` into a standalone `board_toc` library (not linked into the app ELF).
2. Link `toc.elf` against `toc.ld`, with the unsigned app `.bin` embedded via `.incbin` so `_app_start` / `_app_end` reflect the real payload size.
3. `objcopy` `toc.elf` → `toc.bin`.
4. Sign `toc.bin` with `Tools/secure_bootloader/sign_firmware.py` using the app-signing key.
5. Prepend the signed `toc_signed.bin` to the signed app inside the `.px4` package.

**If the board's app image lives on media the CPU can't address directly** (SD card, eMMC, external SPI flash, ...), the above is only the build side.
The board's own bootloader startup code is still responsible for loading the signed TOC into a RAM buffer and calling `bl_set_toc_buffer()` to point at it, and for overriding `BL_TOC_ENTRY_COPY()` so payloads can be copied via the storage driver — see [Bootloader integration](#bootloader-integration) above.
Boards where the image stays in XIP flash need none of this.

## See Also

- [Bootloader Update](../advanced_config/bootloader_update.md)
- [OEM/Factory Configuration](../advanced_config/oem.md)
