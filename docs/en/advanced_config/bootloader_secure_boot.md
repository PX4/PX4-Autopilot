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

<Badge type="tip" text="main (PX4 v2.0)" />

The layout shown above assumes the whole signed image is flashed to memory-mapped (execute-in-place, or XIP) flash at `APP_LOAD_ADDRESS`, where the bootloader can hash it in place.

That assumption doesn't hold for boards that host images on media such as an SD card or eMMC, because the data isn't stored as a contiguous block, and the CPU can't address it directly.
Loading the whole (potentially large) image into RAM just to authenticate it would be inefficient, and may not be possible.

To support these kinds of boards, PX4 can instead build a small, separately signed TOC block that is prepended to the signed app image on the media.
The bootloader can then authenticate this standalone TOC (which board startup code loads into a small RAM buffer) before anything else is trusted, and without having to load the whole image.
Once verified, the TOC entries tell the bootloader exactly what the rest of the image contains and how each part should be located, loaded, and verified.
In other words, this approach allows the image payload to be staged into memory and verified piece by piece, instead of all at once.

PX4 includes an example for `px4_fmu-v6xrt`: `px4_fmu-v6xrt_bootloader_secureboot` (the secure bootloader) and `px4_fmu-v6xrt_secureboot` (firmware with a prepended, separately signed TOC).
On this board the image stays in XIP flash, so it shows the build side and TOC layout without any board-specific loading code.

### On-Media Layout

```txt
+---------------------------+  media offset 0                ─┐
| Standalone TOC block      |                                 │ signed
| + signature               |                                 │ separately
+---------------------------+                                ─┤
| Signed PX4 app image      |                                 │
| (BOOT region + SIG1)      |                                 │ signed app
+---------------------------+                                ─┘
```

The standalone TOC is signed with the same key as the app (`CONFIG_BOARD_SECUREBOOT_KEY`, or the `BOARD_SECUREBOOT_KEY` environment variable).

::: warning
For multi-part images, the upload-time signature check (`VERIFY_SIG`) only verifies the standalone TOC.
The other images are verified when the bootloader boots them, because they can be flashed separately (for example, certificates updated later).
An app image with a bad signature therefore passes the upload check ("Verifying image signature... passed"), but the device stays in the bootloader after reboot.
:::

### Relevant TOC Flag

TOC entries normally describe their payload with an absolute address — the address the image was signed at.
That doesn't work for the standalone TOC once it's loaded into a RAM buffer, because the buffer's location is chosen at runtime by the board's bootloader startup code, not fixed at build/sign time.
[`src/include/image_toc.h`](https://github.com/PX4/PX4-Autopilot/blob/main/src/include/image_toc.h) defines a flag so that an entry can describe its payload relative to that buffer instead of with an absolute address:

- `TOC_FLAG2_RELATIVE_ADDRESSES` — the entry's `start`/`end` are byte offsets from the base of the buffer the TOC was found in, not absolute addresses.

Set this flag in your board's `toc.c` on the standalone TOC's own entry (entry 0) and on its signature entry.
The bootloader only applies it to those two entries: the entries after them are always resolved as absolute addresses.
Entry 0's `start` can't be greater than `BOARD_IMAGE_TOC_OFFSET` (the TOC's offset within the buffer), so it is `0` when the TOC is at the start of the block, as on fmu-v6xrt.

Payloads that must be staged into RAM before use are marked with `TOC_FLAG1_COPY` and a `target` address instead.
The bootloader copies each such payload from its `start` address to `target` (using `BL_TOC_ENTRY_COPY()`, below) and then verifies it there.

### Bootloader Integration

The flag above only helps if the bootloader also knows where that RAM buffer is and how to fill it.
The common PX4 bootloader ([`platforms/nuttx/src/bootloader/common/bl.c`](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/nuttx/src/bootloader/common/bl.c)) provides the following integration points for board-specific code.
None of them is set up automatically: a board that stores its image on non-XIP media must call, override, or define them itself.

- `bl_set_toc_buffer(const void *buf, size_t len)` — function that tells the bootloader where board startup code staged the TOC block, before entering `bootloader()` / `jump_to_app()` (which then locate and verify it via `find_toc()`).
  Boards where the TOC sits at `APP_LOAD_ADDRESS` in XIP flash don't need to call this, since that is the default buffer the bootloader uses.
- `BL_TOC_ENTRY_COPY(dst, src, len)` — macro used to copy the payload of each `TOC_FLAG1_COPY` entry to its `target` address, before that payload's signature is checked.
  Defaults to a plain `memcpy()`, which is sufficient when the image is in XIP flash.
  Override it in `hw_config.h` to define how the copy is performed, so that neither the source nor the target address needs to be directly addressable (for example, `src` can be a byte offset on SD card/eMMC that is read through the storage driver).
  The bootloader doesn't check the macro's result, so a failed copy is only caught if the entry has a signature check (`TOC_FLAG1_CHECK_SIGNATURE`).
- `BOOTLOADER_SKIP_VERIFY_SIG`: define in `hw_config.h` when the full image isn't in addressable memory after upload (for example, when it is written to an SD card in chunks).
  The bootloader then replies OK to the uploader's `VERIFY_SIG` request without checking anything, so the uploader reports "passed" even though no check was done.
  The image is still verified before it boots.

### Enabling the Standalone TOC on a Board

The standalone TOC changes some of the steps in [Enabling Secure Boot on a New Board](#enabling-secure-boot-on-a-new-board).
The following steps are one-time setup for a given board, and the fmu-v6xrt files linked below can be copied as a starting point:

1. Add a `secureboot.px4board` that sets `CONFIG_BOARD_SECUREBOOT=y` and the linker prefix, as for the embedded TOC.
2. Add an app linker script that moves the flash `ORIGIN` past the TOC block (see [`boards/px4/fmu-v6xrt/nuttx-config/scripts/secureboot-script.ld`](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6xrt/nuttx-config/scripts/secureboot-script.ld)).
   Unlike the embedded TOC, it doesn't reserve a `_main_toc` slot in the app.
3. Add [`boards/<vendor>/<board>/src/toc.c`](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6xrt/src/toc.c) declaring the `image_toc_entry_t` table.
   The TOC entry and its signature entry set `TOC_FLAG2_RELATIVE_ADDRESSES` (see [Relevant TOC Flag](#relevant-toc-flag) above).
   Don't add `toc.c` to `drivers_board` in the board's `src/CMakeLists.txt`: the build compiles it separately (see below).
4. Add [`boards/<vendor>/<board>/nuttx-config/scripts/toc.ld`](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6xrt/nuttx-config/scripts/toc.ld) — a standalone linker script for the TOC block.
   It should use the `_app_start` / `_app_end` symbols to size the payload; those symbols are computed from the unsigned app `.bin` that is pulled in via `.incbin` by [`platforms/nuttx/toc/fw_image.c`](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/nuttx/toc/fw_image.c).
   The app address it uses (`_app_addr` on fmu-v6xrt) must match the flash `ORIGIN` in the app linker script.
5. Define `BOOTLOADER_USE_SECURITY`, `BOOTLOADER_SIGNING_ALGORITHM` and `BOARD_IMAGE_TOC_OFFSET` in the board's `hw_config.h`, gated on `PX4_CRYPTO`.
   Set `BOARD_IMAGE_TOC_OFFSET` to `0x0`, because `toc.ld` places the TOC at the start of the block.
6. Add a `bootloader_secureboot.px4board`, as for the embedded TOC, and make sure the board's `src/CMakeLists.txt` builds the bootloader for any label that starts with `bootloader`.

Once `src/toc.c` and `nuttx-config/scripts/toc.ld` exist and `CONFIG_BOARD_SECUREBOOT` is enabled, every subsequent build re-runs the build/sign/prepend pipeline below automatically (no separate command is needed).

The build rules in [`platforms/nuttx/toc/CMakeLists.txt`](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/nuttx/toc/CMakeLists.txt) then automatically:

1. Compile `toc.c` into a standalone `board_toc` library (not linked into the app ELF).
2. Link `toc.elf` against `toc.ld`, with the unsigned app `.bin` embedded via `.incbin` so `_app_start` / `_app_end` reflect the real payload size.
3. Convert `toc.elf` to `toc.bin` with `objcopy`.
4. Sign `toc.bin` with `Tools/secure_bootloader/sign_firmware.py` using the app-signing key.
5. Prepend the signed `toc_signed.bin` to the signed app inside the `.px4` package.

::: tip
If the board's app image lives on media the CPU can't address directly (SD card, eMMC, external SPI flash, and so on), the steps above only cover the build side.
The board's own bootloader startup code is still responsible for loading the signed TOC into a RAM buffer, calling `bl_set_toc_buffer()` to point at it, and overriding `BL_TOC_ENTRY_COPY()` so payloads are copied through the storage driver.
See [Bootloader Integration](#bootloader-integration) above.
Boards where the image stays in XIP flash need none of this.
:::

## See Also

- [Bootloader Update](../advanced_config/bootloader_update.md)
- [OEM/Factory Configuration](../advanced_config/oem.md)
