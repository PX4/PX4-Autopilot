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
+---------------------------+  APP_LOAD_ADDRESS
| Vector table              |
+---------------------------+  APP_LOAD_ADDRESS + BOARD_IMAGE_TOC_OFFSET
| Image TOC                 |
+---------------------------+
| .text / .rodata / .data   |
+---------------------------+  &_boot_signature
| 64-byte ed25519 signature |
+---------------------------+
```

The **Table of Contents (TOC)** is a small data structure compiled into the firmware that tells the bootloader which region to hash and which key slot to verify against.
Its format is defined in [`src/include/image_toc.h`](https://github.com/PX4/PX4-Autopilot/blob/main/src/include/image_toc.h).

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

## See Also

- [Bootloader Update](../advanced_config/bootloader_update.md)
- [OEM/Factory Configuration](../advanced_config/oem.md)
