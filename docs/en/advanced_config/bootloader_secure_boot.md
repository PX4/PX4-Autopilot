# Secure Boot and Firmware Signature Verification

PX4 includes support for secure boot, which prevents unauthorized or tampered firmware from running on the flight controller.
This feature uses cryptographic signature verification to ensure only trusted firmware can be executed.

::: warning
Secure boot is an advanced feature primarily intended for commercial applications and OEM manufacturers who need to protect their firmware from tampering.
Improper configuration can brick your device.
:::

## Overview

The PX4 secure boot implementation provides:

- **Firmware signature verification**: The bootloader verifies cryptographic signatures before executing firmware.
- **Tamper protection**: Unsigned or improperly signed firmware will not be executed.
- **Key-based authentication**: Uses cryptographic keys stored in the device.
- **Table of Contents (TOC)**: A structured format for organizing firmware components with security metadata.

The secure boot feature is available on NuttX-based flight controllers.

## Enabling Secure Boot

### Prerequisites

Before enabling secure boot, you need:

1. A board configuration that supports secure boot (NuttX-based flight controllers)
2. Understanding of the build system and firmware signing process
3. Secure storage for your private keys
4. A workflow for signing firmware images

### Board Configuration

To enable secure boot on your board:

1. **Enable security in bootloader**: Add to your board's bootloader configuration file (e.g., `boards/[vendor]/[board]/nuttx-config/bootloader/defconfig`):

   ```sh
   CONFIG_BOOTLOADER_USE_SECURITY=y
   ```

2. **Set crypto algorithm**: Define the signing algorithm in your board's `hw_config.h`:

   ```c
   #define BOOTLOADER_SIGNING_ALGORITHM CRYPTO_ED25519
   ```

3. **Configure key storage**: Implement or configure the keystore backend for your board
4. **Build bootloader with crypto support**: Ensure `PX4_CRYPTO` is defined in your board's CMake configuration

### Key Management

::: warning CRITICAL
Keep your private keys secure! If private keys are lost, you cannot sign new firmware. If private keys are compromised, unauthorized firmware can be created.
:::

Key management involves:

1. **Key generation**: Generate cryptographic key pairs (public and private keys)
2. **Key storage**:
   - Private keys: Store securely on your build/signing server (never on the device)
   - Public keys: Store in the device's keystore (accessible to bootloader)
3. **Key indices**: Each key has an index (0-15) used to reference it in the TOC

#### Generating ED25519 Keys

You can use various tools to generate ED25519 keys.
Example using Python with the `cryptography` library:

```python
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives import serialization

# Generate private key
private_key = Ed25519PrivateKey.generate()

# Get public key
public_key = private_key.public_key()

# Serialize private key
private_bytes = private_key.private_bytes(
    encoding=serialization.Encoding.Raw,
    format=serialization.PrivateFormat.Raw,
    encryption_algorithm=serialization.NoEncryption()
)

# Serialize public key
public_bytes = public_key.public_bytes(
    encoding=serialization.Encoding.Raw,
    format=serialization.PublicFormat.Raw
)

# Save keys to files
with open('private_key.bin', 'wb') as f:
    f.write(private_bytes)

with open('public_key.bin', 'wb') as f:
    f.write(public_bytes)
```

#### Installing Public Keys on Device

Public keys must be installed in the device's keystore. The exact method depends on your board's keystore implementation:

- Some boards use One-Time Programmable (OTP) memory
- Some boards use secure flash storage
- Some implementations use a stub keystore for development/testing

Refer to your board's keystore implementation in `src/drivers/stub_keystore/` or board-specific keystore drivers.

### Firmware Signing Process

The firmware signing process involves:

1. **Build firmware**: Build your PX4 firmware as usual
2. **Create TOC**: Generate the Table of Contents structure
3. **Sign firmware**: Sign the firmware image with your private key
4. **Append signature**: Add the signature to the firmware image according to TOC structure
5. **Flash to device**: Upload the signed firmware to the device

::: info
The exact tooling for firmware signing depends on your board and security requirements. Custom scripts are typically needed for production use.
:::

### TOC Creation Example

The TOC structure typically looks like this:

```
[TOC Start Magic]
[TOC Version]
[TOC Entry 0: Firmware]
  - Name: "APP"
  - Start: <firmware start address>
  - End: <firmware end address>
  - Signature Index: 1
  - Key Index: 0
  - Flags: TOC_FLAG1_BOOT | TOC_FLAG1_CHECK_SIGNATURE
[TOC Entry 1: Signature]
  - Name: "SIG"
  - Start: <signature start address>
  - End: <signature end address>
  - Flags: 0
[TOC End Magic]
[Firmware Binary Data]
[Signature Data]
```

## Security Considerations

### Best Practices

1. **Key Management**:
   - Use hardware security modules (HSM) for key storage when possible
   - Never commit private keys to version control
   - Use separate keys for development and production
   - Implement key rotation policies

2. **Build Security**:
   - Sign firmware in a secure, controlled environment
   - Verify signatures before distribution
   - Use checksums for firmware integrity verification
   - Maintain an audit log of signed firmware versions

3. **Device Security**:
   - Use OTP or secure flash for storing public keys
   - Consider using R&D certificates for development devices (allows unsigned boot)
   - Implement secure firmware update mechanisms
   - Monitor for signature verification failures

### Development vs Production

For development, you can use the R&D Certificate feature:

```c
// In TOC, mark entry with RDCT flag
toc_entry.flags1 |= TOC_FLAG1_RDCT;

// R&D certificate capabilities
#define RDCT_CAPS0_ALLOW_UNSIGNED_BOOT 0x1
```

R&D certificates allow specific devices (identified by UUID) to boot unsigned firmware, useful during development without compromising production security.

::: warning
Never deploy R&D certificates to production devices!
:::

## Firmware Updates

### Updating Signed Firmware

When using secure boot, firmware updates must:

1. Be properly signed with the correct private key
2. Include a valid TOC structure
3. Use the same key index that matches the public key in the device

The update process through QGroundControl or other tools works the same way, but the firmware file must be properly signed before upload.

### Signature Verification Failures

If signature verification fails:

- The bootloader will not execute the firmware.
- The bootloader remains active and accepts new firmware uploads.
- The device will not be bricked (bootloader always runs).
- You can upload a correctly signed firmware to recover.

Check bootloader logs (if available) to diagnose signature failures.

## Troubleshooting

### Common Issues

#### Firmware won't boot after enabling secure boot

- Ensure the firmware is properly signed
- Verify the TOC structure is correct
- Check that the key index in TOC matches the keystore

#### Signature verification always fails

- Confirm public key is correctly installed in keystore
- Verify the signature was created with the matching private key
- Check that the signature algorithm matches (e.g., ED25519)
- Ensure the signature covers the correct data range

#### Device appears bricked

- The bootloader should always be accessible
- Use a debug probe to reprogram the bootloader if needed
- Check that BOOTLOADER_USE_SECURITY is defined correctly

### Debug Information

To debug secure boot issues:

1. Enable bootloader debug output (if supported by your board)
2. Use a debug probe to examine memory and execution flow
3. Verify TOC parsing with debug tools
4. Check return values from `verify_app()` function

## How It Works

### Boot Process

1. **Bootloader starts**: The secure bootloader runs first after power-on or reset.
2. **TOC validation**: Locates and validates the Firmware Table of Contents (TOC) structure in flash.
3. **Signature verification**: Verifies the cryptographic signature of the firmware using stored keys.
4. **Boot decision**:
   - If verification succeeds, the firmware is executed.
   - If verification fails, the bootloader remains active and waits for a valid firmware upload.

### Firmware Table of Contents (TOC)

The firmware TOC is a data structure embedded in the firmware that describes:

- Firmware components and their locations in flash memory
- Signature information for each component
- Encryption settings (if used)
- Boot flags indicating which component should be executed

The TOC format is defined in `src/include/image_toc.h`:

```c
typedef struct image_toc_entry {
    unsigned char name[4];      // Name of the section
    const void *start;          // Start address in flash
    const void *end;            // End address in flash
    void *target;               // Copy target address (for RAM execution)
    uint8_t signature_idx;      // Index to signature in TOC
    uint8_t signature_key;      // Key index for signature verification
    uint8_t encryption_key;     // Key index for encryption (future)
    uint8_t flags1;             // Control flags
    uint32_t reserved;          // Reserved for future use
} image_toc_entry_t;
```

### Supported Cryptographic Algorithms

PX4's crypto backend supports multiple algorithms (defined in `platforms/common/include/px4_platform_common/crypto_algorithms.h`):

- **ED25519**: EdDSA signatures using Curve25519 (recommended for signatures)
- **RSA-OAEP**: RSA with Optimal Asymmetric Encryption Padding
- **XCHACHA20**: Stream cipher for encryption (future use)
- **AES**: Block cipher for encryption (future use)

The bootloader uses ED25519 by default for signature verification due to its security, performance, and small signature size.

## Implementation Details

The secure boot implementation consists of:

- **Bootloader**: `platforms/nuttx/src/bootloader/common/`
  - `bl.c`: Main bootloader logic
  - `image_toc.c`: TOC parsing and validation
  - `crypto.c`: Cryptographic operation wrappers

- **Crypto Backend**: `src/drivers/sw_crypto/`
  - `crypto.c`: Software crypto implementation using monocypher and libtomcrypt

- **Keystore**: `src/drivers/stub_keystore/` (or board-specific implementations)
  - Key storage and retrieval

- **Headers**:
  - `src/include/image_toc.h`: TOC structure definitions
  - `platforms/common/include/px4_platform_common/crypto_backend.h`: Crypto API
  - `platforms/common/include/px4_platform_common/crypto_algorithms.h`: Algorithm definitions

## Related Topics

- [Bootloader Update](bootloader_update.md) - Updating the bootloader itself
- [Building PX4 Software](../dev_setup/building_px4.md) - Build system information
- [Board Support Guide](../hardware/board_support_guide.md) - Creating custom boards

## References

- Original implementation: [PR #17672](https://github.com/PX4/PX4-Autopilot/pull/17672)
- Contributor: Technology Innovation Institute
- Monocypher library: https://monocypher.org/
- LibTomCrypt library: https://www.libtom.net/LibTomCrypt/
