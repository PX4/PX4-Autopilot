---
applyTo: "src/drivers/**"
---

# PX4 Driver Review Instructions

This file provides guidelines for reviewing driver files in the `src/drivers/` directory.

## Copyright Statements

### Required Files

All source files (`.cpp`, `.c`, `.hpp`, `.h`, `CMakeLists.txt`) MUST include a copyright statement at the top.

**Exceptions:** 
- `Kconfig` files
- `module.yaml` files

### Copyright Format

**For new files (created in 2026):**
```cpp
/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
```

**For updated files (originally created in 2015, updated in 2026):**
```cpp
/****************************************************************************
 *
 *   Copyright (c) 2015-2026 PX4 Development Team. All rights reserved.
 *
 * [... rest of copyright text ...]
 *
 ****************************************************************************/
```

### Key Points
- First year should be the original creation year
- When updating an existing file, update the year range: `YYYY-2026`
- Use the current year (2026 in this case) for new files
- Maintain consistent formatting with other PX4 driver files

---

## Driver Self-Documentation

All drivers MUST be self-documenting through the `print_usage()` method.

### Required Implementation

Every driver `.cpp` file must implement a `print_usage()` method that includes:

1. **PRINT_MODULE_DESCRIPTION macro** - Contains markdown documentation
2. **Module identification** - Using PRINT_MODULE_USAGE_NAME and optionally PRINT_MODULE_USAGE_SUBCATEGORY
3. **Parameter documentation** - Relevant parameters, especially enablement parameters

### PRINT_MODULE_DESCRIPTION Structure

The description should follow this markdown format:

```cpp
PRINT_MODULE_DESCRIPTION(
    R"DESCR_STR(
### Description
[Clear description of what the driver does and its primary functionality]

### Implementation
[Optional: High-level overview of how the driver works]

### Examples
[Optional: CLI usage examples if non-trivial]
$ module_name start -d /dev/ttyS1
$ module_name stop

)DESCR_STR");
```

### Required Sections

#### 1. Description Section
- Brief explanation of the driver's purpose
- Key features/capabilities
- Important parameters (especially enable parameters like `SENS_XX_CFG`)

#### 2. Documentation Links
When applicable, include links to user documentation:
```cpp
Setup/usage information: https://docs.px4.io/main/en/sensor/[sensor-name].html
```

#### 3. Examples Section (when relevant)
Provide CLI usage examples for non-trivial commands:
```cpp
### Examples
Attempt to start driver on a specified serial device.
$ vectornav start -d /dev/ttyS1
Stop driver
$ vectornav stop
```

### Complete Example

```cpp
int MyDriver::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This driver interfaces with the XYZ sensor via I2C/SPI. It provides
distance measurements and is automatically started when SENS_XYZ_CFG
is set to the appropriate value.

Key features:
- Distance range: 0.2m to 50m
- Update rate: up to 100Hz
- I2C and SPI support

Setup/usage information: https://docs.px4.io/main/en/sensor/xyz_sensor.html

### Examples
Start driver on I2C bus 1 with address 0x29:
$ xyz_sensor start -X -b 1 -a 0x29
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("xyz_sensor", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x29);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}
```

### Common Patterns by Driver Type

**For UART/Serial Drivers:**
```cpp
PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "Serial device", true);
```

**For I2C/SPI Drivers:**
```cpp
PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x76);
```

**For Sensor Drivers:**
```cpp
PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");  // or imu, magnetometer, etc.
```

---

## Review Checklist

When reviewing driver files, verify:

- [ ] Copyright header is present (except Kconfig and module.yaml)
- [ ] Copyright year is correct (current year for new files, year range for updates)
- [ ] `print_usage()` method exists
- [ ] `PRINT_MODULE_DESCRIPTION` macro is present with markdown content
- [ ] Description section explains driver purpose clearly
- [ ] Relevant parameters are documented (especially enable parameters)
- [ ] Documentation links are included when available
- [ ] Examples are provided for complex CLI usage
- [ ] Module name and category are correctly specified
- [ ] Standard commands (start, stop, status) are documented

---

## Additional Resources

- Driver template examples: `src/drivers/*/` (various sensor types)
- Module macros: `platforms/common/include/px4_platform_common/module.h`
- Similar drivers for reference patterns (DShot, VectorNav, CrsfRc, etc.)
