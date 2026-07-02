# Hardfault Logging on Boards Without Non-Volatile Memory

This Section introduce how to setup a board-config to persist a hardfault over warm-reset (power-cycle not covered)
on boards without a non-volatile storage (sd-card, BBSRAM, PROGMEM, SSARC, …).
The Hardfault is provided via uORB-log-msg topic after warm-reset.

## Overview

Configuration needed:

- `CONFIG_BOARD_CRASHDUMP=y`
- \#DEFINES in `src/board_config.h`:
  - `BOARD_HAS_RAM_HARDFAULT_DUMP`
  - `BOARD_RAM_HARDFAULT_USTACK_BYTES`
- Memory region in the board's linker script: That survives a warm reset (not zero-initialized) and is big enough for the dump buffer:
  `sizeof(px4_ram_hardfault_dump_s)` (`sizeof(uint32_t)` + `sizeof(fullcontext_s)`, which depends on `BOARD_RAM_HARDFAULT_USTACK_BYTES`)
- Setup module Hardfault_Stream:
  - `CONFIG_MODULES_HARDFAULT_STREAM=y`
  - `hardfault_stream start` in startup script
- Optional: `CONFIG_MODULES_TASK_WATCHDOG=y` to also stream live task/CPU-load dumps through the same path (unrelated feature, same mechanism).
- Optional: `cmake/linker_preprocess.cmake` to keep the linker-script region-size and `BOARD_RAM_HARDFAULT_USTACK_BYTES` in sync automatically, instead of hand-sizing the region. (see example for reference)

## Examples Configuration

#### nuttx-config/<config>/defconfig

```sh
CONFIG_BOARD_CRASHDUMP=y
```

#### src/board_config.h

```c
#define BOARD_HAS_RAM_HARDFAULT_DUMP
#define BOARD_RAM_HARDFAULT_USTACK_BYTES 4096 // bytes of user stack to capture, check actual usage with `top`
// IRAM can be taken from `CONFIG_ARCH_INTERRUPTSTACK`
```
#### nuttx-config/scripts/script.ld

```ld
MEMORY
{
    ...
    noinit (rwx) : ORIGIN = ...,
    		   LENGTH = 8K /* generously-sized, no need to compute exactly */
}
...
.noinit (NOLOAD) : {
	*(.noinit .noinit.*)
} > noinit

/* optional to check it's actual size with `nm` */
_noinit_size = SIZEOF(.noinit);
```

#### `<board>.px4board` And Startup Script

```sh
CONFIG_MODULES_HARDFAULT_STREAM=y
```

```sh
# in the board's rcS, after the log transport is up
hardfault_stream start
```

### Optional:
##### keep the linker-script size in sync (`cmake/linker_preprocess.cmake`)

Instead of hand-sizing the `noinit` region, a preprocessor-script `${PX4_BOARD_DIR}/cmake/linker_preprocess.cmake` can be used compute and inject the region-size into `script.ld`:

```cmake
# >> Values can differ from board to board <<
#
# For <fixed header overhead> compute:
# `src/lib/systemlib/hardfault_log.h`
# BOARD_RAM_HARDFAULT_OVERHEAD at worst-case
# sizeof(uint32_t magic) + sizeof(info_s), chosen from CONFIG_ARCH_FPU:
#   no FPU: XCPTCONTEXT_REGS=19 → sizeof(info_s)=208 → overhead=212
#   FPU:    XCPTCONTEXT_REGS=53 → sizeof(info_s)=344 → overhead=348
#

set(BOARD_RAM_HARDFAULT_USTACK_BYTES 4096) # check for board at runtime eg. `top`
math(EXPR _dump_size "${CONFIG_ARCH_INTERRUPTSTACK} + ${BOARD_RAM_HARDFAULT_USTACK_BYTES} + <fixed header overhead>")

# _ld_script_src/_ld_script_pp: paths to your script.ld and its preprocessed output, set earlier
# preprocess script.ld with -DPX4_NOINIT_DUMP_SIZE=${_dump_size}, use PX4_NOINIT_DUMP_SIZE for the noinit region's LENGTH in script.ld
add_custom_command(
    OUTPUT  "${_ld_script_pp}"
    COMMAND "${CMAKE_C_COMPILER}"
        -E -P -x c
        -DPX4_NOINIT_DUMP_SIZE=${_dump_size}
        "${_ld_script_src}"
        -o "${_ld_script_pp}"
    DEPENDS "${_ld_script_src}"
    COMMENT "Preprocessing script.ld (PX4_NOINIT_DUMP_SIZE=${_dump_size})"
)
add_custom_target(px4_ld_script_pp DEPENDS "${_ld_script_pp}")
add_dependencies(px4 px4_ld_script_pp)

set(_ld_script_flag "-Wl,--script=${_ld_script_pp}")
```

##### task_watchdog

`task_watchdog` (`CONFIG_MODULES_TASK_WATCHDOG=y`)

## What Happens
On hardfault, `the board_crashdump()` stashes the fault-context to the `.noinit` region and resets; on the next boot the [hardfault_stream](../modules/modules_system.md#hardfault-stream) checks for valid hadfault-log and streams it out as a hex dump over `PX4_INFO`/`PX4_ERR` (picked up by `ORB_ID(log_message)`. From there by whatever transport the board forwards log messages over, e.g. DroneCAN). [Tools/hardfaults_rawhex_decode.py](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/hardfaults_rawhex_decode.py) turns a captured hex dump back into a CrashDebug-compatible coredump or a PX4-style fault log.
