# Hardfault Logging on Boards without Usable Non-Volatile Memory

<Badge type="tip" text="PX4 main (v1.19)" />

This topic shows how to setup a flight controller board to persist hardfaults over warm-reset (only) on boards without usable non-volatile storage.
The same mechanism can optionally be used to provide CPU starvation information.

## Overview

Flight controller boards typically log hardfaults to persistent memory, such as Battery-Backed SRAM (BBSRAM), Program memory/on-chip flash (PROGMEM), or Standby SRAM Controller (SSARC).

Boards that don't have this memory can still be configured to persist the fault to a `.noinit` SRAM region that survives warm resets.
On hardfault, `board_crashdump()` then stashes the fault-context to the `.noinit` region and resets.
On the next boot, the [hardfault_stream](../modules/modules_system.md#hardfault-stream) checks for a valid hardfault log and streams it out as a hex dump with a CRC32 over `PX4_INFO`/`PX4_ERR` (picked up by `ORB_ID(log_message)`).
The [`log_message`](../msg_docs/LogMessage.md) is forwarded by the configured logging tool (e.g. DroneCAN).
The captured hex-dump can be parsed into a CrashDebug-compatible coredump or a PX4-style fault log using [Tools/hardfaults_rawhex_decode.py](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/hardfaults_rawhex_decode.py).

The following sections show how to configure the board.
Some settings and instructions are already provided by px4-common-files and are mentioned here only for completeness.

## Configuration

### Hardfault Dumps

The following configuration changes are required to enable crash dumps and streaming.

#### KConfig

The [KConfig board configuration](../hardware/porting_guide_config.md) must include:

```txt
CONFIG_BOARD_CRASHDUMP=y
CONFIG_MODULES_HARDFAULT_STREAM=y
```

The board's `rcS` needs to start the [hardfault-stream](../modules/modules_system.md#hardfault-stream) module.

```sh
hardfault_stream start
```

#### board_config.h

Set these `#DEFINES` in `src/board_config.h`:

```c
#define BOARD_HAS_RAM_HARDFAULT_DUMP
// bytes of user stack to capture, check actual usage with `top`
#define BOARD_RAM_HARDFAULT_USTACK_BYTES 4096
// ISTACK is taken from `CONFIG_ARCH_INTERRUPTSTACK`
```

#### Linker script

Define a memory region in the board's linker script, that survives a warm reset (not zero-initialized) and is big enough for the dump buffer:
`sizeof(px4_ram_hardfault_dump_s)` = `sizeof(uint32_t)` + `sizeof(fullcontext_s)`  
(`fullcontext_s` depends on `BOARD_RAM_HARDFAULT_USTACK_BYTES`), e.g. in `script.ld`:

```txt
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

#### Optional: keep the linker-script size in sync (`cmake/linker_preprocess.cmake`)

Instead of hand-sizing the `noinit` region, a preprocessor-script `${PX4_BOARD_DIR}/cmake/linker_preprocess.cmake`  
can be used to compute and inject the region-size into `script.ld`:

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

### Task/CPU-load streaming (Optional)

Live task/CPU-load dumps can also be published through the same path (unrelated feature, same mechanism) using the [task-watchdog](http://localhost:5173/px4_user_guide/en/modules/modules_system#task-watchdog) module:

The [KConfig board configuration](../hardware/porting_guide_config.md) must include the module:

```txt
CONFIG_MODULES_TASK_WATCHDOG=y
```

The board's `rcS` must include the following line to start the module.

```sh
task_watchdog start
```
