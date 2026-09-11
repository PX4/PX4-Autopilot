# CM4 rpmsg remote

Bare-metal Cortex-M4 firmware for the RT1176 second core. It is the device
side of a virtio-rpmsg link whose host is NuttX `rptun` on the CM7
(`platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_rptun.c`).

Runtime (this directory):

- `startup.c`: vectors, FPU, status block and application name below it,
  exception handler that parks instead of locking up.
- `mu.c`: MU-B kicks, coalesced when the mailbox is full.
- `rpmsg_remote.c`: resource table, vrings, name service, endpoint table.
  Provides `rpmsg-ping` (NuttX `rpmsg_ping` echo) and `rpmsg-hello` (text
  echo; `remote_core hello`, shell-timed `remote_core ping`, `remote_core fault`).
  Applications register their own endpoints with `rp_register_service()`
  before `rp_init()`.
- `fault.c`: `!fault <kind>` on the hello endpoint, transport and core
  kinds; a layer adds kinds by defining `fault_inject_app()`. Built with
  `FAULT_INJECT=1` (default), out with `0`.
- `libc.c`: the freestanding subset the image needs.

Layers (`lib/`) are linked by the applications that need them through
`app.mk`; the runtime never references one.

Applications (`apps/<name>/`), one per image, selected by
`CONFIG_BOARD_CM4_APP` in the px4board (Makefile `APP=`); `basic` is the
runtime alone and the default. `remote_core status` prints the name from the
status block. Each application has its own README. `apps/<name>/app.mk`
adds sources, includes, defines and flags; `apps/<name>/deps.cmake` fetches
what they compile.

Memory (`memmap.h`, shared with `src/board_config.h` and preprocessed into `link.ld`):

| CM4 address | CM7 address | Use |
|---|---|---|
| 0x1FFE0000, 64K | 0x20200000 | code |
| 0x1FFF0000, 64K | 0x20210000 | `.resource_table`, vrings (16 x 1 KB each way), rpmsg buffers, application name, status block |
| 0x20000000, 128K | (not needed) | bss, stack |

Build and run:

`make px4_fmu-v6xrt_rpmsg` builds this directory with the firmware toolchain
(`CMakeLists.txt` drives `Makefile`) and places `cm4_rpmsg.elf` in the ROMFS
at `/etc/extras/cm4_rpmsg.elf`, where `rptun` loads it from. Flash as usual:

```
nsh> remote_core start                    # loads the ELF from ROMFS, releases the CM4 (autostarted)
nsh> remote_core status                   # reset state, status block, application name
nsh> remote_core ping 100 64              # rtt min/avg/max in the shell
nsh> remote_core hello "ping"
nsh> remote_core fault list               # fault injection kinds
nsh> rptun ping /dev/rptun/cm4 100 64 3 0 # NuttX driver ping (ack+check); output via syslog on LPUART1
nsh> rptun dump all                       # vring and endpoint state, via syslog on LPUART1
```

Standalone build: `make -C boards/px4/fmu-v6xrt/cm4 BUILDDIR=/tmp/cm4 APP=basic`.
To load from another filesystem instead, change `BOARD_CM4_FIRMWARE` in
`src/board_config.h`.

The CM4 waits for the host to set the virtio `DRIVER_OK` status in the
resource table before touching the vrings, so it may be released before or
after the host finishes setup. Kicks travel over MU channel 0 in both
directions; the value is the vring notify id but either side rescans both rings.
