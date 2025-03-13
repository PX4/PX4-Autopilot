# Hard Fault Debugging

A hard fault is a state when a CPU executes an invalid instruction or accesses an invalid memory address.
This might occur when key areas in RAM have been corrupted.

## 비디오

The following video demonstrates hardfault debugging on PX4 using Eclipse and a JTAG debugger.
It was presented at the PX4 Developer Conference 2019.

<lite-youtube videoid="KZkAM_PVOi0" title="Hardfault debugging on PX4"/>

## Debugging Hard Faults in NuttX

A typical scenario that can cause a hard fault is when the processor overwrites the stack and then the processor returns to an invalid address from the stack.
This may be caused by a bug in code were a wild pointer corrupts the stack, or another task overwrites this task's stack.

- NuttX maintains two stacks: The IRQ stack for interrupt processing and the user stack
- The stack grows downward.
  So the highest address in the example below is 0x20021060, the size is 0x11f4 (4596 bytes) and consequently the lowest address is 0x2001fe6c.

```sh
Assertion failed at file:armv7-m/up_hardfault.c line: 184 task: ekf_att_pos_estimator
sp:     20003f90
IRQ stack:
  base: 20003fdc
  size: 000002e8
20003f80: 080d27c6 20003f90 20021060 0809b8d5 080d288c 000000b8 08097155 00000010
20003fa0: 20003ce0 00000003 00000000 0809bb61 0809bb4d 080a6857 e000ed24 080a3879
20003fc0: 00000000 2001f578 080ca038 000182b8 20017cc0 0809bad1 20020c14 00000000
sp:     20020ce8
User stack:
  base: 20021060
  size: 000011f4
20020ce0: 60000010 2001f578 2001f578 080ca038 000182b8 0808439f 2001fb88 20020d4c
20020d00: 20020d44 080a1073 666b655b 65686320 205d6b63 6f6c6576 79746963 76696420
20020d20: 65747265 63202c64 6b636568 63636120 63206c65 69666e6f 08020067 0805c4eb
20020d40: 080ca9d4 0805c21b 080ca1cc 080ca9d4 385833fb 38217db9 00000000 080ca964
20020d60: 080ca980 080ca9a0 080ca9bc 080ca9d4 080ca9fc 080caa14 20022824 00000002
20020d80: 2002218c 0806a30f 08069ab2 81000000 3f7fffec 00000000 3b4ae00c 3b12eaa6
20020da0: 00000000 00000000 080ca010 4281fb70 20020f78 20017cc0 20020f98 20017cdc
20020dc0: 2001ee0c 0808d7ff 080ca010 00000000 3f800000 00000000 080ca020 3aa35c4e
20020de0: 3834d331 00000000 01010101 00000000 01010001 000d4f89 000d4f89 000f9fda
20020e00: 3f7d8df4 3bac67ea 3ca594e6 be0b9299 40b643aa 41ebe4ed bcc04e1b 43e89c96
20020e20: 448f3bc9 c3c50317 b4c8d827 362d3366 b49d74cf ba966159 00000000 00000000
20020e40: 3eb4da7b 3b96b9b7 3eead66a 00000000 00000000 00000000 00000000 00000000
20020e60: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
20020e80: 00000016 00000000 00000000 00010000 00000000 3c23d70a 00000000 00000000
20020ea0: 00000000 20020f78 00000000 2001ed20 20020fa4 2001f498 2001f1a8 2001f500
20020ec0: 2001f520 00000003 2001f170 ffffffe9 3b831ad2 3c23d70a 00000000 00000000
20020ee0: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
20020f00: 00000000 00000000 00000000 00000000 2001f4f0 2001f4a0 3d093964 00000001
20020f20: 00000000 0808ae91 20012d10 2001da40 0000260b 2001f577 2001da40 0000260b
20020f40: 2001f1a8 08087fd7 08087f9d 080cf448 0000260b 080afab1 080afa9d 00000003
20020f60: 2001f577 0809c577 2001ed20 2001f4d8 2001f498 0805e077 2001f568 20024540
20020f80: 00000000 00000000 00000000 0000260b 3d093a57 00000000 2001f540 2001f4f0
20020fa0: 0000260b 3ea5b000 3ddbf5fa 00000000 3c23d70a 00000000 00000000 000f423f
20020fc0: 00000000 000182b8 20017cc0 2001ed20 2001f4e8 00000000 2001f120 0805ea0d
20020fe0: 2001f090 2001f120 2001eda8 ffffffff 000182b8 00000000 00000000 00000000
20021000: 00000000 00000000 00000009 00000000 08090001 2001f93c 0000000c 00000000
20021020: 00000101 2001f96c 00000000 00000000 00000000 00000000 00000000 00000000
20021040: 00000000 00000000 00000000 00000000 00000000 0809866d 00000000 00000000
R0: 20000f48 0a91ae0c 20020d00 20020d00 2001f578 080ca038 000182b8 20017cc0
R8: 2001ed20 2001f4e8 2001ed20 00000005 20020d20 20020ce8 0808439f 08087c4e
xPSR: 61000000 BASEPRI: 00000000 CONTROL: 00000000
EXC_RETURN: ffffffe9
```

To decode the hard fault, load the _exact_ binary into the debugger:

```sh
arm-none-eabi-gdb build/px4_fmu-v2_default/px4_fmu-v2_default.elf
```

Then in the GDB prompt, start with the last instructions in R8, with the first address in flash (recognizable because it starts with `0x080`, the first is `0x0808439f`).
The execution is left to right. So one of the last steps before the hard fault was when `mavlink_log.c` tried to publish something,

```sh
(gdb) info line *0x0808439f
Line 77 of "../src/modules/systemlib/mavlink_log.c" starts at address 0x8084398 <mavlink_vasprintf+36>
   and ends at 0x80843a0 <mavlink_vasprintf+44>.
```

```sh
(gdb) info line *0x08087c4e
Line 311 of "../src/modules/uORB/uORBDevices_nuttx.cpp"
   starts at address 0x8087c4e <uORB::DeviceNode::publish(orb_metadata const*, void*, void const*)+2>
   and ends at 0x8087c52 <uORB::DeviceNode::publish(orb_metadata const*, void*, void const*)+6>.
```
