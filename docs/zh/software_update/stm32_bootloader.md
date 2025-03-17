# STM32 Bootloader

The code for the PX4 bootloader is available from the Github [Bootloader](https://github.com/px4/bootloader) repository.

## 支持的飞控板

- FMUv2 (Pixhawk 1, STM32F4)
- FMUv3 (Pixhawk 2, STM32F4)
- FMUv4 (Pixracer 3 and Pixhawk 3 Pro, STM32F4)
- FMUv5 (Pixhawk 4, STM32F7)
- TAPv1 (TBA, STM32F4)
- ASCv1 (TBA, STM32F4)

## 构建 Bootloader

```sh
git clone https://github.com/PX4/Bootloader.git
cd Bootloader
git submodule init
git submodule update
make
```

在此步骤之后，所有支持的主板的 elf 文件范围都出现在引导 Bootloader 目录中。

## 刷写 Bootloader

:::warning
The right power sequence is critical for some boards to allow JTAG / SWD access. 其他 JTAG 仿真器需要不同但相似的步骤。
:::

The instructions below are valid for a Blackmagic / Dronecode probe.
Other JTAG probes will need different but similar steps.
Developers attempting to flash the bootloader should have the required knowledge.
If you do not know how to do this you probably should reconsider if you really need to change anything about the bootloader.

这些指令适用于<a href="https://www.segger.com/jlink-gdb-server.html"> J-Link GDB server</a>。

1. 断开 JTAG 电缆的连接
2. 连接 USB 电源线
3. 连接 JTAG 电缆

### 黑魔法/无人机探测器

#### 使用正确的串行端口

- On LINUX: `/dev/serial/by-id/usb-Black_Sphere_XXX-if00`
- On MAC OS: Make sure to use the cu.xxx port, not the tty.xxx port: `tar ext /dev/tty.usbmodemDDEasdf`

```sh
arm-none-eabi-gdb
  (gdb) tar ext /dev/serial/by-id/usb-Black_Sphere_XXX-if00
  (gdb) mon swdp_scan
  (gdb) attach 1
  (gdb) mon option erase
  (gdb) mon erase_mass
  (gdb) load tapv1_bl.elf
        ...
        Transfer rate: 17 KB/sec, 828 bytes/write.
  (gdb) kill
```

### J-Link

These instructions are for the [J-Link GDB server](https://www.segger.com/jlink-gdb-server.html).

#### 系统必备组件

[Download the J-Link software](https://www.segger.com/downloads/jlink) from the Segger website and install it according to their instructions.

#### 运行 JLink GDB 服务器

常见目标的 <code>--device</code>/SoC是：

```sh
JLinkGDBServer -select USB=0 -device STM32F427VI -if SWD-DP -speed 20000
```

The `--device`/SoC for common targets is:

- **FMUv2, FMUv3, FMUv4, aerofc-v1, mindpx-v2:** STM32F427VI
- **px4_fmu-v4pro:** STM32F469II
- **px4_fmu-v5:** STM32F765II
- **crazyflie:** STM32F405RG

#### 连接 GDB

```sh
arm-none-eabi-gdb
  (gdb) tar ext :2331
  (gdb) load aerofcv1_bl.elf
```

### 故障处理

If any of the commands above are not found, you are either not using a Blackmagic probe or its software is outdated.
Upgrade the on-probe software first.

断开目标连接（同时保持 JTAG 连接）并运行

```
Error erasing flash with vFlashErase packet
```

这将禁用目标供电并尝试另一个闪光周期。

```sh
mon tpwr disable
swdp_scan
attach 1
load tapv1_bl.elf
```

This will disable target powering and attempt another flash cycle.
