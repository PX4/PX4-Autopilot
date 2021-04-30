1. build saluki bootloader:

- Set PATH to point to a working riscv64-unknown-elf-gcc
- $ make ssrc_saluki_bootloader

2. flash the saluki bootloader to eNVM

<TODO. add instructions to run fpgenprog as root>

$ cp /build/ssrc_saluki_bootloader/ssrc_saluki_bootloader.elf .
$ sudo SC_INSTALL_DIR=<path to SoftConsole-v2021.1> FPGENPROG=<path to Libero_SoC_v2021.1>/Libero/bin64/fpgenprog java -jar $SC_INSTALL_DIR/extras/mpfs/mpfsBootmodeProgrammer.jar --bootmode 1 ssrc_saluki_bootloader.elf

3. use px_uploader.py to flash the px4 binary
