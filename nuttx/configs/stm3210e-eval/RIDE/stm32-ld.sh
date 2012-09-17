#!/bin/sh

GNUPATH="/cygdrive/c/Program Files/Raisonance/Ride/arm-gcc/bin"
RIDEPATH="/cygdrive/c/Program Files/Raisonance/Ride/Bin"

LD=arm-none-eabi-gcc.exe 
OBJCOPY=arm-none-eabi-objcopy.exe
REXRDR=rexrdr.exe
SIZE=arm-none-eabi-size.exe
DWF2XML=dwf2xml.exe

SCRIPT="C:\cygwin\home\Owner\projects\nuttx\nuttx\configs\stm3210e-eval\RIDE\nuttx.elf.ld"
LDFLAGS="-mcpu=cortex-m3 -mthumb  -u _start -Wl,-static -nostartfiles -Wl,--warn-once -nostdlib"

${LD} ${LDFLAGS} -Wl,-T -Xlinker "${SCRIPT}"
${OBJCOPY} nuttx.elf --target=ihex nuttx.hex
${REXRDR} nuttx.elf.sizetmp 0 ${SIZE} nuttx.elf
${DWF2XML} nuttx.dbi nuttx-globals.Standard.xml nuttx.Standard.xml ARM
