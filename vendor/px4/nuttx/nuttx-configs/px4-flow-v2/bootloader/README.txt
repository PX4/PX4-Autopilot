/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

This file currently list the SoC that are in uses and the features from the Mfg description.
 It includes what is current upported in Nuttx and nearest Soc to help with porting

Pixhawk ESC- v1.2 -     STM32F105RCT6
Pixhawk ESC- v1.3 -     STM32F105RCT6
Pixhawk ESC- v1.4 -     STM32F105RCT6
Pixhawk ESC- v1.5 -     STM32F205RCT6
Pixhawk ESC- v1.5-prto -STM32F205RET6
Pixhawk ESC- v1.6-prto -STM32F446RET6
Px4Flow                 STM32F407VGT6
Px4FlowFuture           STM32F427VIT7 Rev_C silicon (no PB12 silicon bug, full 2M Flash available)
Olimexino-STM32         STM32F103RBT6
Olimex LPC-P11C24       LPC11C24FBD48
freedom-kl25z           MKL25Z128VLK4 MCU – 48 MHz, 128 KB flash, 16 KB SRAM, USB OTG (FS), 80LQFP
freedom-kl26z           MKL26Z128VLH4 MCU – 48 MHz, 128 KB flash, 16 KB SRAM, USB OTG (FS), 64 LQFP

Using:
Part Number Package Marketing Status  Core  Operating Frequency(F) (Processor speed)  FLASH Size (Prog) Data E2PROM nom (B) Internal RAM Size 16-bit timers (IC/OC/PWM) 32-bit timer (IC/OC/PWM)  Other timer functions A/D Converter I/Os (High Current) Display controller  D/A Converter Integrated op-amps    Serial Interface                        Supply Current(Icc) (Lowest power mode) typ (µA)  Supply Current(Icc) (Run mode (per Mhz)) typ (µA) Operating Temperature min (°C)  Operating Temperature max (°C)
STM32F103RB LQFP 64 10x10x1.4 Active  ARM Cortex-M3 72  128 - 20  4x16-bit  - 2 x WDG, RTC, 24-bit down counter 16x12-bit 51  - - -   2xSPI 2xI2C 3xUSART(IrDA, ISO 7816) USB CAN               1.7 373 -40 85
STM32F105RC LQFP 64 10x10x1.4 Active  ARM Cortex-M3 72  256 - 64  7x16-bit  - 2 x WDG, RTC, 24-bit down counter 16x12-bit 51  - 2x12-bit  -   3xSPI 2xI2S 2xI2C 3xUSART 2xUART  USB OTG FS  2xCAN           1.9 393 -40 105
STM32F205RC LQFP 64 10x10x1.4 Active  ARM Cortex-M3 120 256 - 96  12x16-bit 2x32-bit  2 x WDG, RTC, 24-bit down counter 16x12-bit 51  - 2x12-bit  -   3xSPI 2xI2S 3xI2C 4xUSART(IrDA, ISO 7816) 2xUART  2xUSB OTG (FS+FS/HS)  2xCAN SDIO          2.5 188 -40 105
STM32F205RE LQFP 64 10x10x1.4 Active  ARM Cortex-M3 120 512 - 128 12x16-bit 2x32-bit  2 x WDG, RTC, 24-bit down counter 16x12-bit 51  - 2x12-bit  -   3xSPI 2xI2S 3xI2C 4xUSART(IrDA, ISO 7816) 2xUART  2xUSB OTG (FS+FS/HS)  2xCAN SDIO          2.5 188 -40 105

Nuttx:
STM32F103RB LQFP 64 10x10x1.4 Active  ARM Cortex-M3 72  128 - 20  4x16-bit  - 2 x WDG, RTC, 24-bit down counter 16x12-bit 51  - - -   2xSPI 2xI2C 3xUSART(IrDA, ISO 7816) USB CAN               1.7 373 -40 85
STM32F207IG BGA 176; LQFP     Active  ARM Cortex-M3 120 1024  - 128 12x16-bit 2x32-bit  2 x WDG, RTC, 24-bit down counter 24x12-bit 140 - 2x12-bit  -   3xSPI 2xI2S 2xI2C 4xUSART(IrDA, ISO 7816) 2xUART  2xUSB OTG (FS+FS/HS)  2xCAN Ethernet MAC10/100  SDIO        2.5 188 -40 105
STM32F207ZE LQFP 144          Active  ARM Cortex-M3 120 512 - 128 12x16-bit 2x32-bit  2 x WDG, RTC, 24-bit down counter 24x12-bit 140 - 2x12-bit  -   3xSPI 2xI2S 2xI2C 4xUSART(IrDA, ISO 7816) 2xUART  2xUSB OTG (FS+FS/HS)  2xCAN Ethernet MAC10/100  SDIO        2.5 188 -40 85

Type number Core  Clock speed [max] (MHz) Flash (kB)  RAM (kB)  GPIO  CAN UART  I²C SPI ADC channels  ADC (bits)  Timers  Timer (bits)  PWM Package name  Temperature range (°C)  Demoboard
LPC11C24FBD48 Cortex-M0 50  32  8 36  1 1 1 2 8 10  4 16; 32  11  LQFP48  -40 °C to +85 °C  OM13012

Products  Parts Order Status  Budgetary Price excluding tax(US$)  HW dev. tools Core: Type  Operating Frequency (Max) (MHz)   Cache (KB)  Total Flash (KB)  Internal RAM (KB)   Boot ROM (KB)   UART  SPI I2C I2S USB OTG USB features  GPIOs ADC DAC 16-bit PWM (ch) 16-bit Timer (ch) Other Analog blocks Human Machine Interface VDD (min) (V) VDD (max) (V)   Package type  Pin count Package dimension (WxLxH) (mm3) Pin pitch (mm)  Ambient Operating Temperature (Min-Max) (°C)
KL2x  MKL25Z128VLK4 N Active  10000 @ US$2.18 each  FRDM-KL25Z, TWR-KL25Z48M  ARM Cortex-M0+  48  0.064 128 - - 3 2 2 - 1 (FS)  USB OTG LS/FS 66  16-bit (SAR) x 1  12-bit x 1, 6-bit x 1 10  11  Comparator  Touch System Interface  1.71  3.6 LQFP  80  12x12x1.6 0.5 -40 to 105
KL2x  MKL26Z128VLH4 Y Active  10000 @ US$1.90 each  FRDM-KL26Z                ARM Cortex-M0+  48  0.064 128 - - 3 2 2 1 1 (FS)  USB 2.0 FS Certified  50  16-bit (SAR) x 1  12-bit x 1, 6-bit x 1 10  11  Comparator  Touch System Interface  1.71  3.6 LQFP  64  10x10x1.6 0.5 -40 to 105

Nuttx:
STM32F103RB
STM32F105VB
STM32F107VC
STM32F207IG BGA 176; LQFP 176 24x24x1.4 Active  ARM Cortex-M3 120 1024  - 128 12x16-bit 2x32-bit  2 x WDG, RTC, 24-bit down counter 24x12-bit 140 - 2x12-bit  -   3xSPI 2xI2S 2xI2C 4xUSART(IrDA, ISO 7816) 2xUART  2xUSB OTG (FS+FS/HS)  2xCAN Ethernet MAC10/100  SDIO        2.5 188 -40 105
STM32F207ZE LQFP 144 20x20x1.4          Active  ARM Cortex-M3 120 512 - 128 12x16-bit 2x32-bit  2 x WDG, RTC, 24-bit down counter 24x12-bit 140 - 2x12-bit  -   3xSPI 2xI2S 2xI2C 4xUSART(IrDA, ISO 7816) 2xUART  2xUSB OTG (FS+FS/HS)  2xCAN Ethernet MAC10/100  SDIO        2.5 188 -40 85


Nuttx HAS:
STM32F100C8
STM32F100CB
STM32F100R8
STM32F100RB
STM32F100RC
STM32F100RD
STM32F100RE
STM32F100V8
STM32F100VB
STM32F100VC
STM32F100VD
STM32F100VE
STM32F102CB
STM32F103C4
STM32F103C8
STM32F103CB
STM32F103R8
STM32F103RB
STM32F103RC
STM32F103RD
STM32F103RE
STM32F103RG
STM32F103T8
STM32F103TB
STM32F103V8
STM32F103VB
STM32F103VC
STM32F103VE
STM32F103ZE
STM32F105VB
STM32F107VC
STM32F207IG
STM32F207ZE
STM32F302CB
STM32F302CC
STM32F302RB
STM32F302RC
STM32F302VB
STM32F302VC
STM32F303CB
STM32F303CC
STM32F303RB
STM32F303RC
STM32F303VB
STM32F303VC
STM32F372C8
STM32F372CB
STM32F372CC
STM32F372R8
STM32F372RB
STM32F372RC
STM32F372V8
STM32F372VB
STM32F372VC
STM32F373C8
STM32F373CB
STM32F373CC
STM32F373R8
STM32F373RB
STM32F373RC
STM32F373V8
STM32F373VB
STM32F373VC
STM32F401RE
STM32F405RG
STM32F405VG
STM32F405ZG
STM32F407IE
STM32F407IG
STM32F407VE
STM32F407VG
STM32F407ZE
STM32F407ZG
STM32F411RE
STM32F427I
STM32F427V
STM32F427Z
STM32F429B
STM32F429I
STM32F429N
STM32F429V
STM32F429Z
STM32L151C6
STM32L151C8
STM32L151CB
STM32L151R6
STM32L151R8
STM32L151RB
STM32L151V6
STM32L151V8
STM32L151VB
STM32L152C6
STM32L152C8
STM32L152CB
STM32L152R6
STM32L152R8
STM32L152RB
STM32L152V6
STM32L152V8
STM32L152VB
STM32L162VE
STM32L162ZD
