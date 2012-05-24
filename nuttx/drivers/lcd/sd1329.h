/****************************************************************************
 * drivers/lcd/sd1329.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#ifndef __DRIVERS_LCD_SD1329_H
#define __DRIVERS_LCD_SD1329_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* SD1329 Commands **********************************************************/
/* Set column Address.
 *
 * This triple byte command specifies column start address and end address of
 * the display data RAM. This command also sets the column address pointer to
 * column start address. This pointer is used to define the current read/write
 * column address in graphic display data RAM. If horizontal address increment
 * mode is enabled by command 0xa0, after finishing read/write one column data,
 * it is incremented automatically to the next column address. Whenever the
 * column address pointer finishes accessing the end column address, it is
 * reset back to start column address and the row address is incremented to the
 * next row.
 *
 * Byte 1: 0x15
 * Byte 2: A[5:0]: Start Address, range: 0x00-0x3f
 * Byte 3: B[5:0]: End Address, range: 0x00-0x3f
 */

#define SSD1329_SET_COLADDR 0x15

/* Set Row Address.
 *
 * This triple byte command specifies row start address and end address of the
 * display data RAM. This command also sets the row address pointer to row
 * start address. This pointer is used to define the current read/write row
 * address in graphic display data RAM. If vertical address increment mode is
 * enabled by command 0xa0, after finishing read/write one row data, it is
 * incremented automatically to the next row address. Whenever the row address
 * pointer finishes accessing the end row address, it is reset back to start
 * row address.
 *
 * Byte 1: 0x75
 * Byte 2: A[6:0]: Start Address, range: 0x00-0x7f
 * Byte 3: B[6:0]: End Address, range: 0x00-0x7f
 */

#define SSD1329_SET_ROWADDR     0x75

/* Set Contract Current
 * 
 * This double byte command is to set Contrast Setting of the display. The
 * chip has 256 contrast steps from 0x00 to 0xff. The segment output current
 * increases linearly with the increase of contrast step.
 *
 * Byte 1: 0x81
 * Byte 2: A[7:0]: Contrast Value, range: 0-255
 */

#define SSD1329_SET_CONTRAST    0x81

/* Set Second Pre-Charge Speed
 *
 * This command is used to set the speed of second pre-charge in phase 3.
 * This speed can be doubled to achieve faster pre-charging through setting
 * 0x82 A[0].
 *
 * Byte 1: 0x82
 * Byte 2: A[7:1]: Second Pre-charge Speed
 *         A[0] = 1, Enable doubling the Second Pre-charge speed
 */

#define SSD1329_PRECHRG2_SPEED  0x82
#  define SSD1329_PRECHRG2_DBL  0x01

/* Set Master Icon Control
 *
 * This double command is used to set the ON / OFF conditions of internal
 * charge pump, icon circuits and overall icon status.
 *
 * Byte 1: 0x90
 * Byte 2: Icon control (OR of bits 0-1,4-5)
 */

#define SSD1329_ICON_CONTROL    0x90
#  define SSD1329_ICON_NORMAL   0x00 /* A[1:0]1=00: Icon RESET to normal display */
#  define SSD1329_ICON_ALLON    0x01 /* A[1:0]1=01: Icon All ON */
#  define SSD1329_ICON_ALLOFF   0x02 /* A[1:0]=10: Icon All OFF */
#  define SSD1329_ICON_DISABLE  0x00 /* A[4]=0: Disable Icon display */
#  define SSD1329_ICON_ENABLE   0x10 /* A[4]=1: Enable Icon display */
#  define SSD1329_VICON_DISABLE 0x00 /* A[5]=0: Disable VICON charge pump circuit */
#  define SSD1329_VICON_ENABLE  0x20 /* A[5]=1: Enable VICON charge pump circuit */

/* Set Icon Current Range
 *
 * This double byte command is used to set one fix current range for all icons
 * between the range of 0uA and 127.5uA. The uniformity improves as the icon
 * current range increases.
 *
 * Byte 1: 0x91
 * Byte 2: A[7:0]: Max icon current:
 *         00 = 0.0 uA
 *         01 = 0.5 uA
 *         ...
 *         ff = 127.5 uA
 */
 
#define SSD1329_ICON_CURRRNG  0x91
 
/* Set Individual Icon Current
 *
 * This multiple byte command is used to fine tune the current for each of the
 * 64 icons. Command 0x92 followed by 64 single byte data. These 64 byte data
 * have to be entered in order to make this command function. Below is the
 * formula for calculating the icon current.
 *
 * Icon Current = Single byte value / 127 x Maximum icon current set with command 0x91
 *
 * Byte 1: 0x92
 * Byte 2-65: An[6:0]: icon current for ICSn, range: 0x00-0x7f
 *            Icon Current of ICSn = An[6:0]/127) x max icon current
 */

#define SSD1329_ICON_CURRENT  0x92

/* Set Individual Icon ON / OFF Register
 *
 * This double byte command is used to select one of the 64 icons and choose the
 * ON, OFF or blinking condition of the selected icon.
 *
 * Byte 1: 0x93
 * Byte 2: A[5:0]: Select one of the 64 icons from ICS0 ~ ICS63
 *         A[7:6]: OFF/ON/BLINK
 */

#define SSD1329_ICON_SELECT   0x93
#  define SSD1329_ICON_OFF    0x00
#  define SSD1329_ICON_ON     0x40
#  define SSD1329_ICON_BLINK  0xc0

/* Set Icon ON / OFF Registers
 *
 * This double byte command is used to set the ON / OFF status of all 64 icons.
 *
 * Byte 1: 0x94
 * Byte 2: A[7:6]: OFF/ON/BLINK (Same as 0x93)
 */

#define SSD1329_ICON_ALL 0x94

/* Set Icon Blinking Cycle
 *
 * This double byte command is used to set icon oscillator frequency and
 * blinking cycle selected with above command 0x93.
 *
 * Byte 1: 0x95
 * Byte 2:
 *   - A[2:0]:Icon Blinking cycle
 *   - A[5:4]:Icon oscillation frequency
 */

#define SSD1329_ICON_BLINKING       0x95
#  define SSD1329_ICON_BLINK_0p25S  0x00 /* 0.25 sec */
#  define SSD1329_ICON_BLINK_0p50S  0x01 /* 0.50 sec */
#  define SSD1329_ICON_BLINK_0p75S  0x02 /* 0.75 sec */
#  define SSD1329_ICON_BLINK_0p100S 0x03 /* 1.00 sec */
#  define SSD1329_ICON_BLINK_0p125S 0x04 /* 1.25 sec */
#  define SSD1329_ICON_BLINK_0p150S 0x05 /* 1.50 sec */
#  define SSD1329_ICON_BLINK_0p175S 0x06 /* 1.75 sec */
#  define SSD1329_ICON_BLINK_0p200S 0x07 /* 2.00 sec */
#  define SSD1329_ICON_BLINK_61KHZ  0x00 /* 61 KHz */
#  define SSD1329_ICON_BLINK_64KHZ  0x10 /* 64 KHz */
#  define SSD1329_ICON_BLINK_68KHZ  0x20 /* 68 KHz */
#  define SSD1329_ICON_BLINK_73KHZ  0x30 /* 73 KHz */

/* Set Icon Duty
 *
 * This double byte command is used to set the icon frame frequency and icon AC
 * drive duty ratio.
 *
 * Byte 1: 0x96
 * Byte 2:
 *   - A[2:0]: AC Drive
 *   - A[7:4]: con frame frequency
 */

#define SSD1329_ICON_ACDRIVE      0x96
#  define SSD1329_ICON_DUTY_DC    0x00
#  define SSD1329_ICON_DUTY_63_64 0x01
#  define SSD1329_ICON_DUTY_62_64 0x02
#  define SSD1329_ICON_DUTY_61_64 0x03
#  define SSD1329_ICON_DUTY_60_64 0x04
#  define SSD1329_ICON_DUTY_59_64 0x05
#  define SSD1329_ICON_DUTY_58_64 0x06
#  define SSD1329_ICON_DUTY_57_64 0x07

/* Set Re-map
 *
 * This double command has multiple configurations and each bit setting is
 * described as follows:
 *
 * Column Address Remapping (A[0])
 *   This bit is made for increase the flexibility layout of segment signals in
 *   OLED module with segment arranged from left to right (when A[0] is set to 0)
 *   or from right to left (when A[0] is set to 1).
 *
 * Nibble Remapping (A[1])
 *   When A[1] is set to 1, the two nibbles of the data bus for RAM access are
 *   re-mapped, such that (D7, D6, D5, D4, D3, D2, D1, D0) acts like (D3, D2, D1,
 *   D0, D7, D6, D5, D4) If this feature works together with Column Address
 *   Re-map, it would produce an effect of flipping the outputs from SEG0-127 to
 *   SEG127-SEG0.
 *
 * Address increment mode (A[2])
 *   When A[2] is set to 0, the driver is set as horizontal address incremen
 *   mode. After the display RAM is read/written, the column address pointer is
 *   increased automatically by 1. If the column address pointer reaches column
 *   end address, the column address pointer is reset to column start address and
 *   row address pointer is increased by 1.
 *
 *   When A[2] is set to 1, the driver is set to vertical address increment mode.
 *   After the display RAM is read/written, the row address pointer is increased
 *   automatically by 1. If the row address pointer reaches the row end address,
 *   the row address pointer is reset to row start address and column address
 *   pointer is increased by 1.
 *
 * COM Remapping (A[4])
 *   This bit defines the scanning direction of the common for flexible layout
 *   of common signals in OLED module either from up to down (when A[4] is set to
 *   0) or from bottom to up (when A[4] is set to 1).
 *
 * Splitting of Odd / Even COM Signals (A[6])
 *   This bit is made to match the COM layout connection on the panel. When A[6]
 *   is set to 0, no splitting odd / even of the COM signal is performed. When
 *   A[6] is set to 1, splitting odd / even of the COM signal is performed,
 *   output pin assignment sequence is shown as below (for 128MUX ratio):
 *
 * Byte 1: 0xa0
 * Byte 2: A[7:0]
 */

#define SSD1329_GDDRAM_REMAP    0xa0
#  define SSD1329_COLADDR_REMAP 0x01 /* A[0]: Enable column re-map */
#  define SSD1329_NIBBLE_REMAP  0x02 /* A[1]: Enable nibble re-map */
#  define SSD1329_VADDR_INCR    0x04 /* A[1]: Enable vertical address increment */
#  define SSD1329_COM_REMAP     0x10 /* A[4]: Enable COM re-map */
#  define SSD1329_COM_SPLIT     0x40 /* A[6]: Enable COM slip even/odd */

/* Set Display Start Line
 *
 * This double byte command is to set Display Start Line register for
 * determining the starting address of display RAM to be displayed by selecting
 * a value from 0 to 127.
 *
 * Byte 1: 0xa1
 * Byte 2: A[6:0]: Vertical scroll by setting the starting address of
 *         display RAM from 0-127
 */

#define SSD1329_VERT_START 0xa1

/* Set Display Offset
 *
 * This double byte command specifies the mapping of display start line (it is
 * assumed that COM0 is the display start line, display start line register
 * equals to 0) to one of COM0-COM127.
 *
 * Byte 1: 0xa2
 * Byte 2: A[6:0]: Set vertical offset by COM from 0-127
 */

#define SSD1329_VERT_OFFSET 0xa2

/* Set Display Mode - Normal, all on, all off, inverse
 *
 * These are single byte commands and are used to set display status to Normal
 * Display, Entire Display ON, Entire Display OFF or Inverse Display.
 *
 * Normal Display (0xa4)
 *   Reset the “Entire Display ON, Entire Display OFF or Inverse Display” effects
 *   and turn the data to ON at the corresponding gray level.
 *
 * Set Entire Display ON (0xa5)
 *   Force the entire display to be at gray scale level GS15, regardless of the
 *   contents of the display data RAM.
 *
 * Set Entire Display OFF (0xa6)
 *   Force the entire display to be at gray scale level GS0, regardless of the
 *   contents of the display data RAM.
 *
 * Inverse Display (0xa7)
 *   The gray scale level of display data are swapped such that “GS0” <-> “GS15”,
 *   “GS1” <-> “GS14”, etc.
 *
 * Byte 1: Display mode command
 */

#define SSD1329_DISP_NORMAL 0xa4
#define SSD1329_DISP_OFF    0xa5
#define SSD1329_DISP_ON     0xa6
#define SSD1329_DISP_INVERT 0xa7

/* Set MUX Ratio
 *
 * This double byte command sets multiplex ratio (MUX ratio) from 16MUX to
 * 128MUX. In POR, multiplex ratio is 128MUX.
 *
 * Byte 1: 0xa8
 * Byte 2: A[6:0] 15-127 representing 16-128 MUX
 */

#define SSD1329_MUX_RATIO 0xa8

/* Set Sleep mode ON / OFF
 *
 * These single byte commands are used to turn the matrix display on the OLED
 * panel display either ON or OFF. When the sleep mode is set to ON (0xae), the
 * display is OFF, the segment and common output are in high impedance state
 * and circuits will be turned OFF. When the sleep mode is set to OFF (0xaf),
 * the display is ON.
 *
 * Byte 1: sleep mode command
 */

#define SSD1329_SLEEP_ON  0xae
#define SSD1329_SLEEP_OFF 0xaf

/* Set Phase Length
 *
 * In the second byte of this double command, lower nibble and higher nibble is
 * defined separately. The lower nibble adjusts the phase length of Reset (phase
 * 1). The higher nibble is used to select the phase length of first pre-charge
 * phase (phase 2). The phase length is ranged from 1 to 16 DCLK's. RESET for
 * A[3:0] is set to 3 which means 4 DCLK’s selected for Reset phase. POR for
 * A[7:4] is set to 5 which means 6 DCLK’s is selected for first pre-charge
 * phase. Please refer to Table 9-1 for detail breakdown levels of each step.
 *
 * Byte 1: 0xb1
 * Byte 2: A[3:0]: Phase 1 period of 1~16 DCLK’s
 *         A[7:4]: Phase 2 period of 1~16 DCLK’s
 */

#define SSD1329_PHASE_LENGTH 0xb1

/* Set Frame Frequency
 *
 * This double byte command is used to set the number of DCLK’s per row between
 * the range of 0x14 and 0x7f.  Then the Frame frequency of the matrix display
 * is equal to DCLK frequency / A[6:0].
 *
 * Byte 1: 0xb2
 * Byte 2: A[6:0]:Total number of DCLK’s per row. Ranging from
 *         0x14 to 0x4e DCLK’s. frame Frequency = DCLK freq /A[6:0].
 */

#define SSD1329_FRAME_FREQ 0xb2

/* Set Front Clock Divider / Oscillator Frequency
 *
 * This double command is used to set the frequency of the internal display
 * clocks, DCLK's. It is defined by dividing the oscillator frequency by the
 * divide ratio (Value from 1 to 16). Frame frequency is determined by divide
 * ratio, number of display clocks per row, MUX ratio and oscillator frequency.
 * The lower nibble of the second byte is used to select the oscillator
 * frequency. Please refer to Table 9-1 for detail breakdown levels of each
 * step.
 *
 * Byte 1: 0xb3
 * Byte 2: A[3:0]: Define divide ratio (D) of display clock (DCLK)
 *                 Divide ratio=A[3:0]+1
 *         A[7:4] : Set the Oscillator Frequency, FOSC. Range:0-15
 */

#define SSD1329_DCLK_DIV 0xb3

/* Set Default Gray Scale Table
 *
 * This single byte command is used to set the gray scale table to initial
 * default setting.
 *
 * Byte 1: 0xb7
 */

#define SSD1329_GSCALE_TABLE 0xb7

/* Look Up Table for Gray Scale Pulse width
 *
 * This command is used to set each individual gray scale level for the display.
 * Except gray scale level GS0 that has no pre-charge and current drive, each
 * gray scale level is programmed in the length of current drive stage pulse
 * width with unit of DCLK. The longer the length of the pulse width, the
 * brighter the OLED pixel when it’s turned ON.
 *
 * The setting of gray scale table entry can perform gamma correction on OLED
 * panel display. Normally, it is desired that the brightness response of the
 * panel is linearly proportional to the image data value in display data RAM.
 * However, the OLED panel is somehow responded in non-linear way. Appropriate
 * gray scale table setting like example below can compensate this effect.
 *
 * Byte 1: 0xb8
 * Bytes 2-16: An[5:0], value for GSn level Pulse width
 */

#define SSD1329_GSCALE_LOOKUP 0xb8

/* Set Second Pre-charge Period
 *
 * This double byte command is used to set the phase 3 second pre-charge period.
 * The period of phase 3 can be programmed by command 0xbb and it is ranged from
 * 0 to 15 DCLK's.
 *
 * Byte 1: 0xbb
 * Byte 2: 0-15 DCLKs
 */

#define SSD1329_PRECHRG2_PERIOD 0xbb

/* Set First Precharge voltage, VP
 *
 * This double byte command is used to set phase 2 first pre-charge voltage
 * level. It can be programmed to set the first pre-charge voltage reference to
 * VCC or VCOMH.
 *
 * Byte 1: 0xbc
 * Byte 2: A[5] == 0, Pre-charge voltage is (0.30 + A[4:0]) * Vcc
 *         A{5] == 1, 1.00 x VCC or connect to VCOMH if VCC > VCOMH
 */

#define SSD1329_PRECHRG1_VOLT 0xbc

/* Set VCOMH
 *
 * This double byte command sets the high voltage level of common pins, VCOMH.
 * The level of VCOMH is programmed with reference to VCC.
 *
 * Byte 1: 0xbe
 * Byte 2: (0.51 + A[5:0]) * Vcc
 */

#define SSD1329_COM_HIGH 0xbe

/* NOOP
 *
 * This is a no operation command.
 *
 * Byte 1: 0xe3
 */

#define SSD1329_NOOP 0xe3

/* Set Command Lock
 *
 * This command is used to lock the MCU from accepting any command.
 *
 * Byte 1: 0xfd
 * Byte 2: 0x12 | A[2]
 *         A[2] == 1, Enable locking the MCU from entering command
 */

#define SSD1329_CMD_LOCK   0xfd
#  define SSD1329_LOCK_ON  0x13
#  define SSD1329_LOCK_OFF 0x12

/* SD1329 Status ************************************************************/

#define SDD1329_STATUS_ON  0x00 /* D[6]=0: indicates the display is ON */
#define SDD1329_STATUS_OFF 0x40 /* D[6]=1: indicates the display is OFF */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_LCD_SD1329_H */
