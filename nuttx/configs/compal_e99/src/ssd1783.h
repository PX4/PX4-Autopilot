#ifndef SSD1783_H_
#define SSD1783_H_

#include <nuttx/lcd/lcd.h>

#define FB_COLOR_TO_R(v)                (((v)>>16) & 0xff)
#define FB_COLOR_TO_G(v)                (((v)>> 8) & 0xff)
#define FB_COLOR_TO_B(v)                ( (v)      & 0xff)

#define SSD1783_UWIRE_BITLEN    9
#define SSD1783_DEV_ID          0

#define ARMIO_LATCH_OUT 0xfffe4802
#define IO_CNTL_REG    0xfffe4804
#define ASIC_CONF_REG  0xfffef008

#define ASCONF_PWL_ENA  (1 << 4)

/* begin backlight.c */
#define BASE_ADDR_PWL   0xfffe8000
#define PWL_REG(m)      (BASE_ADDR_PWL + (m))

enum pwl_reg {
        PWL_LEVEL       = 0,
        PWL_CTRL        = 1,
};

enum ssd1783_cmdflag { CMD, DATA, END };

struct ssd1783_cmdlist {
  enum ssd1783_cmdflag is_cmd:8;	/* 1: is a command, 0: is data, 2: end marker! */
  uint8_t data;  			/* 8 bit to send to LC display */
} __attribute__((packed));

static const struct ssd1783_cmdlist nop[] = {
  { CMD, 0x25 }, // NOP command
  { END, 0x00 }
};

static const struct ssd1783_cmdlist
ssd1783_initdata[] = {
  { CMD,  0xD1 }, /* CMD   set internal oscillator on */
  { CMD,  0x94 }, /* CMD   leave sleep mode */
  { CMD,  0xbb }, /* CMD   Set COM Output Scan Direction: */
  { DATA, 0x01 }, /* DATA: 01: COM0-79, then COM159-80 */
/* -------- DIFFERENT FROM ORIGINAL CODE: -------------- */
/* we use 8bit per pixel packed RGB 332 */
  { CMD,  0xbc }, /* CMD   Set Data Output Scan Direction */
  { DATA, 0x00 }, /* DATA: column scan, normal rotation, normal display */
  { DATA, 0x00 }, /* DATA: RGB color arrangement R G B R G B ... */
/*-->*/ { DATA, 0x01 }, /* DATA: 8 bit per pixel mode MSB <RRRGGGBB> LSB */
/* --------- /DIFFERENT ---------- */
  { CMD,  0xce }, /* CMD   Set 256 Color Look Up Table LUT */
  { DATA, 0x00 },	/* DATA red 000 */
  { DATA, 0x03 },	/* DATA red 001 */
  { DATA, 0x05 },	/* DATA red 010 */
  { DATA, 0x07 },	/* DATA red 011 */
  { DATA, 0x09 },	/* DATA red 100 */
  { DATA, 0x0b },	/* DATA red 101 */
  { DATA, 0x0d },	/* DATA red 110 */
  { DATA, 0x0f },	/* DATA red 111 */
  { DATA, 0x00 },	/* DATA green 000 */
  { DATA, 0x03 },	/* DATA green 001 */
  { DATA, 0x05 },	/* DATA green 010 */
  { DATA, 0x07 },	/* DATA green 011 */
  { DATA, 0x09 },	/* DATA green 100 */
  { DATA, 0x0b },	/* DATA green 101 */
  { DATA, 0x0d },	/* DATA green 110 */
  { DATA, 0x0f },	/* DATA green 111 */
  { DATA, 0x00 },	/* DATA blue 00 */
  { DATA, 0x05 },	/* DATA blue 01 */
  { DATA, 0x0a },	/* DATA blue 10 */
  { DATA, 0x0f },	/* DATA blue 11 */
  { CMD,  0xca }, /* CMD   Set Display Control - Driver Duty Selection */
  { DATA, 0xff }, // can't find description of the values in the original
  { DATA, 0x10 }, // display/ssd1783.c in my datasheet :-(
  { DATA, 0x01 }, //
  { CMD,  0xab }, /* CMD   Set Scroll Start */
  { DATA, 0x00 }, /* DATA: Starting address at block 0 */
  { CMD,  0x20 }, /* CMD   Set power control register */
  { DATA, 0x0b }, /* DATA: booster 6x, reference gen. & int regulator */
  { CMD,  0x81 }, /* CMD   Contrast Lvl & Int. Regul. Resistor Ratio */
  { DATA, 0x29 }, /* DATA: contrast = 0x29 */
  { DATA, 0x05 }, /* DATA: 0x05 = 0b101 -> 1+R2/R1 = 11.37 */
  { CMD,  0xa7 }, /* CMD   Invert Display */
  { CMD,  0x82 }, /* CMD   Set Temperature Compensation Coefficient */
  { DATA, 0x00 }, /* DATA: Gradient is -0.10 % / degC */
  { CMD,  0xfb }, /* CMD   Set Biasing Ratio */
  { DATA, 0x03 }, /* DATA: 1/10 bias */
  { CMD,  0xf2 }, /* CMD   Set Frame Frequency and N-line inversion */
  { DATA, 0x08 }, /* DATA: 75 Hz (POR) */
  { DATA, 0x06 }, /* DATA: n-line inversion: 6 lines */
  { CMD,  0xf7 }, /* CMD   Select PWM/FRC Select Full Col./8col mode */
  { DATA, 0x28 }, /* DATA: always 0x28 */
  { DATA, 0x8c }, /* DATA: 4bit PWM + 2 bit FRC */
  { DATA, 0x05 }, /* DATA: full color mode */
  { CMD,  0xaf }, /* CMD   Display On */
  { END,  0x00 }, /* MARKER: end of list */
};

struct ssd1783_dev_s
{
  /* Publicly visible device structure */
  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */
  uint8_t power; /* Current power setting */
};

#endif /* SSD1783_H_ */
