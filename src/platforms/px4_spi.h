#pragma once

#ifdef __PX4_NUTTX
#include <nuttx/spi.h>
#elif defined(__PX4_POSIX)
enum spi_dev_e {
	SPIDEV_NONE = 0,    /* Not a valid value */
	SPIDEV_MMCSD,       /* Select SPI MMC/SD device */
	SPIDEV_FLASH,       /* Select SPI FLASH device */
	SPIDEV_ETHERNET,    /* Select SPI ethernet device */
	SPIDEV_DISPLAY,     /* Select SPI LCD/OLED display device */
	SPIDEV_WIRELESS,    /* Select SPI Wireless device */
	SPIDEV_TOUCHSCREEN, /* Select SPI touchscreen device */
	SPIDEV_EXPANDER,    /* Select SPI I/O expander device */
	SPIDEV_MUX,         /* Select SPI multiplexer device */
	SPIDEV_AUDIO_DATA,  /* Select SPI audio codec device data port */
	SPIDEV_AUDIO_CTRL,  /* Select SPI audio codec device control port */
};

/* Certain SPI devices may required different clocking modes */

enum spi_mode_e {
	SPIDEV_MODE0 = 0,   /* CPOL=0 CHPHA=0 */
	SPIDEV_MODE1,       /* CPOL=0 CHPHA=1 */
	SPIDEV_MODE2,       /* CPOL=1 CHPHA=0 */
	SPIDEV_MODE3        /* CPOL=1 CHPHA=1 */
};
// CPOL 时钟极性 
// CPOL=0时， SCK在空闲状态时为低电平，CPOL=1时，则相反。
// CPHA 时钟相位
// 时钟相位CPHA是指数据的采样的时刻，
// 当CPHA=0时，MOSI或MISO数据线上的信号将会在SCK时钟线的奇数跳变沿被采样

struct spi_dev_s {
	int unused;
};
#else
#endif
