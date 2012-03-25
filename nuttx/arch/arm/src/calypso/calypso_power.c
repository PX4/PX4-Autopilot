#include <stdio.h>
#include <nuttx/spi.h>

int board_power_off(void)
{
	uint16_t tx;
	struct spi_dev_s *spi = up_spiinitialize(0);

	SPI_SETBITS(spi, 16);

	tx = (1 << 6) | (1 << 1);
	SPI_SNDBLOCK(spi, &tx, 1);

	tx = (1 << 6) | (30 << 1);
	SPI_SNDBLOCK(spi, &tx, 1);

	return 0;
}
