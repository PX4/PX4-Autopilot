/****************************************************************************
 * calypso_spi.c
 * SPI driver for TI Calypso
 *
 *   Copyright (C) 2011 Stefan Richter <ichgeh@l--putt.de>
 *
 * All rights reserved.
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

#include <nuttx/config.h>
#include <nuttx/spi.h>

#warning "MOST OF SPI API IS INCOMPLETE! (Wrapper around Osmocom driver)"
extern void spi_init(void);
extern int spi_xfer(uint8_t dev_idx, uint8_t bitlen, const void *dout, void *din);

#ifndef CONFIG_SPI_EXCHANGE
#error "Calypso HW only supports exchange. Enable CONFIG_SPI_EXCHANGE!"
#endif

struct calypso_spidev_s
{
	struct spi_dev_s spidev;	/* External driver interface */
	int		 nbits;		/* Number of transfered bits */

#ifndef CONFIG_SPI_OWNBUS
	sem_t		 exclsem;	/* Mutual exclusion of devices */
#endif
};

/* STUBS! */
#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif

static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
	bool selected)
{
}

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
	return frequency;
}

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
}

/* Osmocom wrapper */
static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
	((FAR struct calypso_spidev_s *)dev)->nbits = nbits;
}

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
	FAR void *rxbuffer, size_t nwords)
{
	FAR struct calypso_spidev_s *priv = (FAR struct calypso_spidev_s *)dev;
	size_t i;

	for(i=0; i<nwords; i++)
		spi_xfer(0, priv->nbits, txbuffer+i, rxbuffer+i);
}

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
	uint16_t buf = wd;
	spi_exchange(dev, &buf, &buf, 1);
	return buf;
}

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = 0,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = ,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
};

static struct calypso_spidev_s g_spidev =
{
	.spidev	= { &g_spiops },
	.nbits	= 0,
};

FAR struct spi_dev_s *up_spiinitialize(int port)
{
	switch(port) {
	case 0:		/* SPI master device */
		spi_init();
		return (FAR struct spi_dev_s *)&g_spidev;
	case 1:		/* uWire device */
		return NULL;
	default:
		return NULL;
	}
}
