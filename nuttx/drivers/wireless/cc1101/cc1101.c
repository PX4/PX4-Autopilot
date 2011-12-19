/****************************************************************************
 * drivers/wireless/cc1101/cc1101.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
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

/** \file
 *  \author Uros Platise
 *  \brief Chipcon CC1101 Device Driver
 * 
 * Features:
 *   - Maximum data length: 61 bytes CC1101_PACKET_MAXDATALEN
 *   - Packet length includes two additional bytes: CC1101_PACKET_MAXTOTALLEN
 *   - Requires one GDO to trigger end-of-packets in RX and TX modes.
 *   - Variable packet length with data payload between 1..61 bytes
 *     (three bytes are reserved for packet length, and RSSI and LQI
 *      appended at the end of RXFIFO after each reception)
 *   - Support for General Digital Outputs with overload protection 
 *     (single XOSC pin is allowed, otherwise error is returned)
 *   - Loadable RF settings, one for ISM Region 1 (Europe) and one for
 *     ISM Region 2 (Complete America)
 * 
 * Todo:
 *   - Extend max packet length up to 255 bytes or rather infinite < 4096 bytes
 *   - Power up/down modes
 *   - Sequencing between states or add protection for correct termination of
 *     various different state (so that CC1101 does not block in case of improper use)
 * 
 * \par RSSI and LQI value interpretation
 * 
 * The LQI can be read from the LQI status register or it can be appended 
 * to the received packet in the RX FIFO. LQI is a metric of the current 
 * quality of the received signal. The LQI gives an estimate of how easily 
 * a received signal can be demodulated by accumulating the magnitude of 
 * the error between ideal constellations and the received signal over 
 * the 64 symbols immediately following the sync word. LQI is best used 
 * as a relative measurement of the link quality (a high value indicates 
 * a better link than what a low value does), since the value is dependent 
 * on the modulation format.
 * 
 * To simplify: If the received modulation is FSK or GFSK, the receiver 
 * will measure the frequency of each "bit" and compare it with the 
 * expected frequency based on the channel frequency and the deviation 
 * and the measured frequency offset. If other modulations are used, the 
 * error of the modulated parameter (frequency for FSK/GFSK, phase for 
 * MSK, amplitude for ASK etc) will be measured against the expected 
 * ideal value
 * 
 * RSSI (Received Signal Strength Indicator) is a signal strength 
 * indication. It does not care about the "quality" or "correctness" of 
 * the signal. LQI does not care about the actual signal strength, but 
 * the signal quality often is linked to signal strength. This is because 
 * a strong signal is likely to be less affected by noise and thus will 
 * be seen as "cleaner" or more "correct" by the receiver.
 * 
 * There are four to five "extreme cases" that can be used to illustrate 
 * how RSSI and LQI work:
 *  1. A weak signal in the presence of noise may give low RSSI and low LQI.
 *  2. A weak signal in "total" absence of noise may give low RSSI and high LQI.
 *  3. Strong noise (usually coming from an interferer) may give high RSSI and low LQI.
 *  4. A strong signal without much noise may give high RSSI and high LQI.
 *  5. A very strong signal that causes the receiver to saturate may give 
 *     high RSSI and low LQI.
 * 
 * Note that both RSSI and LQI are best used as relative measurements since 
 * the values are dependent on the modulation format.
 **/ 
 
#include <nuttx/config.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/cc1101.h>


/****************************************************************************
 * Declarations
 ****************************************************************************/

#define CC1101_SPIFREQ_BURST    6500000 /* Hz, no delay */
#define CC1101_SPIFREQ_SINGLE   9000000 /* Hz, single access only - no delay */

#define CC1101_MCSM0_VALUE      0x1C

/****************************************************************************
 * Chipcon CC1101 Internal Registers
 ****************************************************************************/

/* Configuration Registers */

#define CC1101_IOCFG2           0x00        /* GDO2 output pin configuration */
#define CC1101_IOCFG1           0x01        /* GDO1 output pin configuration */
#define CC1101_IOCFG0           0x02        /* GDO0 output pin configuration */
#define CC1101_FIFOTHR          0x03        /* RX FIFO and TX FIFO thresholds */
#define CC1101_SYNC1            0x04        /* Sync word, high byte */
#define CC1101_SYNC0            0x05        /* Sync word, low byte */
#define CC1101_PKTLEN           0x06        /* Packet length */
#define CC1101_PKTCTRL1         0x07        /* Packet automation control */
#define CC1101_PKTCTRL0         0x08        /* Packet automation control */
#define CC1101_ADDR             0x09        /* Device address */
#define CC1101_CHANNR           0x0A        /* Channel number */
#define CC1101_FSCTRL1          0x0B        /* Frequency synthesizer control */
#define CC1101_FSCTRL0          0x0C        /* Frequency synthesizer control */
#define CC1101_FREQ2            0x0D        /* Frequency control word, high byte */
#define CC1101_FREQ1            0x0E        /* Frequency control word, middle byte */
#define CC1101_FREQ0            0x0F        /* Frequency control word, low byte */
#define CC1101_MDMCFG4          0x10        /* Modem configuration */
#define CC1101_MDMCFG3          0x11        /* Modem configuration */
#define CC1101_MDMCFG2          0x12        /* Modem configuration */
#define CC1101_MDMCFG1          0x13        /* Modem configuration */
#define CC1101_MDMCFG0          0x14        /* Modem configuration */
#define CC1101_DEVIATN          0x15        /* Modem deviation setting */
#define CC1101_MCSM2            0x16        /* Main Radio Cntrl State Machine config */
#define CC1101_MCSM1            0x17        /* Main Radio Cntrl State Machine config */
#define CC1101_MCSM0            0x18        /* Main Radio Cntrl State Machine config */
#define CC1101_FOCCFG           0x19        /* Frequency Offset Compensation config */
#define CC1101_BSCFG            0x1A        /* Bit Synchronization configuration */
#define CC1101_AGCCTRL2         0x1B        /* AGC control */
#define CC1101_AGCCTRL1         0x1C        /* AGC control */
#define CC1101_AGCCTRL0         0x1D        /* AGC control */
#define CC1101_WOREVT1          0x1E        /* High byte Event 0 timeout */
#define CC1101_WOREVT0          0x1F        /* Low byte Event 0 timeout */
#define CC1101_WORCTRL          0x20        /* Wake On Radio control */
#define CC1101_FREND1           0x21        /* Front end RX configuration */
#define CC1101_FREND0           0x22        /* Front end TX configuration */
#define CC1101_FSCAL3           0x23        /* Frequency synthesizer calibration */
#define CC1101_FSCAL2           0x24        /* Frequency synthesizer calibration */
#define CC1101_FSCAL1           0x25        /* Frequency synthesizer calibration */
#define CC1101_FSCAL0           0x26        /* Frequency synthesizer calibration */
#define CC1101_RCCTRL1          0x27        /* RC oscillator configuration */
#define CC1101_RCCTRL0          0x28        /* RC oscillator configuration */
#define CC1101_FSTEST           0x29        /* Frequency synthesizer cal control */
#define CC1101_PTEST            0x2A        /* Production test */
#define CC1101_AGCTEST          0x2B        /* AGC test */
#define CC1101_TEST2            0x2C        /* Various test settings */
#define CC1101_TEST1            0x2D        /* Various test settings */
#define CC1101_TEST0            0x2E        /* Various test settings */

/* Status registers */

#define CC1101_PARTNUM          (0x30 | 0xc0)   /* Part number */
#define CC1101_VERSION          (0x31 | 0xc0)   /* Current version number */
#define CC1101_FREQEST          (0x32 | 0xc0)   /* Frequency offset estimate */
#define CC1101_LQI              (0x33 | 0xc0)   /* Demodulator estimate for link quality */
#define CC1101_RSSI             (0x34 | 0xc0)   /* Received signal strength indication */
#define CC1101_MARCSTATE        (0x35 | 0xc0)   /* Control state machine state */
#define CC1101_WORTIME1         (0x36 | 0xc0)   /* High byte of WOR timer */
#define CC1101_WORTIME0         (0x37 | 0xc0)   /* Low byte of WOR timer */
#define CC1101_PKTSTATUS        (0x38 | 0xc0)   /* Current GDOx status and packet status */
#define CC1101_VCO_VC_DAC       (0x39 | 0xc0)   /* Current setting from PLL cal module */
#define CC1101_TXBYTES          (0x3A | 0xc0)   /* Underflow and # of bytes in TXFIFO */
#define CC1101_RXBYTES          (0x3B | 0xc0)   /* Overflow and # of bytes in RXFIFO */
#define CC1101_RCCTRL1_STATUS   (0x3C | 0xc0)   /* Last RC oscilator calibration results */
#define CC1101_RCCTRL0_STATUS   (0x3D | 0xc0)   /* Last RC oscilator calibration results */

/* Multi byte memory locations */

#define CC1101_PATABLE          0x3E
#define CC1101_TXFIFO           0x3F
#define CC1101_RXFIFO           0x3F

/* Definitions for burst/single access to registers */

#define CC1101_WRITE_BURST      0x40
#define CC1101_READ_SINGLE      0x80
#define CC1101_READ_BURST       0xC0

/* Strobe commands */

#define CC1101_SRES             0x30        /* Reset chip. */
#define CC1101_SFSTXON          0x31        /* Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). */
#define CC1101_SXOFF            0x32        /* Turn off crystal oscillator. */
#define CC1101_SCAL             0x33        /* Calibrate frequency synthesizer and turn it off */
#define CC1101_SRX              0x34        /* Enable RX. Perform calibration first if switching from IDLE and MCSM0.FS_AUTOCAL=1. */
#define CC1101_STX              0x35        /* Enable TX. Perform calibration first if IDLE and MCSM0.FS_AUTOCAL=1.  */
                                            /* If switching from RX state and CCA is enabled then go directly to TX if channel is clear. */
#define CC1101_SIDLE            0x36        /* Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable. */
#define CC1101_SAFC             0x37        /* Perform AFC adjustment of the frequency synthesizer */
#define CC1101_SWOR             0x38        /* Start automatic RX polling sequence (Wake-on-Radio) */
#define CC1101_SPWD             0x39        /* Enter power down mode when CSn goes high. */
#define CC1101_SFRX             0x3A        /* Flush the RX FIFO buffer. */
#define CC1101_SFTX             0x3B        /* Flush the TX FIFO buffer. */
#define CC1101_SWORRST          0x3C        /* Reset real time clock. */
#define CC1101_SNOP             0x3D        /* No operation. */

/* Modem Control */

#define CC1101_MCSM0_XOSC_FORCE_ON  0x01


/* 
 * Chip Status Byte 
 */

/* Bit fields in the chip status byte */

#define CC1101_STATUS_CHIP_RDYn_BM              0x80
#define CC1101_STATUS_STATE_BM                  0x70
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM   0x0F

/* Chip states */

#define CC1101_STATE_MASK                       0x70
#define CC1101_STATE_IDLE                       0x00
#define CC1101_STATE_RX                         0x10
#define CC1101_STATE_TX                         0x20
#define CC1101_STATE_FSTXON                     0x30
#define CC1101_STATE_CALIBRATE                  0x40
#define CC1101_STATE_SETTLING                   0x50
#define CC1101_STATE_RX_OVERFLOW                0x60
#define CC1101_STATE_TX_UNDERFLOW               0x70

/* Values of the MACRSTATE register */

#define CC1101_MARCSTATE_SLEEP                  0x00
#define CC1101_MARCSTATE_IDLE                   0x01
#define CC1101_MARCSTATE_XOFF                   0x02
#define CC1101_MARCSTATE_VCOON_MC               0x03
#define CC1101_MARCSTATE_REGON_MC               0x04
#define CC1101_MARCSTATE_MANCAL                 0x05
#define CC1101_MARCSTATE_VCOON                  0x06
#define CC1101_MARCSTATE_REGON                  0x07
#define CC1101_MARCSTATE_STARTCAL               0x08
#define CC1101_MARCSTATE_BWBOOST                0x09
#define CC1101_MARCSTATE_FS_LOCK                0x0A
#define CC1101_MARCSTATE_IFADCON                0x0B
#define CC1101_MARCSTATE_ENDCAL                 0x0C
#define CC1101_MARCSTATE_RX                     0x0D
#define CC1101_MARCSTATE_RX_END                 0x0E
#define CC1101_MARCSTATE_RX_RST                 0x0F
#define CC1101_MARCSTATE_TXRX_SWITCH            0x10
#define CC1101_MARCSTATE_RXFIFO_OVERFLOW        0x11
#define CC1101_MARCSTATE_FSTXON                 0x12
#define CC1101_MARCSTATE_TX                     0x13
#define CC1101_MARCSTATE_TX_END                 0x14
#define CC1101_MARCSTATE_RXTX_SWITCH            0x15
#define CC1101_MARCSTATE_TXFIFO_UNDERFLOW       0x16

/* Part number and version */

#define CC1101_PARTNUM_VALUE                    0x00
#define CC1101_VERSION_VALUE                    0x04

/*
 *  Others ... 
 */

#define CC1101_LQI_CRC_OK_BM                    0x80
#define CC1101_LQI_EST_BM                       0x7F

 
/****************************************************************************
 * Private Data Types
 ****************************************************************************/

#define FLAGS_RXONLY        1   /* Indicates receive operation only */
#define FLAGS_XOSCENABLED   2   /* Indicates that one pin is configured as XOSC/n */

struct cc1101_dev_s {
    const struct c1101_rfsettings_s *rfsettings;

    struct spi_dev_s *  spi;    
    uint8_t             isrpin; /* CC1101 pin used to trigger interrupts */
    uint32_t            pinset; /* GPIO of the MCU */
    uint8_t             flags;
    uint8_t             channel;
    uint8_t             power;
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
void cc1101_access_begin(struct cc1101_dev_s * dev)
{
    SPI_LOCK(dev->spi, true);
    SPI_SELECT(dev->spi, SPIDEV_WIRELESS, true);
    SPI_SETMODE(dev->spi, SPIDEV_MODE0);     /* CPOL=0, CPHA=0 */
    SPI_SETBITS(dev->spi, 8);
}


void cc1101_access_end(struct cc1101_dev_s * dev)
{
    SPI_SELECT(dev->spi, SPIDEV_WIRELESS, false);
    SPI_LOCK(dev->spi, false);
}



/** CC1101 Access with Range Check
 * 
 * \param dev CC1101 Private Structure
 * \param addr CC1101 Address
 * \param buf Pointer to buffer, either for read or write access
 * \param length when >0 it denotes read access, when <0 it denotes write 
 *        access of -length. abs(length) greater of 1 implies burst mode, 
 *        however 
 * \return OK on success or errno is set.
 */
int cc1101_access(struct cc1101_dev_s * dev, uint8_t addr, uint8_t *buf, int length)
{
    int stabyte;

    /* Address cannot explicitly define READ command while length WRITE.
     * Also access to these cells is only permitted as one byte, eventhough
     * transfer is marked as BURST! 
     */

    if ( (addr & CC1101_READ_SINGLE) && length != 1 )
        return ERROR;

    /* Prepare SPI */
    
    cc1101_access_begin(dev);
 
    if (length>1 || length < -1)
        SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_BURST);
    else SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_SINGLE);
 
    /* Transfer */
    
    if (length <= 0) {      /* 0 length are command strobes */
        if (length < -1) 
            addr |= CC1101_WRITE_BURST;
            
        stabyte = SPI_SEND(dev->spi, addr);
        if (length) {
            SPI_SNDBLOCK(dev->spi, buf, -length);
        }        
    }
    else {
        addr |= CC1101_READ_SINGLE;
        if (length > 1) 
            addr |= CC1101_READ_BURST;
            
        stabyte = SPI_SEND(dev->spi, addr);
        SPI_RECVBLOCK(dev->spi, buf, length);
    }
    
    cc1101_access_end(dev);

    return stabyte;
}


/** Strobes command and returns chip status byte 
 * 
 *  By default commands are send as Write. To a command, 
 *  CC1101_READ_SINGLE may be OR'ed to obtain the number of RX bytes
 *  pending in RX FIFO.
 */
inline uint8_t cc1101_strobe(struct cc1101_dev_s * dev, uint8_t command)
{
    uint8_t status;

    cc1101_access_begin(dev);
    SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_SINGLE);
    
    status = SPI_SEND(dev->spi, command);
    
    cc1101_access_end(dev);
    
    return status;
}


int cc1101_reset(struct cc1101_dev_s * dev)
{
    cc1101_strobe(dev, CC1101_SRES);
    return OK;
}


int cc1101_checkpart(struct cc1101_dev_s * dev)
{
    uint8_t partnum, version;
    
    if (cc1101_access(dev, CC1101_PARTNUM, &partnum, 1) < 0 ||
        cc1101_access(dev, CC1101_VERSION, &version, 1) < 0) 
        return ERROR;
        
    if (partnum == CC1101_PARTNUM_VALUE && version == CC1101_VERSION_VALUE)
        return OK;
        
    return ERROR;
}


void cc1101_dumpregs(struct cc1101_dev_s * dev, uint8_t addr, uint8_t length)
{
    uint8_t buf[0x30], i;
    
    cc1101_access(dev, addr, buf, length);
    
    printf("CC1101[%2x]: ", addr);
    for (i=0; i<length; i++) printf(" %2x,", buf[i]);
    printf("\n");
}


void cc1101_setpacketctrl(struct cc1101_dev_s * dev)
{
    uint8_t values[3];
    
    values[0] = 0;      /* Rx FIFO threshold = 32, Tx FIFO threshold = 33 */
    cc1101_access(dev, CC1101_FIFOTHR, values, -1);
    
    /* Packet length 
     * Limit it to 61 bytes in total: pktlen, data[61], rssi, lqi 
     */
    
    values[0] = CC1101_PACKET_MAXDATALEN;     
    cc1101_access(dev, CC1101_PKTLEN, values, -1);

    /* Packet Control */
    
    values[0] = 0x04;   /* Append status: RSSI and LQI at the end of received packet */
                        /* TODO: CRC Auto Flash bit 0x08 ??? */
    values[1] = 0x05;   /* CRC in Rx and Tx Enabled: Variable Packet mode, defined by first byte */
                        /* TODO: Enable data whitening ... */
    cc1101_access(dev, CC1101_PKTCTRL1, values, -2);

    /* Main Radio Control State Machine */
    
    values[0] = 0x07;   /* No time-out */
    values[1] = 0x00;   /* Clear channel if RSSI < thr && !receiving;
                         * TX -> RX, RX -> RX: 0x3F */
    values[2] = CC1101_MCSM0_VALUE;   /* Calibrate on IDLE -> RX/TX, OSC Timeout = ~500 us
                           TODO: has XOSC_FORCE_ON */
    cc1101_access(dev, CC1101_MCSM2, values, -3);
    
    /* Wake-On Radio Control */
    
    // Not used yet.
    
    // WOREVT1:WOREVT0 - 16-bit timeout register
}


/****************************************************************************
 * Callbacks
 ****************************************************************************/

volatile int cc1101_interrupt = 0; 
 
/** External line triggers this callback 
 * 
 * The concept todo is:
 *  - GPIO provides EXTI Interrupt
 *  - It should handle EXTI Interrupts in ISR, to which chipcon can
 *    register a callback (and others). The ISR then foreach() calls a 
 *    its callback, and it is up to peripheral to find, whether the cause
 *    of EXTI ISR was itself.
 **/

int cc1101_eventcb(int irq, FAR void *context)
{
  cc1101_interrupt++;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct cc1101_dev_s * cc1101_init(struct spi_dev_s * spi, uint8_t isrpin, 
    uint32_t pinset, const struct c1101_rfsettings_s * rfsettings)
{
    struct cc1101_dev_s * dev;
    
    ASSERT(spi);
    
    if ( (dev = kmalloc( sizeof(struct cc1101_dev_s) )) == NULL) {
        errno = ENOMEM;
        return NULL;
    }
    
    dev->rfsettings = rfsettings;
    dev->spi        = spi;
    dev->isrpin     = isrpin;
    dev->pinset     = pinset;
    dev->flags      = 0;
    dev->channel    = rfsettings->CHMIN;
    dev->power      = rfsettings->PAMAX;
    
    /* Reset chip, check status bytes */
    
    if ( cc1101_reset(dev) < 0 ) {
        kfree(dev);
        errno = EFAULT;
        return NULL;
    }

    /* Check part compatibility */
    
    if ( cc1101_checkpart(dev) < 0 ) {
        kfree(dev);
        errno = ENODEV;
        return NULL;
    }
    
    /* Configure CC1101:
     *  - disable GDOx for best performance
     *  - load RF 
     *  - and packet control
     */
    
    cc1101_setgdo(dev, CC1101_PIN_GDO0, CC1101_GDO_HIZ);
    cc1101_setgdo(dev, CC1101_PIN_GDO1, CC1101_GDO_HIZ);
    cc1101_setgdo(dev, CC1101_PIN_GDO2, CC1101_GDO_HIZ);
    cc1101_setrf(dev, rfsettings);
    cc1101_setpacketctrl(dev);
        
    /* Set the ISR to be triggerred on falling edge of the: 
     * 
     * 6 (0x06) Asserts when sync word has been sent / received, and 
     * de-asserts at the end of the packet. In RX, the pin will de-assert
     * when the optional address check fails or the RX FIFO overflows. 
     * In TX the pin will de-assert if the TX FIFO underflows.
     */

    cc1101_setgdo(dev, dev->isrpin, CC1101_GDO_SYNC);
    
    /* Bind to external interrupt line */

    // depends on STM32: TODO: Make that config within pinset and
    // provide general gpio interface 
    //stm32_gpiosetevent(pinset, false, true, true, cc1101_eventcb);

    return dev;
}


int cc1101_deinit(struct cc1101_dev_s * dev)
{
    ASSERT(dev);
    
    /* Release interrupt */
    //stm32_gpiosetevent(pinset, false, false, false, NULL);

    /* Power down chip */
    cc1101_powerdown(dev);
    
    /* Release external interrupt line */
    kfree(dev);

    return 0;
}


int cc1101_powerup(struct cc1101_dev_s * dev)
{
    ASSERT(dev);
    return 0;
}


int cc1101_powerdown(struct cc1101_dev_s * dev)
{
    ASSERT(dev);
    return 0;
}


int cc1101_setgdo(struct cc1101_dev_s * dev, uint8_t pin, uint8_t function)
{
    ASSERT(dev);
    ASSERT(pin <= CC1101_IOCFG0);
    
    if (function >= CC1101_GDO_CLK_XOSC1) {
    
        /* Only one pin can be enabled at a time as XOSC/n */
    
        if (dev->flags & FLAGS_XOSCENABLED) return -EPERM;
     
        /* Force XOSC to stay active even in sleep mode */
        
        int value = CC1101_MCSM0_VALUE | CC1101_MCSM0_XOSC_FORCE_ON;
        cc1101_access(dev, CC1101_MCSM0, &value, -1);
        
        dev->flags |= FLAGS_XOSCENABLED;
    }
    else if (dev->flags & FLAGS_XOSCENABLED) {
    
        /* Disable XOSC in sleep mode */
    
        int value = CC1101_MCSM0_VALUE;
        cc1101_access(dev, CC1101_MCSM0, &value, -1);
        
        dev->flags &= ~FLAGS_XOSCENABLED;
    }
    
    return cc1101_access(dev, pin, &function, -1);
}


int cc1101_setrf(struct cc1101_dev_s * dev, const struct c1101_rfsettings_s *settings)
{
    ASSERT(dev);
    ASSERT(settings);
    
    if (cc1101_access(dev, CC1101_FSCTRL1, &settings->FSCTRL1, -11) < 0) return ERROR;
    if (cc1101_access(dev, CC1101_FOCCFG,  &settings->FOCCFG,  -5)  < 0) return ERROR;
    if (cc1101_access(dev, CC1101_FREND1,  &settings->FREND1,  -6)  < 0) return ERROR;
    
    /* Load Power Table */
    
    if (cc1101_access(dev, CC1101_PATABLE, settings->PA, -8) < 0) return ERROR;
    
    /* If channel is out of valid range, mark that. Limit power.
     * We are not allowed to send any data, but are allowed to listen
     * and receive.
     */

    cc1101_setchannel(dev, dev->channel);
    cc1101_setpower(dev, dev->power);
        
    return OK;
}


int cc1101_setchannel(struct cc1101_dev_s * dev, uint8_t channel)
{
    ASSERT(dev);
    
    /* Store localy in further checks */
    
    dev->channel = channel;
    
    /* If channel is out of valid, we are allowed to listen and receive only */
    
    if (channel < dev->rfsettings->CHMIN || channel > dev->rfsettings->CHMAX)
        dev->flags |= FLAGS_RXONLY;
    else dev->flags &= ~FLAGS_RXONLY;
    
    cc1101_access(dev, CC1101_CHANNR, &dev->channel, -1);
    
    return dev->flags & FLAGS_RXONLY;
}


uint8_t cc1101_setpower(struct cc1101_dev_s * dev, uint8_t power)
{
    ASSERT(dev);

    if (power > dev->rfsettings->PAMAX) 
        power = dev->rfsettings->PAMAX;
        
    dev->power = power;
        
    if (power == 0) {
        dev->flags |= FLAGS_RXONLY;
        return 0;
    }
    else dev->flags &= ~FLAGS_RXONLY;
    
    /* Add remaining part from RF table (to get rid of readback) */
    
    power--;
    power |= dev->rfsettings->FREND0;
    
    /* On error, report that as zero power */
    
    if (cc1101_access(dev, CC1101_FREND0, &power, -1) < 0) 
        dev->power = 0;
    
    return dev->power;
}


int cc1101_calcRSSIdBm(int rssi)
{
    if (rssi >= 128) rssi -= 256;
    return (rssi >> 1) - 74;
}


int cc1101_receive(struct cc1101_dev_s * dev)
{
    ASSERT(dev);

    /* \todo Wait for IDLE before going into another state? */

    cc1101_interrupt = 0;
    
    cc1101_strobe(dev, CC1101_SRX | CC1101_READ_SINGLE);
    
    return 0;
}


int cc1101_read(struct cc1101_dev_s * dev, uint8_t * buf, size_t size)
{
    ASSERT(dev);
    
    if (buf==NULL) {
        if (size==0) return 64;
        // else received packet size
        return 0;
    }
    
    if (cc1101_interrupt == 0) return 0;
    
    int status = cc1101_strobe(dev, CC1101_SNOP | CC1101_READ_SINGLE);
    
    if (status & CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM && 
        (status & CC1101_STATE_MASK) == CC1101_STATE_IDLE) {
        
        uint8_t nbytes;
        
        cc1101_access(dev, CC1101_RXFIFO, &nbytes, 1);
        
        nbytes += 2;    /* RSSI and LQI */
        
        cc1101_access(dev, CC1101_RXFIFO, buf, (nbytes > size) ? size : nbytes);
        
        /* Flush remaining bytes, if there is no room to receive 
         * or if there is a BAD CRC
         */
         
        if (nbytes > size || (nbytes <= size && !(buf[nbytes-1]&0x80)) ) {
            printf("Flushing RX FIFO\n");
            cc1101_strobe(dev, CC1101_SFRX);
        }
        
        return nbytes;
    }
    
    return 0;
}


int cc1101_write(struct cc1101_dev_s * dev, const uint8_t * buf, size_t size)
{
    uint8_t packetlen;

    ASSERT(dev);
    ASSERT(buf);

    if (dev->flags & FLAGS_RXONLY) return -EPERM;
    
    /* Present limit */
    if (size > CC1101_PACKET_MAXDATALEN) 
        packetlen = CC1101_PACKET_MAXDATALEN; 
    else packetlen = size;
    
    cc1101_access(dev, CC1101_TXFIFO, &packetlen, -1);
    cc1101_access(dev, CC1101_TXFIFO, buf, -size);

    return 0;
}


int cc1101_send(struct cc1101_dev_s * dev)
{
    ASSERT(dev);
    
    if (dev->flags & FLAGS_RXONLY) return -EPERM;
    
    cc1101_interrupt = 0;
    
    cc1101_strobe(dev, CC1101_STX);
    
    /* wait until send, going to IDLE */
    
    while( cc1101_interrupt == 0 );

    return 0;
}


int cc1101_idle(struct cc1101_dev_s * dev)
{
    ASSERT(dev);
    cc1101_strobe(dev, CC1101_SIDLE);
    return 0;
}
