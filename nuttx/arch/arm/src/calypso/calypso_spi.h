#ifndef ___ARCH_ARM_SRC_CALYPSO_CALYPSO_SPI_H
#define ___ARCH_ARM_SRC_CALYPSO_CALYPSO_SPI_H

#define BASE_ADDR_SPI   0xfffe3000
#define SPI_REG(n)      (BASE_ADDR_SPI+(n))

enum spi_regs {
        REG_SET1        = 0x00,
        REG_SET2        = 0x02,
        REG_CTRL        = 0x04,
        REG_STATUS      = 0x06,
        REG_TX_LSB      = 0x08,
        REG_TX_MSB      = 0x0a,
        REG_RX_LSB      = 0x0c,
        REG_RX_MSB      = 0x0e,
};

#define SPI_SET1_EN_CLK         (1 << 0)
#define SPI_SET1_WR_IRQ_DIS     (1 << 4)
#define SPI_SET1_RDWR_IRQ_DIS   (1 << 5)

#define SPI_CTRL_RDWR           (1 << 0)
#define SPI_CTRL_WR             (1 << 1)
#define SPI_CTRL_NB_SHIFT       2
#define SPI_CTRL_AD_SHIFT       7

#define SPI_STATUS_RE           (1 << 0)        /* Read End */
#define SPI_STATUS_WE           (1 << 1)        /* Write End */

#endif /* ___ARCH_ARM_SRC_CALYPSO_CALYPSO_SPI_H */
