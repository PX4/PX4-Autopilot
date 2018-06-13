/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

/**
 * @file amov_imu_uart.cpp
 *
 * Driver for the AMOV IMU connected via UART.
 *
 * @author Jin Wu (Zarathustra)
 * @web: www.jinwu.science
 */


#include <px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_mag.h>
#include <termios.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <board_config.h>
#include <drivers/device/spi.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
                       
#include "amov_imu.h"


class AMOV_IMU_UART_DMA : public device::Device
{
public:
    AMOV_IMU_UART_DMA(const char * uart_name, const char * uart_alias, const char * model_name, bool is_dma);
    ~AMOV_IMU_UART_DMA();

    virtual int	init();
    virtual int	ioctl(unsigned operation, unsigned &arg);
    virtual int read(unsigned address, void * data, unsigned count);

    virtual int	exchange(IMU3DMPacket *_packet);

private:
    uint32_t                    _uart_addr;
    uint32_t                    _uart_irq_vector;
    uint32_t                    _uart_tx_dma_addr;
    uint32_t                    _uart_rx_dma_addr;
    uint32_t                    _uart_rcc_reg;
    uint32_t                    _uart_rcc_en;
    uint32_t                    _uart_tx_gpio;
    uint32_t                    _uart_rx_gpio;

    DMA_HANDLE                  _tx_dma;
    DMA_HANDLE                  _rx_dma;

    bool                        _is_dma;

    IMU3DMPacket                *_current_packet;

    int                         _device_type;
    int                         _uart_fd;

    char                        _uart_port[32];
    char                        _model_name[32];

    uint8_t                     _imu_buf_length;
    uint8_t                     _imu_buf_head;

    /** saved DMA status */
    static const unsigned	_dma_status_inactive = 0x80000000;	// low bits overlap DMA_STATUS_* values
    static const unsigned	_dma_status_waiting  = 0x00000000;
    volatile unsigned           _rx_dma_status;

    /** client-waiting lock/signal */
    px4_sem_t			_completion_semaphore;
    bool                        _is_first;

    /**
     * DMA completion handler.
     */
    static void                 _dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
    void			_do_rx_dma_callback(unsigned status);

    /**
     * Serial interrupt handler.
     */
    static int                  _interrupt(int vector, void *context, void *args);
    void			_do_interrupt();

    /**
     * Cancel any DMA in progress with an error.
     */
    void			_abort_dma();

    hrt_abstime                 _last_time;


    void                        setBaud(unsigned baud, int _serial_fd);

    /* do not allow top copying this class */
    AMOV_IMU_UART_DMA(AMOV_IMU_UART_DMA &);
    AMOV_IMU_UART_DMA &operator = (const AMOV_IMU_UART_DMA &);
};

device::Device *
AMOV_IMU_UART_interface(const char * uart_name, const char * uart_alias, const char * model_name, bool is_dma)
{
	device::Device *interface = new AMOV_IMU_UART_DMA(uart_name, uart_alias, model_name, is_dma);
	return interface;
}

/* serial register accessors */
#define REG(uart_base, _x)      (*(volatile uint32_t *)(uart_base + _x))
#define rSR                     REG(_uart_addr, STM32_USART_SR_OFFSET)
#define rDR                     REG(_uart_addr, STM32_USART_DR_OFFSET)
#define rBRR                    REG(_uart_addr, STM32_USART_BRR_OFFSET)
#define rCR1                    REG(_uart_addr, STM32_USART_CR1_OFFSET)
#define rCR2                    REG(_uart_addr, STM32_USART_CR2_OFFSET)
#define rCR3                    REG(_uart_addr, STM32_USART_CR3_OFFSET)
#define rGTPR                   REG(_uart_addr, STM32_USART_GTPR_OFFSET)

uint32_t _irq_list[MAX_DEV_NUM] = {0, 0, 0};
AMOV_IMU_UART_DMA * __uart[MAX_DEV_NUM] = {nullptr, nullptr, nullptr};

AMOV_IMU_UART_DMA::AMOV_IMU_UART_DMA(const char * uart_name, const char * uart_alias, const char * model_name, bool is_dma) :
    Device("AMOV_IMU_UART_DMA"),
    _tx_dma(nullptr),
    _rx_dma(nullptr),
    _is_dma(is_dma),
    _current_packet(nullptr),
    _uart_fd(-1),
    _rx_dma_status(_dma_status_inactive),
    _completion_semaphore(SEM_INITIALIZER(0)),
    _is_first(true),
    _last_time(0)
{
    if(!strcmp(uart_name, "USART1"))
    {
        _uart_addr = STM32_USART1_BASE;
        _uart_irq_vector = STM32_IRQ_USART1;
        _uart_tx_dma_addr =  DMAMAP_USART1_TX;
        _uart_rx_dma_addr = DMAMAP_USART1_RX;
        _uart_rcc_reg = STM32_RCC_APB2ENR;
        _uart_rcc_en = RCC_APB2ENR_USART1EN;
        _uart_tx_gpio = GPIO_USART1_TX;
        _uart_rx_gpio = GPIO_USART1_RX;
    }
    else if(!strcmp(uart_name, "USART2"))
    {
        _uart_addr = STM32_USART2_BASE;
        _uart_irq_vector = STM32_IRQ_USART2;
        _uart_tx_dma_addr =  DMAMAP_USART2_TX;
        _uart_rx_dma_addr = DMAMAP_USART2_RX;
        _uart_rcc_reg = STM32_RCC_APB1ENR;
        _uart_rcc_en = RCC_APB1ENR_USART2EN;
        _uart_tx_gpio = GPIO_USART2_TX;
        _uart_rx_gpio = GPIO_USART2_RX;
    }
    else if(!strcmp(uart_name, "USART3"))
    {
        _uart_addr = STM32_USART3_BASE;
        _uart_irq_vector = STM32_IRQ_USART3;
        _uart_tx_dma_addr =  DMAMAP_USART3_TX_2;
        _uart_rx_dma_addr = DMAMAP_USART3_RX;
        _uart_rcc_reg = STM32_RCC_APB1ENR;
        _uart_rcc_en = RCC_APB1ENR_USART3EN;
        _uart_tx_gpio = GPIO_USART3_TX;
        _uart_rx_gpio = GPIO_USART3_RX;
    }
    else if(!strcmp(uart_name, "UART4"))
    {
        _uart_addr = STM32_UART4_BASE;
        _uart_irq_vector = STM32_IRQ_UART4;
        _uart_tx_dma_addr =  DMAMAP_UART4_TX;
        _uart_rx_dma_addr = DMAMAP_UART4_RX;
        _uart_rcc_reg = STM32_RCC_APB1ENR;
        _uart_rcc_en = RCC_APB1ENR_UART4EN;
        _uart_tx_gpio = GPIO_UART4_TX;
        _uart_rx_gpio = GPIO_UART4_RX;
    }
    else if(!strcmp(uart_name, "USART6"))
    {
        _uart_addr = STM32_USART6_BASE;
        _uart_irq_vector = STM32_IRQ_USART6;
        _uart_tx_dma_addr =  DMAMAP_USART6_TX_1;
        _uart_rx_dma_addr = DMAMAP_USART6_RX_1;
        _uart_rcc_reg = STM32_RCC_APB2ENR;
        _uart_rcc_en = RCC_APB2ENR_USART6EN;
        _uart_tx_gpio = GPIO_USART6_TX;
        _uart_rx_gpio = GPIO_USART6_RX;
    }
    else if(!strcmp(uart_name, "UART7"))
    {
        _uart_addr = STM32_UART7_BASE;
        _uart_irq_vector = STM32_IRQ_UART7;
        _uart_tx_dma_addr =  DMAMAP_UART7_TX;
        _uart_rx_dma_addr = DMAMAP_UART7_RX;
        _uart_rcc_reg = STM32_RCC_APB1ENR;
        _uart_rcc_en = RCC_APB1ENR_UART7EN;
        _uart_tx_gpio = GPIO_UART7_TX;
        _uart_rx_gpio = GPIO_UART7_RX;
    }
    else if(!strcmp(uart_name, "UART8"))
    {
        _uart_addr = STM32_UART8_BASE;
        _uart_irq_vector = STM32_IRQ_UART8;
        _uart_tx_dma_addr =  DMAMAP_UART8_TX;
        _uart_rx_dma_addr = DMAMAP_UART8_RX;
        _uart_rcc_reg = STM32_RCC_APB1ENR;
        _uart_rcc_en = RCC_APB1ENR_UART8EN;
        _uart_tx_gpio = GPIO_UART8_TX;
        _uart_rx_gpio = GPIO_UART8_RX;
    }

    strcpy(_uart_port, uart_alias);
    strcpy(_model_name, model_name);

    if(0 == strcmp(_model_name, "AMOV-IMU-1"))
    {
        _imu_buf_length = 32;
        _imu_buf_head = 0xA5;
    }

    int index;
    bool found = false;
    for(index = 0; index < MAX_DEV_NUM; ++index)
        if(_irq_list[index] == 0)
        {
            found = true;
            break;
        }
    if(!found)
        return;

    __uart[index] = this;
    _irq_list[index] = _uart_irq_vector;

    if(_is_dma)
        warnx("AMOV_IMU DMA Enabled on %s i.e. %s", uart_name, _uart_port);
    warnx("AMOV_IMU UART Class Constructed");
}

AMOV_IMU_UART_DMA::~AMOV_IMU_UART_DMA()
{
    if(_is_dma)
    {

        if (_tx_dma != nullptr) {
            stm32_dmastop(_tx_dma);
            stm32_dmafree(_tx_dma);
        }

        if (_rx_dma != nullptr) {
            stm32_dmastop(_rx_dma);
            stm32_dmafree(_rx_dma);
        }

        /* reset the UART */
        rCR1 = 0;
        rCR2 = 0;
        rCR3 = 0;

        int index;
        bool found = false;
        for(index = 0; index < MAX_DEV_NUM; ++index)
        if(_irq_list[index] == _uart_irq_vector)
        {
            found = true;
            break;
        }
        if(!found)
            return;

        _irq_list[index] = 0;

        /* detach our interrupt handler */
        up_disable_irq(_uart_irq_vector);
        irq_detach(_uart_irq_vector);

        /* restore the GPIOs */
        px4_arch_unconfiggpio(_uart_tx_gpio);
        px4_arch_unconfiggpio(_uart_rx_gpio);

        /* Disable APB clock for the USART peripheral */
        modifyreg32(_uart_rcc_reg, _uart_rcc_en, 0);

        /* and kill our semaphores */
        px4_sem_destroy(&_completion_semaphore);
    }
    else
    {
        if(_uart_fd >= 0)
            ::close(_uart_fd);
    }

}

void AMOV_IMU_UART_DMA::setBaud(unsigned baud, int _serial_fd)
{
    int speed;

    switch (baud) {
    case 9600:   speed = B9600;   break;

    case 19200:  speed = B19200;  break;

    case 38400:  speed = B38400;  break;

    case 57600:  speed = B57600;  break;

    case 115200: speed = B115200; break;

    case 230400: speed = B230400; break;

    case 460800: speed = B460800; break;

    case 921600: speed = B921600; break;

    case 1500000: speed = B1500000; break;

    default:
        PX4_ERR("ERR: unknown baudrate: %d", baud);
        return;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(_serial_fd, &uart_config);

    uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                 INLCR | PARMRK | INPCK | ISTRIP | IXON);

    uart_config.c_oflag = 0;

    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        return;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        return;
    }

    if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
        return;
    }
}

int
AMOV_IMU_UART_DMA::init()
{
    if(_uart_fd < 0)
        _uart_fd = open(_uart_port, O_RDWR);
    
    if(_uart_fd < 0)
        return -EINVAL;
        
    unsigned baud = 1500000;
    setBaud(baud, _uart_fd);

    ::close(_uart_fd);

    if(_is_dma)
    {
        warnx("DMA TX addr: %08x", (uintptr_t) &_tx_dma);
        warnx("DMA RX addr: %08x", (uintptr_t) &_rx_dma);
        warnx("Class addr: %08x", (uintptr_t) this);
        /* allocate DMA */
        _tx_dma = stm32_dmachannel(_uart_tx_dma_addr);
        _rx_dma = stm32_dmachannel(_uart_rx_dma_addr);

        if ((_tx_dma == nullptr) || (_rx_dma == nullptr)) {
            warnx("DMA Allocation Failed");
            return -1;
        }

        warnx("DMA Allocation Success");

        /* Enable the APB clock for the USART peripheral */
        modifyreg32(_uart_rcc_reg, 0, _uart_rcc_en);

        /* configure pins for serial use */
        px4_arch_configgpio(_uart_tx_gpio);
        px4_arch_configgpio(_uart_rx_gpio);

        /* reset & configure the UART */
        rCR1 = 0;
        rCR2 = 0;
        rCR3 = 0;

        /* eat any existing interrupt status */
        (void)rSR;
        (void)rDR;

        /* attach serial interrupt handler */
        irq_attach(_uart_irq_vector, _interrupt, nullptr);
        up_enable_irq(_uart_irq_vector);

        /* enable UART in DMA mode, enable error and line idle interrupts */
        rCR3 = USART_CR3_EIE;

        rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

        /* create semaphores */
        px4_sem_init(&_completion_semaphore, 0, 0);

        /* _completion_semaphore use case is a signal */

        px4_sem_setprotocol(&_completion_semaphore, SEM_PRIO_NONE);

        /* XXX this could try talking to IO */
    }
    else
    {
        if(_uart_fd < 0)
            _uart_fd = ::open(_uart_port, O_RDWR);
    
        if(_uart_fd < 0)
            return -EINVAL;
    }

    return 0;
}

int
AMOV_IMU_UART_DMA::ioctl(unsigned operation, unsigned &arg)
{
    switch (operation) {

    case ACCELIOCGEXTERNAL:
        return 0;

    default:
        break;
    }

    return -1;
}

int
AMOV_IMU_UART_DMA::read(unsigned address, void * data, unsigned count)
{
    int result = -EINVAL;

    if(!data)
        return result;

    if(_is_dma)
    {

        if(_is_first)
        {
            int retry = 50;
            while(retry > 0)
            {
                int retries = 500;
                while(!(rSR & USART_SR_RXNE) && retries > 0)
                {
                    --retries;
                }
                
                uint8_t tmp = rDR;
                if(tmp == _imu_buf_head)
                    break;
                --retry;
            }

            if(retry == 0)
                return -EIO;
            _is_first = false;
        }
        IMU3DMPacket *transaction = (IMU3DMPacket *) data;
        result = exchange(transaction);

            /* successful transaction? */
        if (result != OK || !(transaction->buf[0] == _imu_buf_head)) {
            result = -EINVAL;
        }
        else
            result = count;
    }
    else
    {
        pollfd fds[1];

        fds[0].fd = _uart_fd;
        fds[0].events = POLLIN;

        ::poll(fds, 1, 20000000);

        int err = 0, bytesAvailable = 0;
        err = ::ioctl(_uart_fd, FIONREAD, (unsigned long) &bytesAvailable);

        if ((err != 0) || (bytesAvailable < (int)(count * 2))) {
            warnx("Received %d", bytesAvailable);
            return result;
        }

        int res = ::read(_uart_fd, data, count);
        warnx("Received %d", bytesAvailable);

        result = res;
    }

    return result;
}

int
AMOV_IMU_UART_DMA::exchange(IMU3DMPacket *_packet)
{
    _current_packet = _packet;

    /* clear any lingering error status */
    (void)rSR;
    (void)rDR;

    /* DMA setup time ~3µs */
    _rx_dma_status = _dma_status_waiting;

    /*
     * Note that we enable circular buffer mode as a workaround for
     * there being no API to disable the DMA FIFO. We need direct mode
     * because otherwise when the line idle interrupt fires there
     * will be packet bytes still in the DMA FIFO, and we will assume
     * that the idle was spurious.
     *
     * XXX this should be fixed with a NuttX change.
     */
    stm32_dmasetup(
        _rx_dma,
        _uart_addr + STM32_USART_DR_OFFSET,
        reinterpret_cast<uint32_t>(_current_packet),
        _imu_buf_length,
        0		|	/* XXX see note above */
        DMA_SCR_DIR_P2M		|
        DMA_SCR_MINC		|
        DMA_SCR_PSIZE_8BITS	|
        DMA_SCR_MSIZE_8BITS	|
        DMA_SCR_PBURST_SINGLE	|
        DMA_SCR_MBURST_SINGLE);
    stm32_dmastart(_rx_dma, _dma_callback, this, false);
    rCR3 |= USART_CR3_DMAR;

    /* wait for the transaction to complete - 32 bytes @ 1.5Mbps ~210µs ~ 5KHz */
    int ret;

    for (;;) {
        ret = px4_sem_wait(&_completion_semaphore);

        if (ret == OK) {
            /* check for DMA errors */
            if (_rx_dma_status & DMA_STATUS_TEIF) {
                ret = -EIO;
                break;
            }

            /* successful txn (may still be reporting an error) */
            break;
        }

        if (errno == ETIMEDOUT) {
            /* something has broken - clear out any partial DMA state and reconfigure */
            _abort_dma();
            break;
        }
    }
    /* reset DMA status */
    _rx_dma_status = _dma_status_inactive;

    return ret;
}

void
AMOV_IMU_UART_DMA::_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
    if (arg != nullptr) {
        AMOV_IMU_UART_DMA *ps = reinterpret_cast<AMOV_IMU_UART_DMA *>(arg);

        ps->_do_rx_dma_callback(status);
    }
}

void
AMOV_IMU_UART_DMA::_do_rx_dma_callback(unsigned status)
{
    /* on completion of a reply, wake the waiter */
    if (_rx_dma_status == _dma_status_waiting) {

        /* check for packet overrun - this will occur after DMA completes */
        uint32_t sr = rSR;

        if (sr & (USART_SR_ORE | USART_SR_RXNE)) {
            (void)rDR;
            status = DMA_STATUS_TEIF;
        }

        /* save RX status */
        _rx_dma_status = status;

        /* disable UART DMA */
        rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

        /* complete now */
        px4_sem_post(&_completion_semaphore);
    }
}

int
AMOV_IMU_UART_DMA::_interrupt(int irq, void *context, void *args)
{
    int index;
    bool found = false;
    for(index = 0; index < MAX_DEV_NUM; ++index)
        if(_irq_list[index] == (uint32_t)irq)
        {
            found = true;
            break;
        }
    if(!found)
        return -1;
    __uart[index]->_do_interrupt();

    return 0;
}

void
AMOV_IMU_UART_DMA::_do_interrupt()
{
    uint32_t sr = rSR;	/* get UART status register */
    (void)rDR;		/* read DR to clear status */

    if (sr & (USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
          USART_SR_NE |		/* noise error - we have lost a byte due to noise */
          USART_SR_FE)) {		/* framing error - start/stop bit lost or line break */

        /*
         * If we are in the process of listening for something, these are all fatal;
         * abort the DMA with an error.
         */
        if (_rx_dma_status == _dma_status_waiting) {
            _abort_dma();
            /* complete DMA as though in error */
            _do_rx_dma_callback(DMA_STATUS_TEIF);

            return;
        }

        /* XXX we might want to use FE / line break as an out-of-band handshake ... handle it here */

        /* don't attempt to handle IDLE if it's set - things went bad */
        return;
    }

    if (sr & USART_SR_IDLE) {

        /* if there is DMA reception going on, this is a short packet */
        if (_rx_dma_status == _dma_status_waiting) {

            /* verify that the received packet is complete */
            size_t length = _imu_buf_length - stm32_dmaresidual(_rx_dma);

            if ((length < 1)) {
                /* stop the receive DMA */
                stm32_dmastop(_rx_dma);

                /* complete the short reception */
                _do_rx_dma_callback(DMA_STATUS_TEIF);
                return;
            }
            /* stop the receive DMA */
            stm32_dmastop(_rx_dma);

            /* complete the short reception */
            _do_rx_dma_callback(DMA_STATUS_TCIF);
        }
    }
}

void
AMOV_IMU_UART_DMA::_abort_dma()
{
    /* disable UART DMA */
    rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
    (void)rSR;
    (void)rDR;
    (void)rDR;

    /* stop DMA */
    stm32_dmastop(_tx_dma);
    stm32_dmastop(_rx_dma);
}

IMU3DMPacket _io_buffer_storage[MAX_DEV_NUM];
