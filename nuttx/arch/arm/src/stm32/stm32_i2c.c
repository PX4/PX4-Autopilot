/************************************************************************************
 * arch/arm/src/stm32/stm32_i2c.c
 * STM32 I2C Hardware Layer - Device Driver
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With extensions, modifications by:
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregroy Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/* Supports:
 *  - Master operation, 100 kHz (standard) and 400 kHz (full speed)
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 * 
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained by
 *      the i2c_init(); it extends the Device structure from the nuttx/i2c/i2c.h; 
 *      Instance points to OPS, to common I2C Hardware private data and contains
 *      its own private data, as frequency, address, mode of operation (in the future)
 *  - Private: Private data of an I2C Hardware
 * 
 * TODO
 *  - Check for all possible deadlocks (as BUSY='1' I2C needs to be reset in HW using the I2C_CR1_SWRST)
 *  - SMBus support (hardware layer timings are already supported) and add SMBA gpio pin
 *  - Slave support with multiple addresses (on multiple instances):
 *      - 2 x 7-bit address or 
 *      - 1 x 10 bit adresses + 1 x 7 bit address (?)
 *      - plus the broadcast address (general call)
 *  - Multi-master support
 *  - DMA (to get rid of too many CPU wake-ups and interventions)
 *  - Be ready for IPMI
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "stm32_rcc.h"
#include "stm32_i2c.h"
#include "stm32_waste.h"

#if defined(CONFIG_STM32_I2C1) || defined(CONFIG_STM32_I2C2) || defined(CONFIG_STM32_I2C3)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* CONFIG_I2C_POLLED may be set so that I2C interrrupts will not be used.  Instead,
 * CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_STM32_I2CTIMEOSEC) && !defined(CONFIG_STM32_I2CTIMEOMS)
#  define CONFIG_STM32_I2CTIMEOSEC 0
#  define CONFIG_STM32_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_STM32_I2CTIMEOSEC)
#  define CONFIG_STM32_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_STM32_I2CTIMEOMS)
#  define CONFIG_STM32_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_STM32_I2CTIMEOTICKS
#  define CONFIG_STM32_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_STM32_I2CTIMEOSEC) + MSEC2TICK(CONFIG_STM32_I2CTIMEOMS))
#endif

#ifndef CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_STM32_I2CTIMEOTICKS)
#endif

/* On the STM32F103ZE, there is an internal conflict between I2C1 and FSMC.  In that
 * case, it is necessary to disable FSMC before each I2C1 access and re-enable FSMC
 * when the I2C access completes.
 */

#undef I2C1_FSMC_CONFLICT
#if defined(CONFIG_STM32_STM32F10XX) && defined(CONFIG_STM32_FSMC) && defined(CONFIG_STM32_I2C1)
#  define I2C1_FSMC_CONFLICT
#endif

/* Macros to convert a I2C pin to a GPIO output */

#if defined(CONFIG_STM32_STM32F10XX)
#  define I2C_OUTPUT (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_CNF_OUTOD | \
                      GPIO_MODE_50MHz)
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define I2C_OUTPUT (GPIO_OUTPUT | GPIO_FLOAT | GPIO_OPENDRAIN |\
                      GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#endif

#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

/* Debug ****************************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#  define i2cdbg dbg
#  define i2cvdbg vdbg
#else
#  define i2cdbg(x...)
#  define i2cvdbg(x...)
#endif

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard, low-level
 * debug interface syslog() but does not require that any other debug
 * is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define stm32_i2c_tracereset(p)
#  define stm32_i2c_tracenew(p,s)
#  define stm32_i2c_traceevent(p,e,a)
#  define stm32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/
/* Interrupt state */

enum stm32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum stm32_trace_e
{
  I2CEVENT_NONE = 0,      /* No events have occurred with this status */
  I2CEVENT_SENDADDR,      /* Start/Master bit set and address sent, param = msgc */
  I2CEVENT_SENDBYTE,      /* Send byte, param = dcnt */
  I2CEVENT_ITBUFEN,       /* Enable buffer interrupts, param = 0 */
  I2CEVENT_RCVBYTE,       /* Read more dta, param = dcnt */
  I2CEVENT_REITBUFEN,     /* Re-enable buffer interrupts, param = 0 */
  I2CEVENT_DISITBUFEN,    /* Disable buffer interrupts, param = 0 */
  I2CEVENT_BTFNOSTART,    /* BTF on last byte with no restart, param = msgc */
  I2CEVENT_BTFRESTART,    /* Last byte sent, re-starting, param = msgc */
  I2CEVENT_BTFSTOP,       /* Last byte sten, send stop, param = 0 */
  I2CEVENT_ERROR          /* Error occurred, param = 0 */
};

/* Trace data */

struct stm32_trace_s
{
  uint32_t status;             /* I2C 32-bit SR2|SR1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum stm32_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  uint32_t time;               /* First of event or first status */
};

/* I2C Device hardware configuration */

struct stm32_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t reset_bit;         /* Reset bit */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  int (*isr)(int, void *);    /* Interrupt handler */
  uint32_t ev_irq;            /* Event IRQ */
  uint32_t er_irq;            /* Error IRQ */
#endif
};

/* I2C Device Private Data */

struct stm32_i2c_priv_s
{
  const struct stm32_i2c_config_s *config; /* Port configuration */
  int refs;                    /* Referernce count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum stm32_intstate_e) */
    
  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  int dcnt;                    /* Current message length */
  uint16_t flags;              /* Current message flags */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  uint32_t start_time;         /* Time when the trace was started */

  /* The actual trace data */

  struct stm32_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */
};

/* I2C Device, Instance */

struct stm32_i2c_inst_s
{
  struct i2c_ops_s        *ops;  /* Standard I2C operations */
  struct stm32_i2c_priv_s *priv; /* Common driver private data structure */
    
  uint32_t    frequency;   /* Frequency used in this instantiation */
  int         address;     /* Address used in this instantiation */
  uint16_t    flags;       /* Flags used in this instantiation */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static inline uint16_t stm32_i2c_getreg(FAR struct stm32_i2c_priv_s *priv,
                                        uint8_t offset);
static inline void stm32_i2c_putreg(FAR struct stm32_i2c_priv_s *priv, uint8_t offset,
                                    uint16_t value);
static inline void stm32_i2c_modifyreg(FAR struct stm32_i2c_priv_s *priv,
                                       uint8_t offset, uint16_t clearbits,
                                       uint16_t setbits);
static inline void stm32_i2c_sem_wait(FAR struct i2c_dev_s *dev);
#ifdef CONFIG_STM32_I2C_DYNTIMEO
static useconds_t stm32_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs);
#endif /* CONFIG_STM32_I2C_DYNTIMEO */
static inline int  stm32_i2c_sem_waitdone(FAR struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sem_waitstop(FAR struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sem_post(FAR struct i2c_dev_s *dev);
static inline void stm32_i2c_sem_init(FAR struct i2c_dev_s *dev);
static inline void stm32_i2c_sem_destroy(FAR struct i2c_dev_s *dev);
#ifdef CONFIG_I2C_TRACE
static void stm32_i2c_tracereset(FAR struct stm32_i2c_priv_s *priv);
static void stm32_i2c_tracenew(FAR struct stm32_i2c_priv_s *priv, uint32_t status);
static void stm32_i2c_traceevent(FAR struct stm32_i2c_priv_s *priv,
                               enum stm32_trace_e event, uint32_t parm);
static void stm32_i2c_tracedump(FAR struct stm32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */
static void stm32_i2c_setclock(FAR struct stm32_i2c_priv_s *priv,
                               uint32_t frequency);
static inline void stm32_i2c_sendstart(FAR struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_clrstart(FAR struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sendstop(FAR struct stm32_i2c_priv_s *priv);
static inline uint32_t stm32_i2c_getstatus(FAR struct stm32_i2c_priv_s *priv);
#ifdef I2C1_FSMC_CONFLICT
static inline uint32_t stm32_i2c_disablefsmc(FAR struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_enablefsmc(uint32_t ahbenr);
#endif /* I2C1_FSMC_CONFLICT */
static int stm32_i2c_isr(struct stm32_i2c_priv_s * priv);
#ifndef CONFIG_I2C_POLLED
#ifdef CONFIG_STM32_I2C1
static int stm32_i2c1_isr(int irq, void *context);
#endif
#ifdef CONFIG_STM32_I2C2
static int stm32_i2c2_isr(int irq, void *context);
#endif
#ifdef CONFIG_STM32_I2C3
static int stm32_i2c3_isr(int irq, void *context);
#endif
#endif
static int stm32_i2c_init(FAR struct stm32_i2c_priv_s *priv);
static int stm32_i2c_deinit(FAR struct stm32_i2c_priv_s *priv);
static uint32_t stm32_i2c_setfrequency(FAR struct i2c_dev_s *dev,
                                       uint32_t frequency);
static int stm32_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int stm32_i2c_process(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs,
                             int count);
static int stm32_i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer,
                          int buflen);
static int stm32_i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen);
#ifdef CONFIG_I2C_WRITEREAD
static int stm32_i2c_writeread(FAR struct i2c_dev_s *dev,
                               const uint8_t *wbuffer, int wbuflen,
                               uint8_t *buffer, int buflen);
#endif
#ifdef CONFIG_I2C_TRANSFER
static int stm32_i2c_transfer(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs,
                              int count);
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_STM32_I2C1
static const struct stm32_i2c_config_s stm32_i2c1_config = 
{
  .base       = STM32_I2C1_BASE,
  .clk_bit    = RCC_APB1ENR_I2C1EN,
  .reset_bit  = RCC_APB1RSTR_I2C1RST,
  .scl_pin    = GPIO_I2C1_SCL,
  .sda_pin    = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .isr        = stm32_i2c1_isr,
  .ev_irq     = STM32_IRQ_I2C1EV,
  .er_irq     = STM32_IRQ_I2C1ER
#endif
};

struct stm32_i2c_priv_s stm32_i2c1_priv =
{
  .config     = &stm32_i2c1_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0    
};
#endif

#ifdef CONFIG_STM32_I2C2
static const struct stm32_i2c_config_s stm32_i2c2_config = 
{
  .base       = STM32_I2C2_BASE,
  .clk_bit    = RCC_APB1ENR_I2C2EN,
  .reset_bit  = RCC_APB1RSTR_I2C2RST,
  .scl_pin    = GPIO_I2C2_SCL,
  .sda_pin    = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_POLLED  
  .isr        = stm32_i2c2_isr,
  .ev_irq     = STM32_IRQ_I2C2EV,
  .er_irq     = STM32_IRQ_I2C2ER
#endif
};

struct stm32_i2c_priv_s stm32_i2c2_priv =
{
  .config     = &stm32_i2c2_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

#ifdef CONFIG_STM32_I2C3
static const struct stm32_i2c_config_s stm32_i2c3_config = 
{
  .base       = STM32_I2C3_BASE,
  .clk_bit    = RCC_APB1ENR_I2C3EN,
  .reset_bit  = RCC_APB1RSTR_I2C3RST,
  .scl_pin    = GPIO_I2C3_SCL,
  .sda_pin    = GPIO_I2C3_SDA,
#ifndef CONFIG_I2C_POLLED
  .isr        = stm32_i2c3_isr,
  .ev_irq     = STM32_IRQ_I2C3EV,
  .er_irq     = STM32_IRQ_I2C3ER
#endif
};

struct stm32_i2c_priv_s stm32_i2c3_priv =
{
  .config     = &stm32_i2c3_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0
};
#endif

/* Device Structures, Instantiation */

struct i2c_ops_s stm32_i2c_ops =
{
  .setfrequency       = stm32_i2c_setfrequency,
  .setaddress         = stm32_i2c_setaddress,
  .write              = stm32_i2c_write,
  .read               = stm32_i2c_read
#ifdef CONFIG_I2C_WRITEREAD
  , .writeread          = stm32_i2c_writeread
#endif
#ifdef CONFIG_I2C_TRANSFER
  , .transfer           = stm32_i2c_transfer
#endif
#ifdef CONFIG_I2C_SLAVE
  , .setownaddress      = stm32_i2c_setownaddress,
    .registercallback   = stm32_i2c_registercallback
#endif
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_i2c_getreg
 *
 * Description:
 *   Get register value by offset
 *
 ************************************************************************************/

static inline uint16_t stm32_i2c_getreg(FAR struct stm32_i2c_priv_s *priv,
                                        uint8_t offset)
{
  return getreg16(priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_i2c_putreg
 *
 * Description:
 *  Put register value by offset
 *
 ************************************************************************************/

static inline void stm32_i2c_putreg(FAR struct stm32_i2c_priv_s *priv, uint8_t offset,
                                    uint16_t value)
{
  putreg16(value, priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_i2c_modifyreg
 *
 * Description:
 *   Modify register value by offset
 *
 ************************************************************************************/

static inline void stm32_i2c_modifyreg(FAR struct stm32_i2c_priv_s *priv,
                                       uint8_t offset, uint16_t clearbits,
                                       uint16_t setbits)
{
  modifyreg16(priv->config->base + offset, clearbits, setbits);
}

/************************************************************************************
 * Name:
 *
 * Description:
 *
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_wait(FAR struct i2c_dev_s *dev)
{
  while (sem_wait(&((struct stm32_i2c_inst_s *)dev)->priv->sem_excl) != 0)
    {
      ASSERT(errno == EINTR);
    }
}

/************************************************************************************
 * Name: stm32_i2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be processed.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_I2C_DYNTIMEO
static useconds_t stm32_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor.
   */

  return (useconds_t)(CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/************************************************************************************
 * Name: stm32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int stm32_i2c_sem_waitdone(FAR struct stm32_i2c_priv_s *priv)
{
  struct timespec abstime;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  flags = irqsave();

  /* Enable I2C interrupts */

  regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  regval |= (I2C_CR2_ITERREN | I2C_CR2_ITEVFEN);
  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * sem_timedwait() sleeps.
   */

   priv->intstate = INTSTATE_WAITING;
   do
    {
      /* Get the current time */

      (void)clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_STM32_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_STM32_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
      abstime.tv_nsec += 1000 * stm32_i2c_tousecs(priv->msgc, priv->msgv);
      if (abstime.tv_nsec > 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_STM32_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_STM32_I2CTIMEOMS * 1000 * 1000;
      if (abstime.tv_nsec > 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }
#endif
      /* Wait until either the transfer is complete or the timeout expires */

      ret = sem_timedwait(&priv->sem_isr, &abstime);
      if (ret != OK && errno != EINTR)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by sem_timedwait.
           * NOTE that we try again if we are awakened by a signal (EINTR).
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  regval  = stm32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  regval &= ~I2C_CR2_ALLINTS;
  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);

  irqrestore(flags);
  return ret;
}
#else
static inline int stm32_i2c_sem_waitdone(FAR struct stm32_i2c_priv_s *priv)
{
  uint32_t timeout;
  uint32_t start;
  uint32_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
  timeout = USEC2TICK(stm32_i2c_tousecs(priv->msgc, priv->msgv));
#else
  timeout = CONFIG_STM32_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * sem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  start = clock_systimer();

  do
    {
      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      stm32_i2c_isr(priv);

      /* Calculate the elapsed time */

      elapsed = clock_systimer() - start;
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cvdbg("intstate: %d elapsed: %d threshold: %d status: %08x\n",
          priv->intstate, elapsed, timeout, priv->status);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/************************************************************************************
 * Name: stm32_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_waitstop(FAR struct stm32_i2c_priv_s *priv)
{
  uint32_t start;
  uint32_t elapsed;
  uint32_t timeout;
  uint32_t cr1;
  uint32_t sr1;

  /* Select a timeout */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_STM32_I2CTIMEOTICKS;
#endif

  /* Wait as stop might still be in progress; but stop might also
   * be set because of a timeout error: "The [STOP] bit is set and
   * cleared by software, cleared by hardware when a Stop condition is
   * detected, set by hardware when a timeout error is detected."
   */

  start = clock_systimer();
  do
    {
      /* Check for STOP condition */

      cr1 = stm32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET);
      if ((cr1 & I2C_CR1_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      sr1 = stm32_i2c_getreg(priv, STM32_I2C_SR1_OFFSET);
      if ((sr1 & I2C_SR1_TIMEOUT) != 0)
        {
          return;
        }      

      /* Calculate the elapsed time */

      elapsed = clock_systimer() - start;
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.
   */

  i2cvdbg("Timeout with CR1: %04x SR1: %04x\n", cr1, sr1);
}

/************************************************************************************
 * Name: stm32_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_post(FAR struct i2c_dev_s *dev)
{
  sem_post( &((struct stm32_i2c_inst_s *)dev)->priv->sem_excl );
}

/************************************************************************************
 * Name: stm32_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_init(FAR struct i2c_dev_s *dev)
{
  sem_init(&((struct stm32_i2c_inst_s *)dev)->priv->sem_excl, 0, 1);
#ifndef CONFIG_I2C_POLLED
  sem_init(&((struct stm32_i2c_inst_s *)dev)->priv->sem_isr, 0, 0);
#endif
}

/************************************************************************************
 * Name: stm32_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_destroy(FAR struct i2c_dev_s *dev)
{
  sem_destroy(&((struct stm32_i2c_inst_s *)dev)->priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  sem_destroy(&((struct stm32_i2c_inst_s *)dev)->priv->sem_isr);
#endif
}

/************************************************************************************
 * Name: stm32_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void stm32_i2c_traceclear(FAR struct stm32_i2c_priv_s *priv)
{
  struct stm32_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void stm32_i2c_tracereset(FAR struct stm32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systimer();
  stm32_i2c_traceclear(priv);
}

static void stm32_i2c_tracenew(FAR struct stm32_i2c_priv_s *priv, uint32_t status)
{
  struct stm32_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?   Has the status changed? */

  if (trace->count == 0 || status != trace->status)
    {
      /* Yes.. Was it the status changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index (unless we are out of trace entries) */

          if (priv->tndx >= (CONFIG_I2C_NTRACE-1))
            {
              i2cdbg("Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      stm32_i2c_traceclear(priv);
      trace->status = status;
      trace->count  = 1;
      trace->time   = clock_systimer();
    }
  else
    {
      /* Just increment the count of times that we have seen this status */

      trace->count++;
    }
}

static void stm32_i2c_traceevent(FAR struct stm32_i2c_priv_s *priv,
                                enum stm32_trace_e event, uint32_t parm)
{
  struct stm32_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event  = event;
      trace->parm   = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE-1))
        {
          i2cdbg("Trace table overflow\n");
          return;
        }

      priv->tndx++;
      stm32_i2c_traceclear(priv);
    }
}

static void stm32_i2c_tracedump(FAR struct stm32_i2c_priv_s *priv)
{
  struct stm32_trace_s *trace;
  int i;

  syslog("Elapsed time: %d\n",  clock_systimer() - priv->start_time);
  for (i = 0; i <= priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog("%2d. STATUS: %08x COUNT: %3d EVENT: %2d PARM: %08x TIME: %d\n",
                    i+1, trace->status, trace->count,  trace->event, trace->parm,
                    trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/************************************************************************************
 * Name: stm32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ************************************************************************************/

static void stm32_i2c_setclock(FAR struct stm32_i2c_priv_s *priv, uint32_t frequency)
{
  uint16_t cr1;
  uint16_t ccr;
  uint16_t trise;
  uint16_t freqmhz;
  uint16_t speed;

  /* Disable the selected I2C peripheral to configure TRISE */

  cr1 = stm32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET);
  stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, cr1 & ~I2C_CR1_PE);

  /* Update timing and control registers */

  freqmhz = (uint16_t)(STM32_PCLK1_FREQUENCY / 1000000);
  ccr = 0;

  /* Configure speed in standard mode */

  if (frequency <= 100000)
    {
      /* Standard mode speed calculation */

      speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency << 1));

      /* The CCR fault must be >= 4 */

      if (speed < 4)
        {
          /* Set the minimum allowed value */

          speed = 4;  
        }
      ccr |= speed;

      /* Set Maximum Rise Time for standard mode */

      trise = freqmhz + 1; 
    }

  /* Configure speed in fast mode */

  else /* (frequency <= 400000) */
    {
      /* Fast mode speed calculation with Tlow/Thigh = 16/9 */

#ifdef CONFIG_STM32_I2C_DUTY16_9
      speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency * 25));

      /* Set DUTY and fast speed bits */

      ccr |= (I2C_CCR_DUTY|I2C_CCR_FS);
#else
      /* Fast mode speed calculation with Tlow/Thigh = 2 */

      speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency * 3));

      /* Set fast speed bit */

      ccr |= I2C_CCR_FS;
#endif

      /* Verify that the CCR speed value is nonzero */

      if (speed < 1)
        {
          /* Set the minimum allowed value */

          speed = 1;  
        }
      ccr |= speed;

      /* Set Maximum Rise Time for fast mode */

      trise = (uint16_t)(((freqmhz * 300) / 1000) + 1);  
    }

  /* Write the new values of the CCR and TRISE registers */

  stm32_i2c_putreg(priv, STM32_I2C_CCR_OFFSET, ccr);
  stm32_i2c_putreg(priv, STM32_I2C_TRISE_OFFSET, trise);

  /* Bit 14 of OAR1 must be configured and kept at 1 */

  stm32_i2c_putreg(priv, STM32_I2C_OAR1_OFFSET, I2C_OAR1_ONE);

  /* Re-enable the peripheral (or not) */

  stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, cr1);
}

/************************************************************************************
 * Name: stm32_i2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ************************************************************************************/

static inline void stm32_i2c_sendstart(FAR struct stm32_i2c_priv_s *priv)
{
  /* Disable ACK on receive by default and generate START */

  stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, I2C_CR1_START);
}

/************************************************************************************
 * Name: stm32_i2c_clrstart
 *
 * Description:
 *   Clear the STOP, START or PEC condition on certain error recovery steps.
 *
 ************************************************************************************/

static inline void stm32_i2c_clrstart(FAR struct stm32_i2c_priv_s *priv)
{
  /* "Note: When the STOP, START or PEC bit is set, the software must
   *  not perform any write access to I2C_CR1 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PEC request."
   *
   * "The [STOP] bit is set and cleared by software, cleared by hardware
   *  when a Stop condition is detected, set by hardware when a timeout
   *  error is detected.
   * 
   * "This [START] bit is set and cleared by software and cleared by hardware
   *  when start is sent or PE=0."  The bit must be cleared by software if the
   *  START is never sent.
   *
   * "This [PEC] bit is set and cleared by software, and cleared by hardware
   *  when PEC is transferred or by a START or Stop condition or when PE=0."
   */

  stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                      I2C_CR1_START|I2C_CR1_STOP|I2C_CR1_PEC, 0);
}

/************************************************************************************
 * Name: stm32_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ************************************************************************************/

static inline void stm32_i2c_sendstop(FAR struct stm32_i2c_priv_s *priv)
{
  stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, I2C_CR1_STOP);
}

/************************************************************************************
 * Name: stm32_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status (SR1 and SR2 combined)
 *
 ************************************************************************************/

static inline uint32_t stm32_i2c_getstatus(FAR struct stm32_i2c_priv_s *priv)
{
  uint32_t status = stm32_i2c_getreg(priv, STM32_I2C_SR1_OFFSET);
  status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);
  return status;
}

/************************************************************************************
 * Name: stm32_i2c_disablefsmc
 *
 * Description:
 *   FSMC must be disable while accessing I2C1 because it uses a common resource
 *   (LBAR)
 *
 *  NOTE: This is an issue with the STM32F103ZE, but may not be an issue with other
 *  STM32s.  You may need to experiment
 *
 ************************************************************************************/

#ifdef I2C1_FSMC_CONFLICT
static inline uint32_t stm32_i2c_disablefsmc(FAR struct stm32_i2c_priv_s *priv)
{
  uint32_t ret = 0;
  uint32_t regval;

  /* Is this I2C1 */

#if defined(CONFIG_STM32_I2C2) || defined(CONFIG_STM32_I2C3)
  if (priv->config->base == STM32_I2C1_BASE)
#endif
    {
      /* Disable FSMC unconditionally */

      ret    = getreg32( STM32_RCC_AHBENR);
      regval = ret & ~RCC_AHBENR_FSMCEN;
      putreg32(regval, STM32_RCC_AHBENR);
    }
  return ret;
}

/************************************************************************************
 * Name: stm32_i2c_enablefsmc
 *
 * Description:
 *   Re-enabled the FSMC
 *
 ************************************************************************************/

static inline void stm32_i2c_enablefsmc(uint32_t ahbenr)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC only if it was previously enabled. */

  if ((ahbenr & RCC_AHBENR_FSMCEN) != 0)
    {
      regval  = getreg32( STM32_RCC_AHBENR);
      regval |= RCC_AHBENR_FSMCEN;
      putreg32(regval, STM32_RCC_AHBENR);
    }
}
#else
#  define stm32_i2c_disablefsmc(priv) (0)
#  define stm32_i2c_enablefsmc(ahbenr)
#endif /* I2C1_FSMC_CONFLICT */

/************************************************************************************
 * Name: stm32_i2c_isr
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ************************************************************************************/

static int stm32_i2c_isr(struct stm32_i2c_priv_s *priv)
{
  uint32_t status = stm32_i2c_getstatus(priv);

  /* Check for new trace setup */

  stm32_i2c_tracenew(priv, status);
            
  /* Was start bit sent */
    
  if ((status & I2C_SR1_SB) != 0)
    {
      stm32_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgc);

      /* Get run-time data */

      priv->ptr   = priv->msgv->buffer;
      priv->dcnt  = priv->msgv->length;
      priv->flags = priv->msgv->flags;

      /* Send address byte and define addressing mode */

      stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET,
                      (priv->flags & I2C_M_TEN) ?
                       0 : ((priv->msgv->addr << 1) | (priv->flags & I2C_M_READ)));

      /* Set ACK for receive mode */

      if (priv->dcnt > 1 && (priv->flags & I2C_M_READ) != 0)
        {
          stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, 0, I2C_CR1_ACK);
        }

      /* Increment to next pointer and decrement message count */

      priv->msgv++;
      priv->msgc--;
    }

 /* In 10-bit addressing mode, was first byte sent */
    
  else if ((status & I2C_SR1_ADD10) != 0)
    {
      /* TODO: Finish 10-bit mode addressing */
    }

  /* Was address sent, continue with either sending or reading data */

  else if ((priv->flags & I2C_M_READ) == 0 && (status & (I2C_SR1_ADDR | I2C_SR1_TXE)) != 0)
    {
      if (priv->dcnt > 0)
        {
          /* Send a byte */

          stm32_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
          stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, *priv->ptr++);
          priv->dcnt--;
        }
    }

  else if ((priv->flags & I2C_M_READ) != 0 && (status & I2C_SR1_ADDR) != 0)
    {
      /* Enable RxNE and TxE buffers in order to receive one or multiple bytes */

#ifndef CONFIG_I2C_POLLED
      stm32_i2c_traceevent(priv, I2CEVENT_ITBUFEN, 0);
      stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
#endif
    }

  /* More bytes to read */

  else if ((status & I2C_SR1_RXNE) != 0)
    {
      /* Read a byte, if dcnt goes < 0, then read dummy bytes to ack ISRs */
    
      if (priv->dcnt > 0)
        {
          stm32_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

          /* No interrupts or context switches may occur in the following
           * sequence.  Otherwise, additional bytes may be sent by the
           * device.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t state = irqsave();
#endif
          /* Receive a byte */

          *priv->ptr++ = stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);

          /* Disable acknowledge when last byte is to be received */

          priv->dcnt--;
          if (priv->dcnt == 1)
            {
              stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, 0);  
            }

#ifdef CONFIG_I2C_POLLED
          irqrestore(state);
#endif
        }
    }
    
  /* Do we have more bytes to send, enable/disable buffer interrupts
   * (these ISRs could be replaced by DMAs)
   */

#ifndef CONFIG_I2C_POLLED
  if (priv->dcnt > 0)
    {
      stm32_i2c_traceevent(priv, I2CEVENT_REITBUFEN, 0);
      stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
    }
  else if (priv->dcnt == 0)
    {
      stm32_i2c_traceevent(priv, I2CEVENT_DISITBUFEN, 0);
      stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);  
    }
#endif
    
  /* Was last byte received or sent?  Hmmm... the F2 and F4 seems to differ from 
   * the F1 in that BTF is not set after data is received (only RXNE).
   */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
  if (priv->dcnt <= 0 && (status & (I2C_SR1_BTF|I2C_SR1_RXNE)) != 0)
#else
  if (priv->dcnt <= 0 && (status & I2C_SR1_BTF) != 0)
#endif
    {
      stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);    /* ACK ISR */

      /* Do we need to terminate or restart after this byte?
       * If there are more messages to send, then we may:
       *
       *  - continue with repeated start
       *  - or just continue sending writeable part
       *  - or we close down by sending the stop bit 
       */

      if (priv->msgc > 0)
        {
          if (priv->msgv->flags & I2C_M_NORESTART)
            {
              stm32_i2c_traceevent(priv, I2CEVENT_BTFNOSTART, priv->msgc);
              priv->ptr   = priv->msgv->buffer;
              priv->dcnt  = priv->msgv->length;
              priv->flags = priv->msgv->flags;
              priv->msgv++;
              priv->msgc--;

              /* Restart this ISR! */

#ifndef CONFIG_I2C_POLLED
              stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
#endif
            }
          else
            {
              stm32_i2c_traceevent(priv, I2CEVENT_BTFRESTART, priv->msgc);
              stm32_i2c_sendstart(priv);
            }
        }
      else if (priv->msgv)
        {
          stm32_i2c_traceevent(priv, I2CEVENT_BTFSTOP, 0);
          stm32_i2c_sendstop(priv);

          /* Is there a thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
          if (priv->intstate == INTSTATE_WAITING)
            {
              /* Yes.. inform the thread that the transfer is complete
               * and wake it up.
               */

              sem_post( &priv->sem_isr );
              priv->intstate = INTSTATE_DONE;
            }
#else
          priv->intstate = INTSTATE_DONE;
#endif

          /* Mark that we have stopped with this transaction */

          priv->msgv = NULL;
        }
    }
    
    /* Check for errors, in which case, stop the transfer and return 
     * Note that in master reception mode AF becomes set on last byte
     * since ACK is not returned. We should ignore this error.
     */
    
    if ((status & I2C_SR1_ERRORMASK) != 0)
      {
        stm32_i2c_traceevent(priv, I2CEVENT_ERROR, 0);

        /* Clear interrupt flags */

        stm32_i2c_putreg(priv, STM32_I2C_SR1_OFFSET, 0);

        /* Is there a thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
        if (priv->intstate == INTSTATE_WAITING)
          {
              /* Yes.. inform the thread that the transfer is complete
               * and wake it up.
               */

            sem_post( &priv->sem_isr );
            priv->intstate = INTSTATE_DONE;
          }
#else
        priv->intstate = INTSTATE_DONE;
#endif
      }

    priv->status = status;
    return OK;
}

/************************************************************************************
 * Name: stm32_i2c1_isr
 *
 * Description:
 *   I2C1 interrupt service routine
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
#ifdef CONFIG_STM32_I2C1
static int stm32_i2c1_isr(int irq, void *context)
{
  return stm32_i2c_isr(&stm32_i2c1_priv);
}
#endif

/************************************************************************************
 * Name: stm32_i2c2_isr
 *
 * Description:
 *   I2C2 interrupt service routine
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_I2C2
static int stm32_i2c2_isr(int irq, void *context)
{
  return stm32_i2c_isr(&stm32_i2c2_priv);
}
#endif

/************************************************************************************
 * Name: stm32_i2c3_isr
 *
 * Description:
 *   I2C2 interrupt service routine
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_I2C3
static int stm32_i2c3_isr(int irq, void *context)
{
  return stm32_i2c_isr(&stm32_i2c3_priv);
}
#endif
#endif

/************************************************************************************
 * Private Initialization and Deinitialization
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ************************************************************************************/

static int stm32_i2c_init(FAR struct stm32_i2c_priv_s *priv)
{
  /* Power-up and configure GPIOs */

  /* Enable power and reset the peripheral */

  modifyreg32(STM32_RCC_APB1ENR, 0, priv->config->clk_bit);
  modifyreg32(STM32_RCC_APB1RSTR, 0, priv->config->reset_bit);
  modifyreg32(STM32_RCC_APB1RSTR, priv->config->reset_bit, 0);

  /* Configure pins */
      
  if (stm32_configgpio(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (stm32_configgpio(priv->config->sda_pin) < 0)
    {
      stm32_unconfiggpio(priv->config->scl_pin);
      return ERROR;
    }

  /* Attach ISRs */
            
#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->ev_irq, priv->config->isr);
  irq_attach(priv->config->er_irq, priv->config->isr);
  up_enable_irq(priv->config->ev_irq);
  up_enable_irq(priv->config->er_irq);
#endif

  /* Set peripheral frequency, where it must be at least 2 MHz  for 100 kHz
   * or 4 MHz for 400 kHz.  This also disables all I2C interrupts.
   */

  stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, (STM32_PCLK1_FREQUENCY / 1000000));
  stm32_i2c_setclock(priv, 100000);
        
  /* Enable I2C */
    
  stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_PE);
  return OK;
}

/************************************************************************************
 * Name: stm32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ************************************************************************************/

static int stm32_i2c_deinit(FAR struct stm32_i2c_priv_s *priv)
{
  /* Disable I2C */

  stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, 0);

  /* Unconfigure GPIO pins */

  stm32_unconfiggpio(priv->config->scl_pin);
  stm32_unconfiggpio(priv->config->sda_pin);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->ev_irq);
  up_disable_irq(priv->config->er_irq);
  irq_detach(priv->config->ev_irq);
  irq_detach(priv->config->er_irq);
#endif

  /* Disable clocking */

  modifyreg32(STM32_RCC_APB1ENR, priv->config->clk_bit, 0);
  return OK;
}

/************************************************************************************
 * Device Driver Operations
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_i2c_setfrequency
 *
 * Description:
 *   Set the I2C frequency
 *
 ************************************************************************************/

static uint32_t stm32_i2c_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency)
{
  stm32_i2c_sem_wait(dev);

#if STM32_PCLK1_FREQUENCY < 4000000
  ((struct stm32_i2c_inst_s *)dev)->frequency = 100000;
#else
  ((struct stm32_i2c_inst_s *)dev)->frequency = frequency;
#endif

  stm32_i2c_sem_post(dev);    
  return ((struct stm32_i2c_inst_s *)dev)->frequency;
}

/************************************************************************************
 * Name: stm32_i2c_setaddress
 *
 * Description:
 *   Set the I2C slave address
 *
 ************************************************************************************/

static int stm32_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
  stm32_i2c_sem_wait(dev);

  ((struct stm32_i2c_inst_s *)dev)->address = addr;
  ((struct stm32_i2c_inst_s *)dev)->flags   = (nbits == 10) ? I2C_M_TEN : 0;

    stm32_i2c_sem_post(dev);
    return OK;
}

/************************************************************************************
 * Name: stm32_i2c_process
 *
 * Description:
 *   Common I2C transfer logic
 *
 ************************************************************************************/

static int stm32_i2c_process(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs, int count)
{
  struct stm32_i2c_inst_s     *inst = (struct stm32_i2c_inst_s *)dev;
  FAR struct stm32_i2c_priv_s *priv = inst->priv;
  uint32_t    status = 0;
  uint32_t    ahbenr;
  int         errval = 0;

  ASSERT(count);

  /* Disable FSMC that shares a pin with I2C1 (LBAR) */

  ahbenr = stm32_i2c_disablefsmc(priv);

  /* Wait for any STOP in progress.  NOTE:  If we have to disable the FSMC
   * then we cannot do this at the top of the loop, unfortunately.  The STOP
   * will not complete normally if the FSMC is enabled.
   */

#ifndef I2C1_FSMC_CONFLICT
  stm32_i2c_sem_waitstop(priv);
#endif

  /* Clear any pending error interrupts */

  stm32_i2c_putreg(priv, STM32_I2C_SR1_OFFSET, 0);

  /* "Note: When the STOP, START or PEC bit is set, the software must
   *  not perform any write access to I2C_CR1 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PEC request."  However, if the bits are
   *  not cleared by hardware, then we will have to do that from hardware.
   */

  stm32_i2c_clrstart(priv);
    
  /* Old transfers are done */

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  stm32_i2c_tracereset(priv);
        
  /* Set I2C clock frequency (on change it toggles I2C_CR1_PE !) */

  stm32_i2c_setclock(priv, inst->frequency);

  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within stm32_i2c_waitdone().
   */

  priv->status = 0;
  stm32_i2c_sendstart(priv);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (stm32_i2c_sem_waitdone(priv) < 0)
    {
      status = stm32_i2c_getstatus(priv);
      errval = ETIMEDOUT;

      i2cdbg("Timed out: CR1: %04x status: %08x\n",
             stm32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET), status);

      /* "Note: When the STOP, START or PEC bit is set, the software must
       *  not perform any write access to I2C_CR1 before this bit is
       *  cleared by hardware. Otherwise there is a risk of setting a
       *  second STOP, START or PEC request."
       */

      stm32_i2c_clrstart(priv);

      /* Clear busy flag in case of timeout */

      status = priv->status & 0xffff;
    }
  else
    {
      /* clear SR2 (BUSY flag) as we've done successfully */

      status = priv->status & 0xffff;
    }

  /* Check for error status conditions */

  if ((status & I2C_SR1_ERRORMASK) != 0)
    {
      /* I2C_SR1_ERRORMASK is the 'OR' of the following individual bits: */

      if (status & I2C_SR1_BERR)
        {
          /* Bus Error */

          errval = EIO;
        }
      else if (status & I2C_SR1_ARLO)
        {
          /* Arbitration Lost (master mode) */

          errval = EAGAIN;
        }
      else if (status & I2C_SR1_AF)
        {
          /* Acknowledge Failure */

          errval = ENXIO;
        }
      else if (status & I2C_SR1_OVR)
        {
          /* Overrun/Underrun */

          errval = EIO;
        }
      else if (status & I2C_SR1_PECERR)
        {
          /* PEC Error in reception */

          errval = EPROTO;
        }
      else if (status & I2C_SR1_TIMEOUT)
        {
          /* Timeout or Tlow Error */

          errval = ETIME;
        }

      /* This is not an error and should never happen since SMBus is not enabled */

      else /* if (status & I2C_SR1_SMBALERT) */
        {
          /* SMBus alert is an optional signal with an interrupt line for devices
           * that want to trade their ability to master for a pin.
           */

          errval = EINTR;
        }
    }

  /* This is not an error, but should not happen.  The BUSY signal can hang,
   * however, if there are unhealthy devices on the bus that need to be reset.
   * NOTE:  We will only see this buy indication if stm32_i2c_sem_waitdone()
   * fails above;  Otherwise it is cleared.
   */

  else if ((status & (I2C_SR2_BUSY << 16)) != 0)
    {
      /* I2C Bus is for some reason busy */

      errval = EBUSY;
    }

  /* Dump the trace result */

  stm32_i2c_tracedump(priv);
        
  /* Wait for any STOP in progress.  NOTE:  If we have to disable the FSMC
   * then we cannot do this at the top of the loop, unfortunately.  The STOP
   * will not complete normally if the FSMC is enabled.
   */

#ifdef I2C1_FSMC_CONFLICT
  stm32_i2c_sem_waitstop(priv);
#endif

  /* Re-enable the FSMC */

  stm32_i2c_enablefsmc(ahbenr);
  stm32_i2c_sem_post(dev);

  return -errval;
}

/************************************************************************************
 * Name: stm32_i2c_write
 *
 * Description:
 *   Write I2C data
 *
 ************************************************************************************/

static int stm32_i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen)
{
  stm32_i2c_sem_wait(dev);   /* ensure that address or flags don't change meanwhile */

  struct i2c_msg_s msgv =
  {
    .addr   = ((struct stm32_i2c_inst_s *)dev)->address,
    .flags  = ((struct stm32_i2c_inst_s *)dev)->flags,
    .buffer = (uint8_t *)buffer,
    .length = buflen
  };

  return stm32_i2c_process(dev, &msgv, 1);
}

/************************************************************************************
 * Name: stm32_i2c_read
 *
 * Description:
 *   Read I2C data
 *
 ************************************************************************************/

int stm32_i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen)
{
  stm32_i2c_sem_wait(dev);   /* ensure that address or flags don't change meanwhile */

  struct i2c_msg_s msgv =
  {
    .addr   = ((struct stm32_i2c_inst_s *)dev)->address,
    .flags  = ((struct stm32_i2c_inst_s *)dev)->flags | I2C_M_READ,
    .buffer = buffer,
    .length = buflen
  };
    
  return stm32_i2c_process(dev, &msgv, 1);
}

/************************************************************************************
 * Name: stm32_i2c_writeread
 *
 * Description:
 *  Read then write I2C data
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_WRITEREAD
static int stm32_i2c_writeread(FAR struct i2c_dev_s *dev,
                               const uint8_t *wbuffer, int wbuflen,
                               uint8_t *buffer, int buflen)
{
  stm32_i2c_sem_wait(dev);   /* ensure that address or flags don't change meanwhile */

  struct i2c_msg_s msgv[2] =
  {
    {
      .addr   = ((struct stm32_i2c_inst_s *)dev)->address,
      .flags  = ((struct stm32_i2c_inst_s *)dev)->flags,
      .buffer = (uint8_t *)wbuffer,          /* this is really ugly, sorry const ... */
      .length = wbuflen
    },
    {
      .addr   = ((struct stm32_i2c_inst_s *)dev)->address,
      .flags  = ((struct stm32_i2c_inst_s *)dev)->flags | ((buflen>0) ? I2C_M_READ : I2C_M_NORESTART),
      .buffer = buffer,
      .length = (buflen>0) ? buflen : -buflen
    }
  };

  return stm32_i2c_process(dev, msgv, 2);
}
#endif

/************************************************************************************
 * Name: stm32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
static int stm32_i2c_transfer(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs,
                              int count)
{
  stm32_i2c_sem_wait(dev);   /* ensure that address or flags don't change meanwhile */
  return stm32_i2c_process(dev, msgs, count);
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ************************************************************************************/

FAR struct i2c_dev_s *up_i2cinitialize(int port)
{
  struct stm32_i2c_priv_s * priv = NULL;  /* private data of device with multiple instances */
  struct stm32_i2c_inst_s * inst = NULL;  /* device, single instance */
  int irqs;

#if STM32_PCLK1_FREQUENCY < 4000000
#   warning STM32_I2C_INIT: Peripheral clock must be at least 4 MHz to support 400 kHz operation.
#endif

#if STM32_PCLK1_FREQUENCY < 2000000
#   warning STM32_I2C_INIT: Peripheral clock must be at least 2 MHz to support 100 kHz operation.
    return NULL;
#endif
    
  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_STM32_I2C1
      case 1:
        priv = (struct stm32_i2c_priv_s *)&stm32_i2c1_priv;
        break;
#endif
#ifdef CONFIG_STM32_I2C2
      case 2:
        priv = (struct stm32_i2c_priv_s *)&stm32_i2c2_priv;
        break;
#endif
#ifdef CONFIG_STM32_I2C3
      case 3:
        priv = (struct stm32_i2c_priv_s *)&stm32_i2c3_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Allocate instance */
    
  if (!(inst = kmalloc( sizeof(struct stm32_i2c_inst_s))))
    {
      return NULL;
    }

  /* Initialize instance */

  inst->ops       = &stm32_i2c_ops;
  inst->priv      = priv;
  inst->frequency = 100000;
  inst->address   = 0;
  inst->flags     = 0;

  /* Init private data for the first time, increment refs count,
   * power-up hardware and configure GPIOs. 
   */
    
  irqs = irqsave();
    
  if ((volatile int)priv->refs++ == 0)
    {
      stm32_i2c_sem_init( (struct i2c_dev_s *)inst );
      stm32_i2c_init( priv );
    }
    
  irqrestore(irqs);
  return (struct i2c_dev_s *)inst;
}

/************************************************************************************
 * Name: up_i2cuninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ************************************************************************************/

int up_i2cuninitialize(FAR struct i2c_dev_s * dev)
{
  int irqs;
    
  ASSERT(dev);
    
  /* Decrement refs and check for underflow */
    
  if (((struct stm32_i2c_inst_s *)dev)->priv->refs == 0) 
    {
      return ERROR;
    }

  irqs = irqsave();

  if (--((struct stm32_i2c_inst_s *)dev)->priv->refs)
    {
      irqrestore(irqs);
      kfree(dev);
      return OK;
    }

  irqrestore(irqs);

  /* Disable power and other HW resource (GPIO's) */

  stm32_i2c_deinit( ((struct stm32_i2c_inst_s *)dev)->priv );

  /* Release unused resources */

  stm32_i2c_sem_destroy( (struct i2c_dev_s *)dev );

  kfree(dev);
  return OK;
}

/************************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
int up_i2creset(FAR struct i2c_dev_s * dev)
{
  struct stm32_i2c_priv_s * priv;
  unsigned int clock_count;
  unsigned int stretch_count;
  unit32_ scl_gpio;
  unit32_ sda_gpio;
  int ret = ERROR;

  ASSERT(dev);

  /* Get I2C private structure */

  priv = ((struct stm32_i2c_inst_s *)dev)->priv;

  /* Our caller must own a ref */

  ASSERT(priv->refs > 0);

  /* Lock out other clients */

  stm32_i2c_sem_wait(dev);

  /* De-init the port */

  stm32_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  /* Let SDA go high */

  stm32_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!stm32_gpioread(sda_gpio)) 
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        { 
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!stm32_gpioread(scl_gpio))
        { 
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            { 
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      stm32_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      stm32_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  stm32_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  stm32_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  stm32_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  stm32_gpiowrite(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */
 
  stm32_unconfiggpio(sda_gpio);
  stm32_unconfiggpio(scl_gpio);

  /* Re-init the port */

  stm32_i2c_init(priv);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  stm32_i2c_sem_post(dev);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

#endif /* CONFIG_STM32_I2C1 || CONFIG_STM32_I2C2 || CONFIG_STM32_I2C3 */
