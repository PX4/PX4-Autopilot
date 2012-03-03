/****************************************************************************
 * drivers/net/slip.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: RFC 1055
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/net/net.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arch.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_SLIP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NOTE:  Slip requires UART hardware handshake.  If hardware handshake is
 * not available with your UART, then you might try the 'slattach' option
 * -L which enable "3-wire operation."  That allows operation without the
 * hardware handshake (but with the possibility of data overrun).
 */

/* Configuration ************************************************************/

#if UIP_LLH_LEN > 0
#  error "UIP_LLH_LEN must be set to zero"
#endif

#ifndef CONFIG_NET_NOINTS
#  warning "CONFIG_NET_NOINTS must be set"
#endif

#ifndef CONFIG_NET_MULTIBUFFER
#  warning "CONFIG_NET_MULTIBUFFER must be set"
#endif

#ifndef CONFIG_SLIP_STACKSIZE
#  define CONFIG_SLIP_STACKSIZE 2048
#endif

#ifndef CONFIG_SLIP_DEFPRIO
#  define CONFIG_SLIP_DEFPRIO 128
#endif

/* The Linux slip module hard-codes its MTU size to 296 (40 bytes for the
 * IP+TPC headers plus 256 bytes of data).  So you might as well set
 * CONFIG_NET_BUFSIZE to 296 as well.
 *
 * There may be an issue with this setting, however.  I see that Linux uses
 * a MTU of 296 and window of 256, but actually only sends 168 bytes of data:
 * 40 + 128.  I believe that is to allow for the 2x worst cast packet
 * expansion.  Ideally we would like to advertise the 256 MSS, but restrict
 * uIP to 128 bytes (possibly by modifying the uip_mss() macro).
 */

#if CONFIG_NET_BUFSIZE < 296
#  error "CONFIG_NET_BUFSIZE >= 296 is required"
#elif CONFIG_NET_BUFSIZE > 296
#  warning "CONFIG_NET_BUFSIZE == 296 is optimal"
#endif

/* CONFIG_SLIP_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_SLIP_NINTERFACES
# define CONFIG_SLIP_NINTERFACES 1
#endif

/*  SLIP special character codes *******************************************/

#define SLIP_END      0300    /* Indicates end of packet */
#define SLIP_ESC      0333    /* Indicates byte stuffing */
#define SLIP_ESC_END  0334    /* ESC ESC_END means SLIP_END data byte */
#define SLIP_ESC_ESC  0335    /* ESC ESC_ESC means ESC data byte */

/* General driver definitions **********************************************/

/* TX poll delay = 1 second = 1000000 microseconds. */

#define SLIP_WDDELAY   (1*1000000)
#define SLIP_POLLHSEC  (1*2)

/* Statistics helper */

#ifdef CONFIG_NET_STATISTICS
#  define SLIP_STAT(p,f) (p->stats.f)++
#else
#  define SLIP_STAT(p,f)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Driver statistics */

#ifdef CONFIG_NET_STATISTICS
struct slip_statistics_s
{
  uint32_t transmitted;     /* Number of packets transmitted */
  uint32_t received         /* Number of packets received */
};
#endif

/* The slip_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct slip_driver_s
{
  volatile bool bifup;      /* true:ifup false:ifdown */
  int           fd;         /* TTY file descriptor */
  pid_t         rxpid;      /* Receiver thread ID */
  pid_t         txpid;      /* Transmitter thread ID */
  sem_t         waitsem;    /* Mutually exclusive access to uIP */
  uint16_t      rxlen;      /* The number of bytes in rxbuf */

  /* Driver statistics */

#ifdef CONFIG_NET_STATISTICS
  struct slip_statistics_s stats;
#endif

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s dev;  /* Interface understood by uIP */
  uint8_t rxbuf[CONFIG_NET_BUFSIZE + 2];
  uint8_t txbuf[CONFIG_NET_BUFSIZE + 2];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

 /* We really should get rid of CONFIG_SLIP_NINTERFACES and, instead,
  * kmalloc() new interface instances as needed.
  */

static struct slip_driver_s g_slip[CONFIG_SLIP_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void slip_semtake(FAR struct slip_driver_s *priv);

/* Common TX logic */

static void slip_write(FAR struct slip_driver_s *priv, const uint8_t *buffer, int len);
static void slip_putc(FAR struct slip_driver_s *priv, int ch);
static int slip_transmit(FAR struct slip_driver_s *priv);
static int slip_uiptxpoll(struct uip_driver_s *dev);
static void slip_txtask(int argc, char *argv[]);

/* Packet receiver task */

static int slip_getc(FAR struct slip_driver_s *priv);
static inline void slip_receive(FAR struct slip_driver_s *priv);
static int slip_rxtask(int argc, char *argv[]);

/* NuttX callback functions */

static int slip_ifup(struct uip_driver_s *dev);
static int slip_ifdown(struct uip_driver_s *dev);
static int slip_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int slip_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int slip_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slip_semtake
 ****************************************************************************/

static void slip_semtake(FAR struct slip_driver_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->waitsem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

#define slip_semgive(p) sem_post(&(p)->waitsem);

/****************************************************************************
 * Function: slip_write
 *
 * Description:
 *   Just an inline wrapper around fwrite with error checking.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *   buffer - Buffer data to send
 *   len - Buffer length in bytes
 *
 ****************************************************************************/

static inline void slip_write(FAR struct slip_driver_s *priv,
                              const uint8_t *buffer, int len)
{
  /* Handle the case where the write is awakened by a signal */

  while (write(priv->fd, buffer, len) < 0)
    {
      DEBUGASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Function: slip_putc
 *
 * Description:
 *   Just an inline wrapper around putc with error checking.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *   ch - The character to send
 *
 ****************************************************************************/

static inline void slip_putc(FAR struct slip_driver_s *priv, int ch)
{
  uint8_t buffer = (uint8_t)ch;
  slip_write(priv, &buffer, 1);
}

/****************************************************************************
 * Function: slip_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int slip_transmit(FAR struct slip_driver_s *priv)
{
  uint8_t *src;
  uint8_t *start;
  uint8_t  esc;
  int      remaining;
  int      len;

  /* Increment statistics */

  nvdbg("Sending packet size %d\n", priv->dev.d_len);
  SLIP_STAT(priv, transmitted);

  /* Send an initial END character to flush out any data that may have
   * accumulated in the receiver due to line noise
   */

  slip_putc(priv, SLIP_END);

  /* For each byte in the packet, send the appropriate character sequence */

  src       = priv->dev.d_buf;
  remaining = priv->dev.d_len;
  start     = src;
  len       = 0;
  
  while (remaining-- > 0)
    {
      switch (*src)
        {
          /* If it's the same code as an END character, we send a special two
           * character code so as not to make the receiver think we sent an
           * END
           */

          case SLIP_END:
            esc = SLIP_ESC_END;
            goto escape;

          /* If it's the same code as an ESC character, we send a special two
           * character code so as not to make the receiver think we sent an
           * ESC
           */

          case SLIP_ESC:
            esc = SLIP_ESC_ESC;

          escape:
            {
              /* Flush any unsent data */

              if (len > 0)
                {
                  slip_write(priv, start, len);

                  /* Reset */

                  start = src + 1;
                  len   = 0;
                }

              /* Then send the escape sequence */

              slip_putc(priv, SLIP_ESC);
              slip_putc(priv, esc);
            }
          break;

          /* otherwise, just bump up the count */

          default:
            len++;
            break;
        }

      /* Point to the next character in the packet */

      src++;
    }

  /* We have looked at every character in the packet.  Now flush any unsent
   * data
   */

  if (len > 0)
    {
      slip_write(priv, start, len);
    }

  /* And send the END token */

  slip_putc(priv, SLIP_END);
  return OK;
}

/****************************************************************************
 * Function: slip_uiptxpoll
 *
 * Description:
 *   Check if uIP has any outgoing packets ready to send.  This is a
 *   callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete, or
 *   2. When the preceding TX packet send times o ]ut and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The initiator of the poll holds the priv->waitsem;
 *
 ****************************************************************************/

static int slip_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      slip_transmit(priv);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: slip_txtask
 *
 * Description:
 *   Polling and transmission is performed on tx thread.
 *
 * Parameters:
 *   arg  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void slip_txtask(int argc, char *argv[])
{
  FAR struct slip_driver_s *priv;
  unsigned int index = *(argv[1]) - '0';
  uip_lock_t flags;

  ndbg("index: %d\n", index);
  DEBUGASSERT(index < CONFIG_SLIP_NINTERFACES);

  /* Get our private data structure instance and wake up the waiting
   * initialization logic.
   */

  priv = &g_slip[index];
  slip_semgive(priv);

  /* Loop forever */

  for (;;)
    {
      /* Wait for the timeout to expire (or until we are signaled by by  */

      usleep(SLIP_WDDELAY);

      /* Is the interface up? */

      if (priv->bifup)
        {
          /* Get exclusive access to uIP (if it it is already being used
           * slip_rxtask, then we have to wait).
           */

          slip_semtake(priv);

          /* Poll uIP for new XMIT data. BUG:  We really need to calculate
           * the number of hsecs!  When we are awakened by slip_txavail, the
           * number will be smaller; when we have to wait for the semaphore
           * (above), it may be larger.
           */

          flags = uip_lock();
          priv->dev.d_buf = priv->txbuf;
          (void)uip_timer(&priv->dev, slip_uiptxpoll, SLIP_POLLHSEC);
          uip_unlock(flags);
          slip_semgive(priv);
        }
    }
}

/****************************************************************************
 * Function: slip_getc
 *
 * Description:
 *   Get one byte from the serial input.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   The returned byte
 *
 ****************************************************************************/

static inline int slip_getc(FAR struct slip_driver_s *priv)
{
  uint8_t ch;

  while (read(priv->fd, &ch, 1) < 0)
    {
      DEBUGASSERT(errno == EINTR);
    }

  return (int)ch;
}

/****************************************************************************
 * Function: slip_receive
 *
 * Description:
 *   Read a packet from the serial input
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void slip_receive(FAR struct slip_driver_s *priv)
{
  uint8_t ch;

  /* Copy the data data from the hardware to to the RX buffer until we
   * put together a whole packet. Make sure not to copy them into the
   * packet if we run out of room.
   */

  nvdbg("Receiving packet\n");
  for (;;)
    {
      /* Get the next character in the stream. */

      ch = slip_getc(priv);

      /* Handle bytestuffing if necessary */
 
      switch (ch)
        {
        /* If it's an END character then we're done with the packet.
         * (OR we are just starting a packet)
         */

        case SLIP_END:
          nvdbg("END\n");

          /* A minor optimization: if there is no data in the packet, ignore
           * it. This is meant to avoid bothering IP with all the empty
           * packets generated by the duplicate END characters which are in
           * turn sent to try to detect line noise.
           */

          if (priv->rxlen > 0)
            {
              nvdbg("Received packet size %d\n", priv->rxlen);
              return;
            }
          break;

        /* if it's the same code as an ESC character, wait and get another
         * character and then figure out what to store in the packet based
         * on that.
         */

        case SLIP_ESC:
          nvdbg("ESC\n");
          ch = slip_getc(priv);

          /* if "ch" is not one of these two, then we have a protocol
           * violation.  The best bet seems to be to leave the byte alone
           * and just stuff it into the packet
           */

          switch (ch)
            {
            case SLIP_ESC_END:
              nvdbg("ESC-END\n");
              ch = SLIP_END;
              break;
             case SLIP_ESC_ESC:
              nvdbg("ESC-ESC\n");
              ch = SLIP_ESC;
              break;
            default:
              ndbg("ERROR: Protocol violation: %02x\n", ch);
              break;
            }

          /* Here we fall into the default handler and let it store the
           * character for us
           */

        default:
          if (priv->rxlen < CONFIG_NET_BUFSIZE+2)
            {
              priv->rxbuf[priv->rxlen++] = ch;
            }
          break;
        }
    }
}

/****************************************************************************
 * Function: slip_rxtask
 *
 * Description:
 *   Wait for incoming data.
 *
 * Parameters:
 *   argc
 *   argv
 *
 * Returned Value:
 *   (Does not return)
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_rxtask(int argc, char *argv[])
{
  FAR struct slip_driver_s *priv;
  unsigned int index = *(argv[1]) - '0';
  uip_lock_t flags;
  int ch;

  ndbg("index: %d\n", index);
  DEBUGASSERT(index < CONFIG_SLIP_NINTERFACES);

  /* Get our private data structure instance and wake up the waiting
   * initialization logic.
   */

  priv = &g_slip[index];
  slip_semgive(priv);

  /* Loop forever */

  for (;;)
    {
      /* Wait for the next character to be available on the input stream. */

      nvdbg("Waiting...\n");
      ch = slip_getc(priv);

      /* Ignore any input that we receive before the interface is up. */

      if (!priv->bifup)
        {
          continue;
        }

      /* We have something...
       *
       * END characters may appear at packet boundaries BEFORE as well as
       * after the beginning of the packet.  This is normal and expected.
       */

      if (ch == SLIP_END)
        {
          priv->rxlen = 0;
        }

      /* Otherwise, we are in danger of being out-of-sync.  Apparently the
       * leading END character is optional.  Let's try to continue.
       */

      else
        {
          priv->rxbuf[0] = (uint8_t)ch;
          priv->rxlen    = 1;
        }

      /* Copy the data data from the hardware to priv->rxbuf until we put
       * together a whole packet.
       */

      slip_receive(priv);
      SLIP_STAT(priv, received);

      /* All packets are assumed to be IP packets (we don't have a choice..
       * there is no Ethernet header containing the EtherType).  So pass the
       * received packet on for IP processing -- but only if it is big
       * enough to hold an IP header.
       */

      if (priv->rxlen >= UIP_IPH_LEN)
        {
          /* Handle the IP input.  Get exclusive access to uIP. */

          slip_semtake(priv);
          priv->dev.d_buf = priv->rxbuf;
          priv->dev.d_len = priv->rxlen;

          flags = uip_lock();
          uip_input(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.  NOTE that we are transmitting using the RX buffer!
           */

          if (priv->dev.d_len > 0)
            {
              slip_transmit(priv); 
            }
          uip_unlock(flags);
          slip_semgive(priv);
        }
      else
        {
          SLIP_STAT(priv, rxsmallpacket);
        }
    }

  /* We won't get here */

  return OK;
}

/****************************************************************************
 * Function: slip_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided 
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_ifup(struct uip_driver_s *dev)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Mark the interface up */

  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Function: slip_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_ifdown(struct uip_driver_s *dev)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  /* Mark the device "down" */

  priv->bifup = false;
  return OK;
}

/****************************************************************************
 * Function: slip_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a 
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int slip_txavail(struct uip_driver_s *dev)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Wake up the TX polling thread */

      kill(priv->txpid, SIGALRM);
    }

  return OK;
}

/****************************************************************************
 * Function: slip_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int slip_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: slip_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int slip_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: slip_initialize
 *
 * Description:
 *   Instantiate a SLIP network interface.
 *
 * Parameters:
 *   intf - In the case where there are multiple SLIP interfaces, this value
 *          identifies which is to be initialized.  The network name will be,
 *          for example, "/dev/slip5" for intf == 5
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int slip_initialize(int intf, const char *devname)
{
  struct slip_driver_s *priv;
  char buffer[8];
  const char *argv[2];

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_SLIP_NINTERFACES);
  priv = &g_slip[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct slip_driver_s));
  priv->dev.d_ifup    = slip_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = slip_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = slip_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = slip_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = slip_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = priv;          /* Used to recover private state from dev */

  /* Open the device */

  priv->fd            = open(devname, O_RDWR, 0666);
  if (priv->fd < 0)
    {
      ndbg("ERROR: Failed to open %s: %d\n", devname, errno);
      return -errno;
    }

  /* Initialize the wait semaphore */

  sem_init(&priv->waitsem, 0, 0);

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling slip_ifdown().
   */

  slip_ifdown(&priv->dev);

  /* Start the SLIP receiver task */

  snprintf(buffer, 8, "%d", intf);
  argv[0] = buffer;
  argv[1] = NULL;

#ifndef CONFIG_CUSTOM_STACK
  priv->rxpid = task_create("rxslip", CONFIG_SLIP_DEFPRIO,
                          CONFIG_SLIP_STACKSIZE, (main_t)slip_rxtask, argv);
#else
  priv->rxpid = task_create("rxslip", CONFIG_SLIP_DEFPRIO,
                          (main_t)slip_rxtask, argv);
#endif
  if (priv->rxpid < 0)
    {
      ndbg("ERROR: Failed to start receiver task\n");
      return -errno;
    }

  /* Wait and make sure that the receive task is started. */

  slip_semtake(priv);

  /* Start the SLIP transmitter task */

#ifndef CONFIG_CUSTOM_STACK
  priv->txpid = task_create("txslip", CONFIG_SLIP_DEFPRIO,
                          CONFIG_SLIP_STACKSIZE, (main_t)slip_txtask, argv);
#else
  priv->txpid = task_create("txslip", CONFIG_SLIP_DEFPRIO,
                          (main_t)slip_txtask, argv);
#endif
  if (priv->txpid < 0)
    {
      ndbg("ERROR: Failed to start receiver task\n");
      return -errno;
    }

  /* Wait and make sure that the transmit task is started. */

  slip_semtake(priv);

  /* Bump the semaphore count so that it can now be used as a mutex */

  slip_semgive(priv);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->dev);

  /* When the RX and TX tasks were created, the TTY file descriptor was
   * dup'ed for each task.  This task no longer needs the file descriptor
   * and we can safely close it.
   */

  close(priv->fd);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_SLIP */

