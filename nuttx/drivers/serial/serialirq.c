/************************************************************************************
 * drivers/serial/serialirq.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <semaphore.h>
#include <debug.h>
#include <nuttx/serial/serial.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Private Variables
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: uart_xmitchars
 *
 * Description:
 *   This function is called from the UART interrupt handler when an interrupt
 *   is received indicating that there is more space in the transmit FIFO.  This
 *   function will send characters from the tail of the xmit buffer while the driver
 *   write() logic adds data to the head of the xmit buffer.
 *
 ************************************************************************************/

void uart_xmitchars(FAR uart_dev_t *dev)
{
  uint16_t nbytes = 0;

  /* Send while we still have data & room in the fifo */

  while (dev->xmit.head != dev->xmit.tail && uart_txready(dev))
    {
      /* Send the next byte */

      uart_send(dev, dev->xmit.buffer[dev->xmit.tail]);
      nbytes++;

      /* Increment the tail index */

      if (++(dev->xmit.tail) >= dev->xmit.size)
        {
          dev->xmit.tail = 0;
        }
    }

  /* When all of the characters have been sent from the buffer disable the TX
   * interrupt.
   */

  if (dev->xmit.head == dev->xmit.tail)
    {
      uart_disabletxint(dev);
    }

  /* If any bytes were removed from the buffer, inform any waiters there there is
   * space available.
   */

  if (nbytes)
    {
      uart_datasent(dev);
    }
}

/************************************************************************************
 * Name: uart_receivechars
 *
 * Description:
 *   This function is called from the UART interrupt handler when an interrupt
 *   is received indicating that are bytes available in the receive fifo.  This
 *   function will add chars to head of receive buffer.  Driver read() logic will
 *   take characters from the tail of the buffer.
 *
 ************************************************************************************/

void uart_recvchars(FAR uart_dev_t *dev)
{
  unsigned int status;
  int nexthead = dev->recv.head + 1;
  uint16_t nbytes = 0;

  if (nexthead >= dev->recv.size)
    {
      nexthead = 0;
    }

  /* Loop putting characters into the receive buffer until either there are no
   * further characters to available.
   */

  while (uart_rxavailable(dev))
    {
      char ch =  uart_receive(dev, &status);
      
      /* If the RX buffer becomes full, then the serial data is discarded.  This is
       * necessary because on most serial hardware, you must read the data in order
       * to clear the RX interrupt. An option on some hardware might be to simply
       * disable RX interrupts until the RX buffer becomes non-FULL.  However, that
       * would probably just cause the overrun to occur in hardware (unless it has
       * some large internal buffering).
       */

      if (nexthead != dev->recv.tail)
        {
          /* Add the character to the buffer */

          dev->recv.buffer[dev->recv.head] = ch;
          nbytes++;

          /* Increment the head index */

          dev->recv.head = nexthead;
          if (++nexthead >= dev->recv.size)
            {
               nexthead = 0;
            }
        }
    }

  /* If any bytes were added to the buffer, inform any waiters there there is new
   * incoming data available.
   */

  if (nbytes)
    {
      uart_datareceived(dev);
    }
}
