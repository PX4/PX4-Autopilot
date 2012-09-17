/****************************************************************************
 * drivers/sercomm/console.c
 * Driver for NuttX Console
 *
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 Stefan Richter <ichgeh@l--putt.de>
 *
 * This source code is derivated from Osmocom-BB project and was
 * relicensed as BSD with permission from original authors.
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
 **************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/serial/serial.h>

#include <errno.h>
#include <debug.h>
#include <string.h>

#include "uart.h"
#include <nuttx/sercomm/sercomm.h>

/* stubs to make serial driver happy */
void sercomm_recvchars(void *a) { }
void sercomm_xmitchars(void *a) { }

/* Stubs to make memory allocator happy */
void cons_puts(void *foo){}
void delay_ms(int ms){}

/************************************************************************************
 * Fileops Prototypes and Structures
 ************************************************************************************/

typedef FAR struct file		file_t;

static ssize_t sc_console_read(file_t *filep, FAR char *buffer, size_t buflen);
static ssize_t sc_console_write(file_t *filep, FAR const char *buffer, size_t buflen);
static int     sc_console_ioctl(file_t *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     sc_console_poll(file_t *filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations g_sercom_console_ops =
{
	0,			/* open, always opened */
	0,			/* close, stays open */
	sc_console_read,	/* read */
	sc_console_write,	/* write */
	0,			/* seek, not supported */
	sc_console_ioctl,	/* ioctl */
#ifndef CONFIG_DISABLE_POLL
	sc_console_poll		/* poll */
#endif
};

/****************************************************************************
 * Helper functions
 ****************************************************************************/
static FAR uart_dev_t *readdev = NULL;
static struct msgb *recvmsg = NULL;
static void recv_cb(uint8_t dlci, struct msgb *msg)
{
	sem_post(&readdev->recvsem);
	recvmsg = msg;
}

/****************************************************************************
 * Fileops
 ****************************************************************************/

/* XXX: recvmsg is overwritten when multiple msg arrive! */
static ssize_t sc_console_read(file_t *filep, FAR char *buffer, size_t buflen)
{
	size_t len;
	struct msgb *tmp;

	/* Wait until data is received */
	while(recvmsg == NULL) {
		sem_wait(&readdev->recvsem);
	}

	len = recvmsg->len > buflen ? buflen : recvmsg->len;
	memcpy(buffer, msgb_get(recvmsg, len), len);

	if(recvmsg->len == 0) {
		/* prevent inconsistent msg by first invalidating it, then free it */
		tmp = recvmsg;
		recvmsg = NULL;
		msgb_free(tmp);
	}

	return len;
}

/* XXX: redirect to old Osmocom-BB comm/sercomm_cons.c -> 2 buffers */
extern int sercomm_puts(const char *s);
static ssize_t sc_console_write(file_t *filep, FAR const char *buffer, size_t buflen)
{
	int i, cnt;
	char dstbuf[32];

	if (buflen >= 31)
		cnt = 31;
	else
		cnt = buflen;

        memcpy(dstbuf, buffer, cnt);
        dstbuf[cnt] = '\0';

	/* print part of our buffer */
	sercomm_puts(dstbuf);

	/* wait a little bit to get data transfered */
	up_mdelay(1);

	return cnt;
}

/* Forward ioctl to uart driver */
static int sc_console_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode *inode = filep->f_inode;
	FAR uart_dev_t   *dev   = inode->i_private;

	return dev->ops->ioctl(filep, cmd, arg);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Use sercomm on uart driver, register console driver */
int sercomm_register(FAR const char *path, FAR uart_dev_t *dev)
{
	/* XXX: initialize MODEMUART to be used for sercomm*/
	uart_init(SERCOMM_UART_NR, 1);
	uart_baudrate(SERCOMM_UART_NR, UART_115200);
	readdev = dev;
	sercomm_register_rx_cb(SC_DLCI_LOADER, &recv_cb);

	sem_init(&dev->xmit.sem, 0, 1);
	sem_init(&dev->recv.sem, 0, 1);
	sem_init(&dev->closesem, 0, 1);
	sem_init(&dev->xmitsem,  0, 0);
	sem_init(&dev->recvsem,  0, 0);
#ifndef CONFIG_DISABLE_POLL
	sem_init(&dev->pollsem,  0, 1);
#endif

	dbg("Registering %s\n", path);
	return register_driver(path, &g_sercom_console_ops, 0666, NULL);
}
