/****************************************************************************
 * arch/rgmp/src/bridge.c
 *
 *   Copyright (C) 2011 Yu Qiang. All rights reserved.
 *   Author: Yu Qiang <yuq825@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdbool.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <fs_internal.h>
#include <queue.h>
#include <arch/irq.h>
#include <rgmp/rtos.h>
#include <rgmp/bridge.h>
#include <rgmp/string.h>
#include <rgmp/stdio.h>

struct bridge {
	struct rgmp_bridge *b;
	sem_t rd_lock;
	sem_t wr_lock;
};

static ssize_t up_bridge_read(struct file *filp, char *buffer, size_t len)
{
	ssize_t ret;
    struct bridge *b = filp->f_inode->i_private;

	sem_wait(&b->rd_lock);
    ret = rgmp_bridge_read(b->b, buffer, len, 0);
	sem_post(&b->rd_lock);
	return ret;
}

static ssize_t up_bridge_write(struct file *filp, const char *buffer, size_t len)
{
	ssize_t ret;
    struct bridge *b = filp->f_inode->i_private;

	sem_wait(&b->wr_lock);
    ret = rgmp_bridge_write(b->b, (char *)buffer, len, 0);
	sem_post(&b->wr_lock);
	return ret;
}

static int up_bridge_open(struct file *filp)
{
    return 0;
}

static int up_bridge_close(struct file *filp)
{
    return 0;
}

static const struct file_operations up_bridge_fops = {
    .read = up_bridge_read,
    .write = up_bridge_write,
    .open = up_bridge_open,
    .close = up_bridge_close,
};

int rtos_bridge_init(struct rgmp_bridge *b)
{
    int err;
	struct bridge *bridge;
    char path[30] = {'/', 'd', 'e', 'v', '/'};

	if ((bridge = kmalloc(sizeof(*bridge))) == NULL)
		goto err0;

	bridge->b = b;
	if ((err = sem_init(&bridge->rd_lock, 0, 1)) == ERROR)
		goto err1;
	if ((err = sem_init(&bridge->wr_lock, 0, 1)) == ERROR)
		goto err1;

	// make rgmp_bridge0 to be the console
	if (strcmp(b->vdev->name, "rgmp_bridge0") == 0)
	    strlcpy(path + 5, "console", 25);
	else
	    strlcpy(path + 5, b->vdev->name, 25);

	if ((err = register_driver(path, &up_bridge_fops, 0666, bridge)) == ERROR) {
	    cprintf("NuttX: register bridge %s fail\n", b->vdev->name);
		goto err1;
	}

	return 0;

err1:
	kfree(bridge);
err0:
	return -1;
}


