/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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


#include "syslink_main.h"

#include "drv_deck.h"


SyslinkMemory::SyslinkMemory(Syslink *link) :
	CDev("SyslinkMemory", DECK_DEVICE_PATH),
	_link(link),
	_activeI(0)
{


}

SyslinkMemory::~SyslinkMemory()
{

}


int
SyslinkMemory::init()
{
	int ret = CDev::init();

	/* if init failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		return ret;
	}


	return ret;
}

ssize_t
SyslinkMemory::read(struct file *filp, char *buffer, size_t buflen)
{
	return read(_activeI, 0, buffer, buflen);
}

ssize_t
SyslinkMemory::write(struct file *filp, const char *buffer, size_t buflen)
{
	// For now, unsupported
	return -1;
//	return buflen;
}

int
SyslinkMemory::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DECKIOGNUM:
		*((int *) arg) = scan();
		return 0;

	case DECKIOSNUM:
		_activeI = *((int *) arg);
		return 0;

	case DECKIOID: {
			syslink_ow_getinfo_t *data = (syslink_ow_getinfo_t *) &msgbuf.data;
			getinfo(_activeI);
			*((uint8_t **)arg) = data->id;
			return 8;
		}

	default:
		CDev::ioctl(filp, cmd, arg);
		return 0;
	}
}


uint8_t
SyslinkMemory::scan()
{
	syslink_ow_scan_t *data = (syslink_ow_scan_t *) &msgbuf.data;
	msgbuf.type = SYSLINK_OW_SCAN;
	msgbuf.length = 0;
	sendAndWait();

	return data->nmems;
}

void
SyslinkMemory::getinfo(int i)
{
	syslink_ow_getinfo_t *data = (syslink_ow_getinfo_t *) &msgbuf.data;
	msgbuf.type = SYSLINK_OW_GETINFO;
	msgbuf.length = 1;
	data->idx = i;
	sendAndWait();
}

int
SyslinkMemory::read(int i, uint16_t addr, char *buf, int length)
{
	syslink_ow_read_t *data = (syslink_ow_read_t *) &msgbuf.data;
	msgbuf.type = SYSLINK_OW_READ;

	int nread = 0;

	while (nread < length) {

		msgbuf.length = 3;
		data->idx = i;
		data->addr = addr;
		sendAndWait();

		// Number of bytes actually read
		int n = MIN(length - nread, msgbuf.length - 3);

		if (n == 0) {
			break;
		}

		memcpy(buf, data->data, n);
		nread += n;
		buf += n;
		addr += n;
	}

	return nread;
}

int
SyslinkMemory::write(int i, uint16_t addr, const char *buf, int length)
{
	// TODO: Unimplemented
	return -1;
}

void
SyslinkMemory::sendAndWait()
{
	// TODO: Force the syslink thread to wake up
	_link->_queue.force(&msgbuf, sizeof(msgbuf));
	px4_sem_wait(&_link->memory_sem);
}
