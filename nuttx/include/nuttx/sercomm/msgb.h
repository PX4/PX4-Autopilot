#ifndef _MSGB_H
#define _MSGB_H

/* (C) 2008-2010 by Harald Welte <laforge@gnumonks.org>
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

#include <osmocom/core/linuxlist.h>
#include <console.h>

struct msgb {
	struct llist_head list;

	/* the layer 1 header, if any */
	unsigned char *l1h;
	/* the A-bis layer 2 header: OML, RSL(RLL), NS */
	unsigned char *l2h;
	/* the layer 3 header. For OML: FOM; RSL: 04.08; GPRS: BSSGP */
	unsigned char *l3h;

	uint16_t data_len;
	uint16_t len;

	unsigned char *head;	/* start of buffer */
	unsigned char *tail;	/* end of message */
	unsigned char *data;	/* start of message */
	unsigned char _data[0];
};

extern struct msgb *msgb_alloc(uint16_t size, const char *name);
extern void msgb_free(struct msgb *m);
extern void msgb_enqueue(struct llist_head *queue, struct msgb *msg);
extern struct msgb *msgb_dequeue(struct llist_head *queue);
extern void msgb_reset(struct msgb *m);

#define msgb_l1(m)	((void *)(m->l1h))
#define msgb_l2(m)	((void *)(m->l2h))
#define msgb_l3(m)	((void *)(m->l3h))

static inline unsigned int msgb_l1len(const struct msgb *msgb)
{
	return msgb->tail - (uint8_t *)msgb_l1(msgb);
}

static inline unsigned int msgb_l2len(const struct msgb *msgb)
{
	return msgb->tail - (uint8_t *)msgb_l2(msgb);
}

static inline unsigned int msgb_l3len(const struct msgb *msgb)
{
	return msgb->tail - (uint8_t *)msgb_l3(msgb);
}

static inline unsigned int msgb_headlen(const struct msgb *msgb)
{
	return msgb->len - msgb->data_len;
}
static inline int msgb_tailroom(const struct msgb *msgb)
{
	return (msgb->head + msgb->data_len) - msgb->tail;
}
static inline unsigned char *msgb_put(struct msgb *msgb, unsigned int len)
{
	unsigned char *tmp = msgb->tail;

	/* we intentionally call cons_puts() here to display an allocation
	 * failure on the _other_ serial port (i.e. the one that doesn't
	 * have the HDLC layer on it */
	if (msgb_tailroom(msgb) < len)
		cons_puts("msgb_tailroom insufficient!\n");

	msgb->tail += len;
	msgb->len += len;
	return tmp;
}
static inline void msgb_put_u8(struct msgb *msgb, uint8_t word)
{
	uint8_t *space = msgb_put(msgb, 1);
	space[0] = word & 0xFF;
}
static inline void msgb_put_u16(struct msgb *msgb, uint16_t word)
{
	uint8_t *space = msgb_put(msgb, 2);
	space[0] = word >> 8 & 0xFF;
	space[1] = word & 0xFF;
}
static inline void msgb_put_u32(struct msgb *msgb, uint32_t word)
{
	uint8_t *space = msgb_put(msgb, 4);
	space[0] = word >> 24 & 0xFF;
	space[1] = word >> 16 & 0xFF;
	space[2] = word >> 8 & 0xFF;
	space[3] = word & 0xFF;
}
static inline unsigned char *msgb_get(struct msgb *msgb, unsigned int len)
{
	unsigned char *tmp = msgb->data;
	msgb->data += len;
	msgb->len -= len;
	return tmp;
}
static inline uint8_t msgb_get_u8(struct msgb *msgb)
{
	uint8_t *space = msgb_get(msgb, 1);
	return space[0];
}
static inline uint16_t msgb_get_u16(struct msgb *msgb)
{
	uint8_t *space = msgb_get(msgb, 2);
	return space[0] << 8 | space[1];
}
static inline uint32_t msgb_get_u32(struct msgb *msgb)
{
	uint8_t *space = msgb_get(msgb, 4);
	return space[0] << 24 | space[1] << 16 | space[2] << 8 | space[3];
}
static inline unsigned char *msgb_push(struct msgb *msgb, unsigned int len)
{
	msgb->data -= len;
	msgb->len += len;
	return msgb->data;
}
static inline unsigned char *msgb_pull(struct msgb *msgb, unsigned int len)
{
	msgb->len -= len;
	return msgb->data += len;
}

/* increase the headroom of an empty msgb, reducing the tailroom */
static inline void msgb_reserve(struct msgb *msg, int len)
{
	msg->data += len;
	msg->tail += len;
}

static inline struct msgb *msgb_alloc_headroom(int size, int headroom,
						const char *name)
{
	struct msgb *msg = msgb_alloc(size, name);
	if (msg)
		msgb_reserve(msg, headroom);
	return msg;
}

#endif /* _MSGB_H */
