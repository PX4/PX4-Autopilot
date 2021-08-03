/************************************************************************
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
 * Modified for use in Linux by Mark Charlebois
 *
 ************************************************************************/

// FIXME - need px4_queue
#include <queue.h>
#include <stddef.h>

void sq_rem(sq_entry_t *node, sq_queue_t *queue)
{
	if (queue->head && node) {
		if (node == queue->head) {
			queue->head = node->flink;

			if (node == queue->tail) {
				queue->tail = NULL;
			}

		} else {
			sq_entry_t *prev;

			for (prev = (sq_entry_t *)queue->head;
			     prev && prev->flink != node;
			     prev = prev->flink) {}

			if (prev) {
				sq_remafter(prev, queue);
			}
		}
	}
}

sq_entry_t *sq_remafter(sq_entry_t *node, sq_queue_t *queue)
{
	sq_entry_t *ret = node->flink;

	if (queue->head && ret) {
		if (queue->tail == ret) {
			queue->tail = node;
			node->flink = NULL;

		} else {
			node->flink = ret->flink;
		}

		ret->flink = NULL;
	}

	return ret;
}

void sq_addfirst(sq_entry_t *node, sq_queue_t *queue)
{
	node->flink = queue->head;

	if (!queue->head) {
		queue->tail = node;
	}

	queue->head = node;
}


