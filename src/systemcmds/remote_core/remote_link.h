/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* rptun control and the "rpmsg-hello" endpoint towards the remote core.
 * C because the OpenAMP and rptun headers are not C++-clean.
 */

__BEGIN_DECLS

/* Register for rpmsg device creation; idempotent. */
void remote_link_register(const char *cpuname);

/* RPTUNIOC_START, or RPTUNIOC_STOP without name-service teardown, on
 * /dev/rptun/<cpuname>. Returns 0 or -errno.
 */
int remote_link_rptun(bool start);

/* Send len bytes on the hello endpoint and wait up to timeout_ms for the
 * reply, copied NUL terminated into reply[cap] when reply is set; the round
 * trip in microseconds lands in rtt_us when set. Returns 0, -ENOTCONN if the
 * endpoint never bound, -ETIMEDOUT, or a negative rpmsg error.
 */
int remote_link_xfer(const void *payload, size_t len, char *reply, size_t cap, int timeout_ms,
		     uint64_t *rtt_us);

__END_DECLS
