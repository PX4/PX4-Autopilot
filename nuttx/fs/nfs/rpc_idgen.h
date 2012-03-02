
/*    $OpenBSD: idgen.h,v 1.2 2008/06/25 00:55:53 djm Exp $    */

/*
 */

/****************************************************************************
 * fs/nfs/rpc_idgen.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Leveraged from OpenBSD:
 *
 * Copyright (c) 2008 Damien Miller <djm@mindrot.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

#ifndef __FS_NFS_RPC_IDGEN_H
#define __FS_NFS_RPC_IDGEN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IDGEN32_ROUNDS        31
#define IDGEN32_KEYLEN        32
#define IDGEN32_REKEY_LIMIT   0x60000000
#define IDGEN32_REKEY_TIME    600

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct idgen32_ctx
{
  uint32_t id_counter;
  uint32_t id_offset;
  uint32_t id_hibit;
  uint8_t id_key[IDGEN32_KEYLEN];
  time_t id_rekey_time;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void idgen32_init(struct idgen32_ctx *);
uint32_t idgen32(struct idgen32_ctx *);

#endif /* __FS_NFS_RPC_IDGEN_H */

