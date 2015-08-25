/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/**
 * @file param.h
 *
 * Global flash based parameter store.
 *
 * This provides the mechanisms to interface to the PX4
 * parameter system but replace the IO with non file based flash
 * i/o routines. So that the code my be implemented on a SMALL memory
 * foot print device.
 *
 */

#ifndef _SYSTEMLIB_FLASHPARAMS_NUTTX_FLASH_H
#define _SYSTEMLIB_FLASHPARAMS_NUTTX_FLASH_H

#include <stdint.h>
#include <stdbool.h>


__BEGIN_DECLS

/*
 *  Define the interface data
 *  A flash_file_token_t is like a file name
 *
 */
typedef uint32_t flash_file_tokens_t;

typedef struct flash_file_token_t {
  union {
    flash_file_tokens_t t;
    uint8_t n[sizeof(flash_file_tokens_t)];
  };
} flash_file_token_t;

/*Define the parameter "file name" */
__EXPORT extern const flash_file_token_t parameters_token;

/* Define the initialization  */
typedef struct flash_file_sector_t {
  uint8_t       page;
  uint16_t      size;
  uint32_t      address;
} flash_file_sector_t;


int nuttx_flash_init(flash_file_sector_t *pconfig, uint8_t *buffer, uint16_t largest_block);

int read_flash(flash_file_token_t ft, uint8_t **buffer, size_t *buf_size);
int write_flash(flash_file_token_t ft, uint8_t *buffer, size_t buf_size);
int flash_alloc_buffer(flash_file_token_t ft, uint8_t **buffer, size_t *buf_size);
__END_DECLS
#endif /* _SYSTEMLIB_FLASHPARAMS_NUTTX_FLASH_H */
