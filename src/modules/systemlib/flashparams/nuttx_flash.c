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
 * @file flashparam.c
 *
 * Global flash based parameter store.
 *
 * This provides the mechanisms to interface to the PX4
 * parameter system but replace the IO with non file based flash
 * i/o routines. So that the code my be implemented on a SMALL memory
 * foot print device.
 */

#include <px4_defines.h>
#include <px4_posix.h>
#include <crc32.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include "nuttx_flash.h"
#include <nuttx/progmem.h>


/* The following code will use */

static uint8_t *working_buffer;
static uint16_t working_buffer_size;
static flash_file_sector_t* sector_map;
static int last_erased;

typedef enum  nuttx_flash_config_t {
    LargestBlock = 2*1024, // This represents the size need for backing store
    MagicSig     = 0xaa553cc3,
    BlankSig     = 0xffffffff
} nuttx_flash_config_t;

typedef enum  nuttx_flash_flags_t {
  Erased = 0xff,
  ValidFile = 0x0a33a,
  ErasedFile = 0x0,

} nuttx_flash_flags_t;

typedef struct flash_file_header_t {
  uint32_t             magic;           /* Used to ID files*/
  uint16_t             flag;            /* Used to erase this entry */
  uint32_t             crc;             /* Calculated over the size - end of data */
  uint16_t             size;            /* When added to a byte pointer to flash_file_header_t
                                         * Will result the offset of the next active file or
                                         * free space. */
  flash_file_token_t   file_token;      /* file token type - essentially the name/type */
} flash_file_header_t;


const flash_file_token_t parameters_token = {
    .n = {'p','a','r','m'},
};


int nuttx_flash_init(flash_file_sector_t *pconfig, uint8_t *buffer, uint16_t largest_block)
{
  sector_map = pconfig;
  working_buffer = buffer;
  working_buffer_size = largest_block;
  last_erased= -1;
  return 0;
}


static
flash_file_header_t * nuttx_flash_findfile(flash_file_token_t token)
{
  for (int s = 0; sector_map[s].address != 0; s++) {
      uint32_t * p32 = (uint32_t *) sector_map[s].address;
      uint32_t * pe = p32 + sector_map[s].size / sizeof(uint32_t);

      /* Hunt for Magic Signature */
cont:
      while(p32 != pe && *p32 != MagicSig) {
          p32++;
      }
      /* Did we reach the end
       * if so try the next sector */

      if (p32 == pe) continue;

      /* Found a magic So assume it is a file header */

      flash_file_header_t *pf = (flash_file_header_t*) p32;

      /* Test the CRC */

      if (pf->crc == crc32((const uint8_t *)&pf->size, pf->size-offsetof(flash_file_header_t, size))) {

          /* Good CRC is it the one we are lookking for ?*/

          if (pf->flag == ValidFile && pf->file_token.t == token.t) {

              return pf;

          } else {
              /* Not the one we wanted but we can trust the size */
              p32 = (uint32_t *) (((uint8_t *)pf) + pf->size);
              pf = (flash_file_header_t*) p32;

              /* If the next one is erased */
              if (pf->magic == BlankSig) continue;
          }
          goto cont;

      } else {
          /* in valid CRC so keep looking */
          p32++;
      }
  }
  return 0;
}

/*
 *  Given a pointer to sector information return the next sector information
 *  to write to.
 */
static flash_file_sector_t * get_next_sector(flash_file_sector_t * current)
{
  for (int s = 0; sector_map[s].address != 0; s++) {
      if (current == &sector_map[s]) {
        if (sector_map[s+1].address) {
              return &sector_map[s+1];
        } else {
              return &sector_map[0];
        }
      }
  }
  return 0;
}


/* Given sector info and a pf to validate */
static int nuttx_flash_erase_sector(flash_file_sector_t * sm, flash_file_header_t *pf)
{
  int rv = 0;
  ssize_t page = up_progmem_getpage(pf);
  if (page >= 0 && page == sm->page) {
      last_erased = sm->page;
      ssize_t size = up_progmem_erasepage(page);
      if (size < 0 || size != sm->size) {
          rv = size;
      }
  }
  return rv;
}
/*
 *  Given a pointer to a flash_file header return the sector information
 *  the pointer resides in
 */
static flash_file_sector_t * get_sector_info(flash_file_header_t *pf)
{
  for (int s = 0; sector_map[s].address != 0; s++) {
      uint8_t * pb = (uint8_t *) sector_map[s].address;
      uint8_t * pe = pb + sector_map[s].size - 1;
      if (pf > pb && pf < pe) {
          return &sector_map[s];
      }
  }
  return 0;
}

/*
 * Returns 0 if there is enough space left to write new size
 * If not it returns the flash_file_sector_t * that needs to
 * be erased.
 */

static flash_file_sector_t * check_free_space(flash_file_header_t *pf, size_t new_size)
{
      flash_file_sector_t * sm = get_sector_info(pf);
      uint8_t * pb = (uint8_t *) sm->address;
      uint8_t * pe = pb + sm->size - 1;
      if (pf >= pb && pf <= pe) {
          pb = (uint8_t *)pf;
          pb += pf->size;
          pb += new_size;
          if (pb >= pe) {
              return sm;
          }
      }
      return 0;
  }
}
int read_flash(flash_file_token_t ft, uint8_t **buffer, size_t *buf_size)
{
  int rv = -ENOENT;
  flash_file_header_t *pf = nuttx_flash_findfile(ft);
  if (pf) {
      (*buffer) = ((uint8_t*)pf) + sizeof(flash_file_header_t);
      *buf_size = pf->size - sizeof(flash_file_header_t);
  }
  return rv;
}


int write_flash(flash_file_token_t ft, uint8_t *buffer, size_t buf_size)
{
  int rv = 0;
  size_t total_size = buf_size + sizeof(flash_file_header_t);
  flash_file_header_t *pf = nuttx_flash_findfile(ft);
  if (pf != 0) {

      flash_file_sector_t * sm = check_free_space(pf, total_size);
      if (sm == 0) {
        uint16_t data = ErasedFile;
        rv = up_progmem_write((size_t) &pf->flag, &data, sizeof(data));
        if (rv == sizeof(data)) {
            pf = (uint8_t *)pf + pf->size;
        }
      } else {
         rv = nuttx_flash_erase_sector(sm, pf);
         sm = get_next_sector(sm);
         pf = sm->address;
      }
      if (blank_check(pf, total_size)) {
          flash_file_header_t *pn = (flash_file_header_t *) (buffer -= sizeof(flash_file_header_t));
          pn->magic = MagicSig;
          pn->file_token.t = ft.t;
          pn->flag = ValidFile;
          pn->size = total_size;
          pn->crc32(&pn->size,  total_size - offsetof(flash_file_header_t, size));
          rv = up_progmem_write(pn, pn->size);
      }

  }

  if (rv == -ENOENT) {
      pf = nuttx_flash_free(buf_size);

  }
  return 0;
}


int flash_alloc_buffer(flash_file_token_t ft, uint8_t **buffer, size_t *buf_size)
{
  (*buffer) = &working_buffer[sizeof(flash_file_header_t)];
  *buf_size = working_buffer_size-sizeof(flash_file_header_t);
  memset(working_buffer, 0xff , working_buffer_size);
  return 0;
}



static flash_file_sector_t  test_sector_map[] =
    {
        {1, 16*1024, 0x08004000},
        {2, 16*1024, 0x08004000},
        {0, 0, 0},
    };

__EXPORT void test(void);
__EXPORT void test(void)
{
    uint16_t largest_block = 2048;
    uint8_t *buffer = malloc(largest_block);

    nuttx_flash_init(test_sector_map, buffer, largest_block);

    flash_file_header_t *pf = nuttx_flash_findfile(parameters_token);
    if (pf) {
        static volatile int j = 0;
        nuttx_flash_erasefile(parameters_token);
        j++;
    }
}
