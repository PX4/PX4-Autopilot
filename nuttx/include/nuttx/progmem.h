/****************************************************************************
 * include/nuttx/progmem.h
 *
 *   Copyright(C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_PROGMEM_H
#define __INCLUDE_NUTTX_PROGMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_progmem_npages
 *
 * Description:
 *   Return number of pages
 *
 ****************************************************************************/

uint16_t up_progmem_npages(void);

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *   Is program memory uniform or page size differs?
 *
 ****************************************************************************/

bool up_progmem_isuniform(void);

/****************************************************************************
 * Name: up_progmem_pagesize
 *
 * Description:
 *   Return page size
 *
 ****************************************************************************/

uint16_t up_progmem_pagesize(uint16_t page);

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to page conversion 
 *
 * Input Parameters:
 *   addr - Address with of without flash offset (absolute or aligned to page0)
 *
 * Returned Value:
 *   Page or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     EFAULT: On invalid address
 *
 ****************************************************************************/

int up_progmem_getpage(uint32_t addr);

/****************************************************************************
 * Name: up_progmem_erasepage
 *
 * Description:
 *   Erase selected page.
 *
 * Input Parameters:
 *   page - 
 *
 * Returned Value:
 *   Page size or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     EFAULT: On invalid page
 *     EIO: On unsuccessful erase
 *     EROFS: On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM: If operation is not permitted due to some other constraints
 *        (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

int up_progmem_erasepage(uint16_t page);

/****************************************************************************
 * Name: up_progmem_ispageerased
 *
 * Description:
 *   Checks whether page is erased
 *
 * Input Parameters:
 *    page - 
 *
 * Returned Value:
 *   Returns number of bytes written or negative value on error. If it
 *   returns zero then complete page is empty (erased).
 *
 *   The following errors are reported (errno is not set!)
 *     EFAULT: On invalid page
 *
 ****************************************************************************/

int up_progmem_ispageerased(uint16_t page);

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at given address
 *
 *   Note: this function is not limited to single page and nor it requires
 *   the address be aligned inside the page boundaries.
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset (absolute or aligned to page0)
 *   buf   - Pointer to buffer
 *   count - Number of bytes to write *
 *
 * Returned Value:
 *   Bytes written or negative value on error.  The following errors are
 *   reported (errno is not set!)
 *
 *     EINVAL: if count is not aligned with the flash boundaries (i.e.
 *        some MCU's require per half-word or even word access)
 *     EFAULT: On invalid address
 *     EIO: On unsuccessful write
 *     EROFS: On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM: If operation is not permitted due to some other constraints
 *        (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

int up_progmem_write(uint32_t addr, const void *buf, size_t count);

/* TODO: Define the following functions and their options:
 *  - up_progmem_protect() 
 *  - up_progmem_unprotect()
 */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_PROGMEM_H */
