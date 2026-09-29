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
 * @file flashfs.h
 *
 * Global flash based parameter store.
 *
 * This provides the mechanisms to interface to the PX4
 * parameter system but replace the IO with non file based flash
 * i/o routines. So that the code my be implemented on a SMALL memory
 * foot print device.
 *
 */

#ifndef _SYSTEMLIB_FLASHPARAMS_NUTTX_PARAM_H
#define _SYSTEMLIB_FLASHPARAMS_NUTTX_PARAM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>



/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*
 *  PARAMETER_BUFFER_SIZE must be defined larger then the maximum parameter
 * memory needed to commit the recored + ~20 bytes. For the syslib's parameter
 * this would be the size of the bson representations of the data
 */
#if !defined(PARAMETER_BUFFER_SIZE)
#define PARAMETER_BUFFER_SIZE 512
#endif

__BEGIN_DECLS

/*
 *  Define the interface data a flash_file_token_t
 *  is like a file name
 *
 */
typedef uint32_t flash_file_tokens_t;

typedef struct flash_file_token_t {
	union {
		flash_file_tokens_t t;
		uint8_t n[sizeof(flash_file_tokens_t)];
	};
} flash_file_token_t;

/*
 * Parameter "file names". parameters_token marks records of the append log
 * (a snapshot followed by deltas, replayed in order). parameters_legacy_token
 * is the single-snapshot record written by firmware before the log format;
 * that firmware ignores parameters_token records, so it boots on defaults
 * from a log-format store rather than a partial set.
 */
__EXPORT extern const flash_file_token_t parameters_token;
__EXPORT extern const flash_file_token_t parameters_legacy_token;

/* Define the elements of the array passed to the
 * parameter_flashfs_init function
 *
 * For example
 * static sector_descriptor_t  sector_map[] = {
 *      {1, 16 * 1024, 0x08004000},
 *      {2, 16 * 1024, 0x08008000},
 *       {0, 0, 0},
 *
 */
typedef struct sector_descriptor_t {
	uint16_t      page;
	uint32_t      size;
	uint32_t      address;
} sector_descriptor_t;


/****************************************************************************
 * Name: parameter_flashfs_init
 *
 * Description:
 *   This helper function advances the flash entry header pointer to the
 *   locations of the next entry.
 *
 * Input Parameters:
 *   fconfig      - A pointer to an null entry terminated array of
 *                  flash_file_sector_t
 *    buffer      - A pointer to a memory to make available to callers
 *                  for write operations. When allocated to the caller
 *                  space is reserved in the front for the
 *                  flash_entry_header_t.
 *                  If this is passes as NULL. The buffer will be
 *                  allocated from the heap on calls to
 *                  parameter_flashfs_alloc and fread on calls
 *                  to parameter_flashfs_free
 *
 *   size         - The size of the buffer in bytes. Should be be 0 if buffer
 *                  is NULL
 *
 * Returned value:
 *                - A pointer to the next file header location
 *
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_init(sector_descriptor_t *fconfig, uint8_t *buffer, uint16_t size);

/****************************************************************************
 * Name: parameter_flashfs_read
 *
 * Description:
 *   This function returns a pointer to the locations of the data associated
 *   with the file token. On successful return *buffer will be set to Flash
 *   location and *buf_size the length of the user data.
 *
 * Input Parameters:
 *   token       - File Token File to read
 *   buffer      - A pointer to a pointer that will receive the address
 *                 in flash of the data of this "files" data
 *   buf_size    - A pointer to receive the number of bytes in the "file"
 *
 * Returned value:
 *   On success number of bytes read or a negative errno value,
 *
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_read(flash_file_token_t ft, uint8_t **buffer, size_t *buf_size);

/****************************************************************************
 * Name: parameter_flashfs_walk
 *
 * Description:
 *   Call cb for every valid entry matching token, in the order they were
 *   written. cb returns 0 to continue, or a negative errno to abort.
 *
 * Returned value:
 *   Number of entries visited, or a negative errno (including from cb).
 *
 ****************************************************************************/

typedef int (*parameter_flashfs_walk_cb)(uint8_t *buffer, size_t buf_size, void *arg);

__EXPORT int parameter_flashfs_walk(flash_file_token_t token, parameter_flashfs_walk_cb cb, void *arg);

/****************************************************************************
 * Name: parameter_flashfs_blank
 *
 * Description:
 *   Reports whether the parameter storage is fully erased. This lets callers
 *   tell a never-written store (first boot, or after switching firmware) from
 *   one that holds data but no valid entry (torn write, bit-rot, or foreign
 *   data) - both of which surface as -ENOENT from parameter_flashfs_read.
 *
 * Returned value:
 *   1 if every configured sector is fully erased
 *   0 if any byte is programmed
 *   A negative errno if the flashfs has not been initialized
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_blank(void);

/****************************************************************************
 * Name: parameter_flashfs_write
 *
 * Description:
 *   Append a new entry after the last CRC-valid record. Previous valid
 *   entries are left intact so a boot replay can apply the whole log; the
 *   caller erases and rewrites a snapshot when a burst of deltas would no
 *   longer fit. Returns -ENOSPC if the new entry does not fit after the log.
 *
 * Input Parameters:
 *   token      - File Token File to read
 *   buffer      - A pointer to a buffer with buf_size bytes to be written
 *                 to the flash. This buffer must be allocated
 *                 with a previous call to parameter_flashfs_alloc
 *   buf_size    - Number of bytes to write
 *
 * Returned value:
 *   On success the number of bytes written On Error a negative value of errno
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_write(flash_file_token_t ft, uint8_t *buffer, size_t buf_size);

/****************************************************************************
 * Name: parameter_flashfs_erase
 *
 * Description:
 *   This function erases the sectors that were passed to parameter_flashfs_init
 *
 * Input Parameters:
 *
 * Returned value:
 *   On success the number of bytes erased
 *   On Error a negative value of errno
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_erase(void);

/****************************************************************************
 * Name: parameter_flashfs_needs_compact
 *
 * Description:
 *   True when the log for token should be rewritten as one snapshot of
 *   snapshot_size payload bytes: the live records are not a packed prefix
 *   of sector 0, or the tail after the last CRC-valid record cannot hold a
 *   burst the size of that snapshot (floored at 8 KB so a UUID append does
 *   not force a boot erase) and a compacted layout would. The erase stalls
 *   every read of that flash bank, so callers run it at boot before sensors
 *   and DShot start.
 *
 * Returned value:
 *   1 if compact is needed, 0 if not, or a negative errno.
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_needs_compact(flash_file_token_t token, size_t snapshot_size);

/****************************************************************************
 * Name: parameter_flashfs_max_payload
 *
 * Description:
 *   Largest user payload that fits in one mapped sector, or 0.
 *
 ****************************************************************************/

__EXPORT size_t parameter_flashfs_max_payload(void);

/****************************************************************************
 * Name: parameter_flashfs_alloc
 *
 * Description:
 *   This function is called to get a buffer to use in a subsequent call
 *   to parameter_flashfs_write. The address returned is advanced into the
 *   buffer to reserve space for the flash entry header.
 *
 * Input Parameters:
 *   token      - File Token File to read (not used)
 *   buffer     - Memory of buf_size length suitable for calling
 *                parameter_flashfs_write
 *   buf_size   - The maximum number of bytes that can be written to
 *                the buffer
 *
 * Returned value:
 *   On success the number of bytes written On Error a negative value of errno
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_alloc(flash_file_token_t ft, uint8_t **buffer, size_t *buf_size);


/****************************************************************************
 * Name: parameter_flashfs_free
 *
 * Description:
 *   Frees  dynamically allocated memory
 *
 *
 ****************************************************************************/

__EXPORT void parameter_flashfs_free(void);

__END_DECLS
#endif /* _SYSTEMLIB_FLASHPARAMS_NUTTX_PARAM_H */
