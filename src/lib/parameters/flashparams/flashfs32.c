/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file flashfs32.c
 *
 * Global flash based parameter store.
 *
 * This provides the mechanisms to interface to the PX4
 * parameter system but replace the IO with non file based flash
 * i/o routines. So that the code my be implemented on a SMALL memory
 * foot print device.
 *
 * It was made to provide flashfs functionality on the STM32H7 chip,
 * which uses 32-byte flash lines that cannot be overwritten because of
 * on-board CRC functionality.
 */

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <crc32.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include "flashfs.h"
#include <nuttx/compiler.h>
#include <nuttx/progmem.h>
#include <board_config.h>

#if defined(CONFIG_ARCH_HAVE_PROGMEM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_FLAG_ROW_SIZE 16 ///< size in h_flag_t units of a flash row

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef uint32_t h_magic_t;
typedef uint16_t h_size_t;
typedef uint16_t h_flag_t;
typedef uint16_t data_size_t;

typedef enum  flash_config_t {
	LargestBlock = 2 * 1024, // This represents the size need for backing store
	MagicSig     = 0xaa553cc3,
	BlankSig     = 0xffffffff
} flash_config_t;

/* Aligns are done up to 32 bytes, so storing the size padding within 0x001f mask */
typedef enum  flash_flags_t {
	SizeMask      = 0x001f,
	MaskEntry     = ~SizeMask,
	BlankEntry    = (h_flag_t)BlankSig,
	ValidEntry    = (0xa5ac & ~SizeMask),
	ErasedEntry   = 0x0000,
} flash_flags_t;

/* The struct flash_entry_header_t will be 32-byte aligned
 * The Size will be the actual length of the header plus the data
 * and any padding needed to have the size be an even multiple of
 * the 32-byte flash line
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
typedef begin_packed_struct struct flash_entry_header_t {
	h_magic_t            magic;           /* Used to ID files*/
	h_flag_t             flag;            /* Used to mark this entry as valid */
	uint8_t              unused_buffer[32 - sizeof(h_magic_t) - sizeof(h_flag_t)]; /* padding */
	h_flag_t             flag_erased;     /* Used to erase this entry */
	uint8_t              unused_buffer2[32 - sizeof(h_flag_t)]; /* padding */
	uint32_t             crc;             /* Calculated over the size - end of data */
	h_size_t             size;            /* When added to a byte pointer to flash_entry_header_t
                                               * Will result the offset of the next active file or
                                               * free space. */
	flash_file_token_t   file_token;      /* file token type - essentially the name/type */
} end_packed_struct flash_entry_header_t __attribute__((aligned(sizeof(uint32_t))));
#pragma GCC diagnostic pop

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static uint8_t *working_buffer;
static uint16_t working_buffer_size;
static bool working_buffer_static;
static sector_descriptor_t *sector_map;

/****************************************************************************
 * Public Data
 ****************************************************************************/

const flash_file_token_t parameters_token = {
	.n = {'p', 'a', 'r', 'm'},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parameter_flashfs_free
 *
 * Description:
 *   Frees  dynamically allocated memory
 *
 *
 ****************************************************************************/

void parameter_flashfs_free(void)
{
	if (!working_buffer_static && working_buffer != NULL) {
		free(working_buffer);
		working_buffer = NULL;
		working_buffer_size = 0;
	}
}

/****************************************************************************
 * Name: blank_flash
 *
 * Description:
 *   This helper function returns true if the pointer points to a blank_flash
 *   location
 *
 * Input Parameters:
 *   pf   - A pointer to memory aligned on sizeof(uint32_t) boundaries
 *
 * Returned value:
 *   true if blank
 *
 *
 ****************************************************************************/

static inline int blank_flash(uint32_t *pf)
{
	return *pf == BlankSig;
}

/****************************************************************************
 * Name: blank_check
 *
 * Description:
 *   Given a pointer to a flash entry header and a new size
 *
 * Input Parameters:
 *   pf       - A pointer to the current flash entry header
 *   new_size - The total number of bytes to be written
 *
 * Returned value:
 *  true if space is blank, If it is not blank it returns false
 *
 ****************************************************************************/

static bool blank_check(flash_entry_header_t *pf,
			size_t new_size)
{
	bool rv = true;
	uint32_t *pm = (uint32_t *) pf;
	new_size /= sizeof(uint32_t);

	while (new_size-- && rv) {
		if (!blank_flash(pm++)) {
			rv = false;
		}
	}

	return rv;
}

/****************************************************************************
 * Name: valid_magic
 *
 * Description:
 *   This helper function returns true if the pointer points to a valid
 *   magic signature
 *
 * Input Parameters:
 *   pm   - A pointer to memory aligned on sizeof(h_magic_t) boundaries
 *
 * Returned value:
 *   true if magic is valid
 *
 *
 ****************************************************************************/

static inline int valid_magic(h_magic_t *pm)
{
	return *pm == MagicSig;
}

/****************************************************************************
 * Name: blank_magic
 *
 * Description:
 *   This helper function returns true if the pointer points to a valid
 *   blank magic signature
 *
 * Input Parameters:
 *   pm   - A pointer to memory aligned on sizeof(h_magic_t) boundaries
 *
 * Returned value:
 *   true if magic is valid
 *
 *
 ****************************************************************************/

static inline int blank_magic(h_magic_t *pm)
{
	return *pm == BlankSig;
}

/****************************************************************************
 * Name: erased_entry
 *
 * Description:
 *   This helper function returns true if the entry is Erased
 *
 * Input Parameters:
 *   fi            - A pointer to the current flash entry header
 *
 * Returned value:
 *   true if  erased
 *
 *
 ****************************************************************************/

static inline int erased_entry(flash_entry_header_t *fi)
{
	return (fi->flag_erased & MaskEntry) == ErasedEntry;
}

/****************************************************************************
 * Name: blank_entry
 *
 * Description:
 *   This helper function returns true if the entry is Blank
 *
 * Input Parameters:
 *   fi            - A pointer to the current flash entry header
 *
 * Returned value:
 *   true if Blank
 *
 *
 ****************************************************************************/

static inline int blank_entry(flash_entry_header_t *fi)
{
	return fi->magic == BlankSig && fi->flag == BlankEntry && fi->flag_erased == BlankEntry;
}

/****************************************************************************
 * Name: valid_entry
 *
 * Description:
 *   This helper function returns true if the entry is Blank
 *
 * Input Parameters:
 *   fi            - A pointer to the current flash entry header
 *
 * Returned value:
 *   true if valid_entry
 *
 *
 ****************************************************************************/

static inline int valid_entry(flash_entry_header_t *fi)
{
	return (fi->flag & MaskEntry) == ValidEntry && (fi->flag_erased) == BlankEntry;
}

/****************************************************************************
 * Name: entry_size_adjust
 *
 * Description:
 *   This helper function returns the size adjust
 *
 * Input Parameters:
 *   fi            - A pointer to the current flash entry header
 *
 * Returned value:
 *   true if valid_entry
 *
 *
 ****************************************************************************/

static inline int entry_size_adjust(flash_entry_header_t *fi)
{
	return fi->flag & SizeMask;
}

/****************************************************************************
 * Name: next_entry
 *
 * Description:
 *   This helper function advances the flash entry header pointer to the
 *   locations of the next entry.
 *
 * Input Parameters:
 *   fh            - A pointer to the current file header
 *
 * Returned value:
 *                - A pointer to the next file header location
 *
 *
 ****************************************************************************/

static inline flash_entry_header_t *next_entry(flash_entry_header_t *fi)
{
	uint8_t *pb = (uint8_t *)fi;
	return (flash_entry_header_t *)(pb + fi->size);
}

/****************************************************************************
 * Name: entry_data
 *
 * Description:
 *   This helper function returns a pointer the the data in the entry
 *
 * Input Parameters:
 *   fh            - A pointer to the current file header
 *
 * Returned value:
 *                - A pointer to the next file header location
 *
 *
 ****************************************************************************/

static inline uint8_t *entry_data(flash_entry_header_t *fi)
{
	return ((uint8_t *)fi) + sizeof(flash_entry_header_t);
}

/****************************************************************************
 * Name: entry_data_length
 *
 * Description:
 *   This helper function returns the size of the user data
 *
 * Input Parameters:
 *   fh            - A pointer to the current file header
 *
 * Returned value:
 *                - The length of the data in the entry
 *
 *
 ****************************************************************************/

static inline data_size_t entry_data_length(flash_entry_header_t *fi)
{
	return fi->size - (sizeof(flash_entry_header_t) + entry_size_adjust(fi));
}

/****************************************************************************
 * Name: entry_crc_start
 *
 * Description:
 *   This helper function returns a const byte pointer to the location
 *   where the CRC is calculated over
 *
 * Input Parameters:
 *   fi  - A pointer to the current file header
 *
 * Returned value:
 * A pointer to the point at which the crc is calculated from.
 *
 *
 ****************************************************************************/

static inline const uint8_t *entry_crc_start(flash_entry_header_t *fi)
{
	return (const uint8_t *)&fi->size;
}

/****************************************************************************
 * Name: entry_crc_length
 *
 * Description:
 *   This helper function returns the length of the regone where the CRC is
 *   calculated over
 *
 * Input Parameters:
 *   fi  - A pointer to the current file header
 *
 * Returned value:
 * Number of bytes to to crc
 *
 *
 ****************************************************************************/

static inline data_size_t entry_crc_length(flash_entry_header_t *fi)
{
	return fi->size - offsetof(flash_entry_header_t, size);
}

/****************************************************************************
 * Name: find_entry
 *
 * Description:
 *   This helper function locates an "file" from the the file token
 *
 * Input Parameters:
 *   token        - A flash file token, the pseudo file name
 *
 * Returned value:
 *  On Success a pointer to flash entry header or NULL on failure
 *
 *
 ****************************************************************************/

static flash_entry_header_t *find_entry(flash_file_token_t token)
{
	for (int s = 0; sector_map[s].address; s++) {

		h_magic_t *pmagic = (h_magic_t *) sector_map[s].address;
		h_magic_t *pe = pmagic + (sector_map[s].size / sizeof(h_magic_t)) - 1;

		/* Hunt for Magic Signature */
cont:

		while (pmagic < pe && !valid_magic(pmagic)) {
			pmagic++;
		}

		/* Did we reach the end
		 * if so try the next sector */

		if (pmagic >= pe) { continue; }

		/* Found a magic So assume it is a file header */

		flash_entry_header_t *pf = (flash_entry_header_t *) pmagic;

		/* Ensure that the header is fully inside the current sector */

		if (pf + 1 > (flash_entry_header_t *)pe) { continue; }

		const uint8_t *crc_start = entry_crc_start(pf);
		data_size_t crc_length = entry_crc_length(pf);

		if (crc_start + crc_length > (uint8_t *)pe) { continue; }

		if (pf->crc == crc32(crc_start, crc_length)) {

			/* Good CRC is it the one we are looking for ?*/

			if (valid_entry(pf) && pf->file_token.t == token.t) {

				return pf;

			} else {

				/* Not the one we wanted but we can trust the size */

				pf = next_entry(pf);
				pmagic = (h_magic_t *) pf;

				/* If the next one is erased */

				if (pmagic >= pe || blank_entry(pf)) {
					continue;
				}
			}

		} else {

			/* invalid CRC so keep looking */

			pmagic++;
		}

		goto cont;
	}

	return NULL;
}

/****************************************************************************
 * Name: find_free
 *
 * Description:
 *   This helper function locates free space for the number of bytes required
 *
 * Input Parameters:
 *   required  - Number of bytes required
 *
 * Returned value:
 *  On Success a pointer to flash entry header or NULL on failure
 *
 *
 ****************************************************************************/

static flash_entry_header_t *find_free(data_size_t required)
{
	for (int s = 0; sector_map[s].address; s++) {

		h_magic_t *pmagic = (h_magic_t *) sector_map[s].address;
		h_magic_t *pe = pmagic + (sector_map[s].size / sizeof(h_magic_t)) - 1;

		/* Hunt for Magic Signature */

		do {

			if (valid_magic(pmagic)) {

				flash_entry_header_t *pf = (flash_entry_header_t *) pmagic;

				/* Ensure that the header is fully inside the current sector */

				if (pf + 1 > (flash_entry_header_t *)pe) { break; }

				/* Test the CRC */

				if (pf->crc == crc32(entry_crc_start(pf), entry_crc_length(pf))) {

					/* Valid Magic and CRC look for the next record*/

					pmagic = ((uint32_t *) next_entry(pf));

				} else {

					pmagic++;
				}
			}

			if (pmagic + (required / sizeof(h_magic_t)) <= pe && blank_magic(pmagic)) {

				flash_entry_header_t *pf = (flash_entry_header_t *) pmagic;

				if (blank_entry(pf) && blank_check(pf, required)) {
					return pf;
				}

			}
		}  while (++pmagic < pe);
	}

	return NULL;
}

/****************************************************************************
 * Name: get_next_sector_descriptor
 *
 * Description:
 *   Given a pointer to sector_descriptor_t, this helper function
 *   returns a pointer to the next sector_descriptor_t
 *
 * Input Parameters:
 *   current      - A pointer to the current sector_descriptor_t
 *
 * Returned value:
 *  On Success A pointer to the next sector_descriptor_t,
 *  otherwise NULL
 *
 *
 ****************************************************************************/

static sector_descriptor_t *get_next_sector_descriptor(sector_descriptor_t *
		current)
{
	for (int s = 0; sector_map[s].address; s++) {
		if (current == &sector_map[s]) {
			if (sector_map[s + 1].address) {
				s++;

			} else {
				s = 0;
			}

			return &sector_map[s];
		}
	}

	return NULL;
}

/****************************************************************************
 * Name: get_sector_info
 *
 * Description:
 *   Given a pointer to a flash entry header returns the sector descriptor
 *   for the file is located in
 *
 * Input Parameters:
 *   current      - A pointer to the current flash entry header
 *
 * Returned value:
 *  On Success A pointer to the next sector_descriptor_t,
 *  otherwise NULL
 *
 *
 ****************************************************************************/

static sector_descriptor_t *get_sector_info(flash_entry_header_t *current)
{
	for (int s = 0; sector_map[s].address != 0; s++) {
		uint8_t *pb = (uint8_t *) sector_map[s].address;
		uint8_t *pe = pb + sector_map[s].size - 1;
		uint8_t *pc = (uint8_t *) current;

		if (pc >= pb && pc <= pe) {
			return &sector_map[s];
		}
	}

	return 0;
}

/****************************************************************************
 * Name: erase_sector
 *
 * Description:
 *   Given a pointer to sector_descriptor_t, this function
 *   erases the sector using the pointer to the
 *   flash_entry_header_t as a sanity check
 *
 * Input Parameters:
 *   sm      - A pointer to the current sector_descriptor_t
 *   pf      - A pointer to the current flash entry header
 *
 * Returned value:
 *  O On Success or a negative errno
 *
 *
 ****************************************************************************/

static int erase_sector(sector_descriptor_t *sm, flash_entry_header_t *pf)
{
	int rv = 0;

	/*
	* Because up_progmem interface doesn't allow one-to-one check of
	* the block number for an address, use uniform-memory assumption
	* to confirm the block number in the sector descriptor is
	* the correct block to erase for the given address.
	*
	* This is an optional sanity check against the sector configuration
	*/
	if (up_progmem_isuniform()) {
		ssize_t page = up_progmem_getpage((size_t)pf);

		if (page < 0) {
			rv = page;
		}

		if (rv == 0) {
			size_t page_size  = up_progmem_pagesize(page);
			size_t block_size = up_progmem_erasesize(sm->page);
			size_t block_num  = ((page * page_size) / block_size);

			if (block_num != sm->page) {
				rv = -EINVAL;
			}
		}
	}

	if (rv == 0) {

		ssize_t size_erased = up_progmem_eraseblock(sm->page);

		if (size_erased < 0 || size_erased != (ssize_t)sm->size) {
			rv = size_erased;
		}

	}

	return rv;
}

/****************************************************************************
 * Name: erase_entry
 *
 * Description:
 *   Given a pointer to a flash entry header erases the entry
 *
 * Input Parameters:
 *   pf  - A pointer to the current flash entry header
 *
 *
 * Returned value:
 *  >0 On Success or a negative errno
 *
 *
 ****************************************************************************/

static int erase_entry(flash_entry_header_t *pf)
{
	/* Need to write entire 32-byte line */
	h_flag_t data[FLASH_FLAG_ROW_SIZE] = {0xffff};
	data[0] = ErasedEntry;
	size_t size = FLASH_FLAG_ROW_SIZE * sizeof(h_flag_t);
	int rv = up_progmem_write((size_t) &pf->flag_erased, data, size);
	return rv;
}

/****************************************************************************
 * Name: check_free_space_in_sector
 *
 * Description:
 *   Given a pointer to a flash entry header and a new size
 *
 * Input Parameters:
 *   pf       - A pointer to the current flash entry header
 *   new_size - The total number of bytes to be written
 *
 * Returned value:
 *  0 if there is enough space left to write new size
 *  If not it returns the flash_file_sector_t * that needs to be erased.
 *
 ****************************************************************************/

static sector_descriptor_t *check_free_space_in_sector(flash_entry_header_t
		*pf, size_t new_size)
{
	sector_descriptor_t *sm = get_sector_info(pf);
	uint8_t *psector_first = (uint8_t *) sm->address;
	uint8_t *psector_last = psector_first + sm->size - 1;
	uint8_t *pnext_end = (uint8_t *)(valid_magic((h_magic_t *)pf) ? next_entry(pf) : pf) + new_size;

	if (pnext_end >= psector_first && pnext_end <= psector_last) {
		sm = 0;
	}

	return sm;
}

/****************************************************************************
 * Name: write_flash_entry
 *
 * Description:
 *   Writes a 32-byte-aligned flash entry to flash memory
 *   Notably, does not write the flag_erased line, but reports
 *   the line as written regardless; the line's contents must remain
 *   unwritten to avoid possible CRC errors in hardware later while erasing.
 *
 * Input Parameters:
 *   pf       - Address in flash to which to write
 *   pn       - Pointer to flash_entry_header_t to which to write
 *
 * Returned value:
 *   On success the number of bytes written (including the unwritten
 *     flag_erased line). On Error, a negative value of errno
 *
 ****************************************************************************/

static int write_flash_entry(size_t pf, const flash_entry_header_t *pn)
{
	ssize_t rv;
	size_t line_size = up_progmem_pagesize(up_progmem_getpage(pf));
	uint8_t *first_flag_line  = (uint8_t *)pn;
	uint8_t *content_buffer   = (uint8_t *)(&pn->crc);

	/* write first line (magic, flag) */
	rv = up_progmem_write(pf, first_flag_line, line_size);

	if (rv < 0) {
		return rv;
	}

	/* pretend to write the second line (flag_erased) */
	rv += line_size;

	/* write the remaining contents */
	ssize_t rv2 = up_progmem_write(pf + 2 * line_size, content_buffer, pn->size - 2 * line_size);

	if (rv2 < 0) {
		return rv2;
	}

	return rv + rv2;
}//write_flash_entry

/****************************************************************************
 * Public Functions
 ****************************************************************************/
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

int parameter_flashfs_read(flash_file_token_t token, uint8_t **buffer, size_t
			   *buf_size)
{
	int rv = -ENXIO;

	if (sector_map) {

		rv = -ENOENT;
		flash_entry_header_t *pf = find_entry(token);

		if (pf) {
			(*buffer) = entry_data(pf);
			rv = entry_data_length(pf);
			*buf_size = rv;
		}
	}

	return rv;
}

/****************************************************************************
 * Name: parameter_flashfs_write
 *
 * Description:
 *   This function writes user data from the buffer allocated with a previous call
 *   to parameter_flashfs_alloc. flash starting at the given address
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

int
parameter_flashfs_write(flash_file_token_t token, uint8_t *buffer, size_t buf_size)
{
	int rv = -ENXIO;

	if (sector_map) {

		rv = 0;

		/* Calculate the total space needed */
		size_t total_size = buf_size + sizeof(flash_entry_header_t);
		size_t alignment = SizeMask;///< 32-byte flash row
		size_t  size_adjust = ((total_size + alignment) & ~alignment) - total_size;
		total_size += size_adjust;

		/* Is this and existing entry */

		flash_entry_header_t *pf = find_entry(token);

		if (!pf) {

			/* No Entry exists for this token so find a place for it */
			pf = find_free(total_size);

			/* No Space */

			if (pf == 0) {
				return -ENOSPC;
			}

		} else {

			/* Do we have space after the entry in the sector for the update */

			sector_descriptor_t *current_sector = check_free_space_in_sector(pf,
							      total_size);


			if (current_sector == 0) {

				/* Mark the last entry erased */

				/* todo:consider a 2 stage erase or write before erase and do a fs check
				 * at start up
				 */

				rv = erase_entry(pf);

				if (rv < 0) {
					return rv;
				}

				/* We had space and marked the last entry erased so use the  Next Free */

				pf = next_entry(pf);

			} else {

				/*
				 * We did not have space in the current sector so select the next sector
				 */

				current_sector = get_next_sector_descriptor(current_sector);

				/* Will the data fit */

				if (current_sector->size < total_size) {
					return -ENOSPC;
				}

				/* Mark the last entry erased */

				/* todo:consider a 2 stage erase or write before erase and do a fs check
				 * at start up
				 */

				rv = erase_entry(pf);

				if (rv < 0) {
					return rv;
				}

				pf = (flash_entry_header_t *) current_sector->address;

				if (!blank_check(pf, total_size)) {
					rv = erase_sector(current_sector, pf);
				}
			}//free_space_in_sector returned not-zero

		}

		flash_entry_header_t *pn = (flash_entry_header_t *)(buffer - sizeof(flash_entry_header_t));
		pn->magic = MagicSig;
		pn->file_token.t = token.t;
		pn->flag = ValidEntry + size_adjust;
		pn->size = total_size;

		for (size_t a = 0; a < size_adjust; a++) {
			buffer[buf_size + a] = (uint8_t)BlankSig;
		}

		pn->crc = crc32(entry_crc_start(pn), entry_crc_length(pn));
		rv = write_flash_entry((size_t)pf, pn);
		int system_bytes = (sizeof(flash_entry_header_t) + size_adjust);

		if (rv >= system_bytes) {
			rv -= system_bytes;
		}
	}

	return rv;
}

/****************************************************************************
 * Name: parameter_flashfs_alloc
 *
 * Description:
 *   This function is called to get a buffer to use in a subsequent call
 *   to parameter_flashfs_write. The address returned is advanced into the
 *   buffer to reserve space for the flash entry header.
 *   The caller is responsible to call parameter_flashfs_free after usage.
 *
 * Input Parameters:
 *   token      - File Token File to read (not used)
 *   buffer     - A pointer to return a pointer to Memory of buf_size length
 *   			  suitable for calling parameter_flashfs_write
 *   buf_size   - In: the size needed for the write operation.
 *   			  Out: The maximum number of bytes that can be written to
 *                the buffer
 *
 * Returned value:
 *   On success the number of bytes written On Error a negative value of errno
 *
 ****************************************************************************/

int parameter_flashfs_alloc(flash_file_token_t token, uint8_t **buffer, size_t *buf_size)
{
	int rv = -ENXIO;

	if (sector_map) {

		rv = -ENOMEM;

		if (!working_buffer_static) {

			working_buffer_size = *buf_size + sizeof(flash_entry_header_t) + SizeMask;
			working_buffer = malloc(working_buffer_size);

		}

		/* Allocation failed or not provided */

		if (working_buffer == NULL) {

			working_buffer_size = 0;

		} else {

			/* We have a buffer reserve space and init it */
			*buffer = &working_buffer[sizeof(flash_entry_header_t)];
			*buf_size = working_buffer_size - sizeof(flash_entry_header_t) - SizeMask;
			memset(working_buffer, 0xff, working_buffer_size);
			rv = 0;

		}
	}

	return rv;
}

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

int parameter_flashfs_erase(void)
{
	int rv = -ENXIO;

	if (sector_map) {
		rv = 0;

		for (int s = 0; sector_map[s].address; s++) {
			int sz = erase_sector(&sector_map[s], (flash_entry_header_t *)sector_map[s].address);

			if (sz != 0) {
				return sz;
			}

			rv += sector_map[s].size;
		}
	}

	return rv;
}


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

int parameter_flashfs_init(sector_descriptor_t *fconfig, uint8_t *buffer, uint16_t size)
{
	int rv = 0;
	sector_map = fconfig;
	working_buffer_static = buffer != NULL;

	if (!working_buffer_static) {
		size = 0;
	}

	working_buffer = buffer;
	working_buffer_size = size;

	/* Sanity check */

	flash_entry_header_t *pf = find_entry(parameters_token);

	/*  No paramaters */

	if (pf == NULL) {
		// Parameters can't be found, assume sector is corrupt or empty
		rv = parameter_flashfs_erase();

		// A positive return value means flash space has been erased successfully.
		if (rv > 0) {
			rv = 0;
		}
	}

	return rv;
}

#if defined(FLASH_UNIT_TEST)

static sector_descriptor_t  test_sector_map[] = {
	{14, 128 * 1024, 0x081c0000},
	{15, 128 * 1024, 0x081e0000},
	{0, 0, 0},
};

__EXPORT void test(void);

uint8_t test_buf[32 * 1024];

__EXPORT void test(void)
{
	uint16_t largest_block = (32 * 1024) + 96;
	uint8_t *buffer = malloc(largest_block);

	parameter_flashfs_init(test_sector_map, buffer, largest_block);

	for (int t = 0; t < sizeof(test_buf); t++) {
		test_buf[t] = (uint8_t) t;
	}

	int er = parameter_flashfs_erase();
	uint8_t *fbuffer;
	size_t buf_size;
	int written = 0;
	int read = 0;
	int rv = 0;

	for (int a = 0; a <= 4; a++) {
		rv =  parameter_flashfs_alloc(parameters_token, &fbuffer, &buf_size);
		memcpy(fbuffer, test_buf, a);
		buf_size = a;
		written = parameter_flashfs_write(parameters_token, fbuffer, buf_size);
		read = parameter_flashfs_read(parameters_token, &fbuffer, &buf_size);
		parameter_flashfs_free();

		if (read != written) {
			static volatile int j;
			j++;
		}
	}

	int block = 2048;

	for (int a = 0; a <= 8; a++) {
		rv =  parameter_flashfs_alloc(parameters_token, &fbuffer, &buf_size);
		memcpy(fbuffer, test_buf, block);
		buf_size = block;
		written = parameter_flashfs_write(parameters_token, fbuffer, buf_size);
		read = parameter_flashfs_read(parameters_token, &fbuffer, &buf_size);
		parameter_flashfs_free();

		if (read != written) {
			static volatile int j;
			j++;
		}

		block += 2048;
	}

	rv++;
	er++;
	free(buffer);
}
#endif /* FLASH_UNIT_TEST */
#endif /* CONFIG_ARCH_HAVE_PROGMEM */
