/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_flexspi_nor_boot.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "imxrt_flexspi_nor_boot.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

locate_data(".boot_hdr.ivt")
const struct ivt_s g_image_vector_table = {
	IVT_HEADER,                    /* IVT Header */
	IMAGE_ENTRY_ADDRESS,           /* Image Entry Function */
	IVT_RSVD,                      /* Reserved = 0 */
	(uint32_t) DCD_ADDRESS,        /* Address where DCD information is
                                        * stored */
	(uint32_t) BOOT_DATA_ADDRESS,  /* Address where BOOT Data Structure
                                        * is stored */
	(uint32_t) IMAG_VECTOR_TABLE,  /* Pointer to IVT Self (absolute
                                        * address */
	(uint32_t) CSF_ADDRESS,        /* Address where CSF file is stored */
	IVT_RSVD                       /* Reserved = 0 */
};

locate_data(".boot_hdr.boot_data")
const struct boot_data_s g_boot_data = {
	IMAGE_DEST,                      /* boot start location */
	(IMAGE_DEST_END - IMAGE_DEST),   /* size */
	PLUGIN_FLAG,                     /* Plugin flag */
	0xffffffff                       /* empty - extra data word */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
