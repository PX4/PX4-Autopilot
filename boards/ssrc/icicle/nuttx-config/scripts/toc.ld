/****************************************************************************
 * boards/risc-v/icicle/mpfs/scripts/ld.script
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

MEMORY
{
  progmem (r) : ORIGIN = 0xACA00000, LENGTH = 6M - 64
}

OUTPUT_ARCH("riscv")

EXTERN(_main_toc)

SECTIONS
{
    .toc : {
        /* The ToC */
        _toc_start = ABSOLUTE(.);
        KEEP(*(.main_toc));
        /* Padd the rest */
        FILL(0xff);
        . = ALIGN(8);
        _toc_end = ABSOLUTE(.);
    } > progmem

    /* Start of the ToC signature, appended directly after ToC */
    PROVIDE(_toc_signature = ALIGN(4));

    .toc_sig (NOLOAD) : {
        /* Make a hole for the singature */
        KEEP(*(.main_toc_sig));
    } > progmem

    .app (NOLOAD) : {
        /* The application firmware payload */
        _app_start = ABSOLUTE(.);
        *(.firmware)
        . = ALIGN(4);
        _app_end = ABSOLUTE(.);
    } > progmem

    /* Start of the payload signature. This has to be in the end of the
     * payload and aligned to a 4 byte boundary
     */
    PROVIDE(_boot_signature = ALIGN(4));
}
