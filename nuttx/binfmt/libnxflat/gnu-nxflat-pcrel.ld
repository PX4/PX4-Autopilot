/****************************************************************************
 * examples/nxflat/gnu-nxflat-pcrel.ld
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

MEMORY 
{
  ISPACE : ORIGIN = 0x0, LENGTH = 2097152
  DSPACE : ORIGIN = 0x0, LENGTH = 2097152
}

/****************************************************************************
 * The XFLAT program image is divided into two segments:
 *
 * (1) ISpace (Instruction Space).  This is the segment that contains
 *     code (.text) as well as read-only data (.rodata).  Everything in the
 *     segment should be fetch-able  machine PC instructions (jump, branch,
 *     call, etc.) or PC-relative loads.
 * (2) DSpace (Data Space).  This is the segment that contains read-write
 *     data (.data, .bss). Everything in this segment should be access-able
 *     with machine PIC load and store instructions.
 *
 * Older versions of GCC (at least up to GCC 4.3.3), use GOT-relative
 * addressing to access RO data. In that case, read-only data (.rodata) must
 * reside in D-Space and this linker script should NOT be used with those
 * older tools.
 *
 * Newer versions of GCC (at least as of GCC 4.6.3), use PC-relative
 * addressing to access RO data. In that case, read-only data (.rodata) must
 * reside in I-Space and this linker script should be used.
 *
 ****************************************************************************/

SECTIONS
{
  .text 0x00000000 :
    {
      /* ISpace is located at address 0.  Every (unrelocated) ISpace
       * address is an offset from the begining of this segment.
       */

      text_start = . ;

      *(.text)
      *(.text.*)
      *(.gnu.warning)
      *(.stub)
      *(.glue_7)
      *(.glue_7t)
      *(.jcr)

      /* C++ support:  The .init and .fini sections contain XFLAT-
       * specific logic to manage static constructors and destructors.
       */

      *(.gnu.linkonce.t.*)
      *(.init)
      *(.fini)

      /* This is special code area at the end of the normal
         text section.  It contains a small lookup table at
         the start followed by the code pointed to by entries
         in the lookup table.  */

      . = ALIGN (4) ;
      PROVIDE(__ctbp = .);
      *(.call_table_data)
      *(.call_table_text)

      /* In this model, .rodata is access using PC-relative addressing
       * and, hence, must also reside in the .text section.
       */

      *(.rodata)
      *(.rodata1)
      *(.rodata.*)
      *(.gnu.linkonce.r*)

      _etext = . ;

    } > ISPACE

    /* DSpace is also located at address 0.  Every (unrelocated) DSpace
     * address is an offset from the begining of this segment.
     */

  .data 0x00000000 :
    {
      __data_start = . ;
      *(.data)
      *(.data1)
      *(.data.*)
      *(.gnu.linkonce.d*)
      *(.data1)
      *(.eh_frame)
      *(.gcc_except_table)

      *(.gnu.linkonce.s.*)
      *(__libc_atexit)
      *(__libc_subinit)
      *(__libc_subfreeres)
      *(.note.ABI-tag)

      /* C++ support. For each global and static local C++ object,
       * GCC creates a small subroutine to construct the object. Pointers
       * to these routines (not the routines themselves) are stored as
       * simple, linear arrays in the .ctors section of the object file.
       * Similarly, pointers to global/static destructor routines are
       * stored in .dtors.
       */

      *(.gnu.linkonce.d.*)

      _ctors_start = . ;
      *(.ctors)
      _ctors_end = . ;
      _dtors_start = . ;
      *(.dtors)
      _dtors_end = . ;

      _edata = . ;
      edata = ALIGN( 0x10 ) ;
    } > DSPACE

  .bss :
    {
      __bss_start = _edata ;
      *(.dynsbss)
      *(.sbss)
      *(.sbss.*)
      *(.scommon)
      *(.dynbss)
      *(.bss)
      *(.bss.*)
      *(.bss*)
      *(.gnu.linkonce.b*)
      *(COMMON)
      end = ALIGN( 0x10 ) ;
      _end = ALIGN( 0x10 ) ;
    } > DSPACE

    .got 0 : { *(.got.plt) *(.got) }
    .junk 0 : { *(.rel*) *(.rela*) }
    /* Stabs debugging sections.    */
    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
