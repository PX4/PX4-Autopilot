/* Place a branch to the real head at the entry point */
.section .text.start
	b	__start


/* Exception Vectors like they are needed for the exception vector
   indirection of the internal boot ROM.  The following section must
   be liked to appear at 0x80001c */
.section .text.exceptions
_undef_instr:
	b	up_vectorundefinsn
_sw_interr:
	b	up_vectorswi
_prefetch_abort:
	b	up_vectorprefetch
_data_abort:
	b	up_vectordata
_reserved:
	b	_reserved
_irq:
	b	up_vectorirq
_fiq:
	b	up_vectorfiq
