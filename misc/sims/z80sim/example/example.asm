;************************************************************************
; example.s
;************************************************************************

	.title	z80sim Test
	.module	example

;************************************************************************
; Constants
;************************************************************************

	STACKBASE		==    0xFFFF

;************************************************************************
; Data
;************************************************************************

	.area	DATA	(ABS,OVR)
	.org    0x8000
hello:
	.ascii "Hello, World!\n\0"

;************************************************************************
; Reset entry point
;************************************************************************

	.area	TEXT	(ABS,OVR)
	.org	0x0000
	di				; Disable interrupts
	ld	SP, #STACKBASE		; Set stack pointer
	im	1			; Set interrupt mode 1
	jp	start			; jump to start of program

;************************************************************************
; Interrupt handler
;************************************************************************

	.org   0x0038			; Int mode 1
	reti				; return from interrupt

;************************************************************************
; NMI interrupt handler
;************************************************************************

	.org   0x0066
	retn

;************************************************************************
; Start of program
;************************************************************************

	.org   0x0100
start:
	;ei				; Enable interrrupts
	ld	hl, #hello		; Say hello
	call   print 

forever:				; Then stop execution
	jp	forever

;******************************************************************
;        print    *
;        Funktion....: Sen tekst and data with serielport *
;        Input.......: hl points at text start adr   *
;        Output......: Text to serielport    *
;        uses........: a,hl*
;        call........: TX_BUSY         tst 28-4-1994 *
;******************************************************************

print:
	push   af
loop:
	ld	a, (hl)			; Get character to print
	cp	#0			; Null terminates the string
	jp	z, done

	out	(0xbe), a		; Send character
	inc    hl			; Increment to next character
	jp	loop			; Loop til done
done:
	pop    af
	ret


