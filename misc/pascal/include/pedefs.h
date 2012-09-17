/***********************************************************************
 * pedefs.h
 * Definitions of error codes
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 ***********************************************************************/

#ifndef __PEDEFS_H
#define __PEDEFS_H

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdint.h>

/***********************************************************************
 * Pre-processor Definitions
 ***********************************************************************/

/***********************************************************************
 * COMPILATION TIME ERRORS
 * eASSIGN              Expected ':='
 * eBEGIN               Expected 'BEGIN'
 * eCOLON               Expected ':'
 * eCOMMA               Expected ','
 * eCOUNT               Error count exceeded (FATAL)
 * eDO                  Expected 'DO'
 * eDUPFILE             Attempt to re-define file
 * eDUPSYM              Attempt to declare duplicate symbol
 * eEND                 Expected 'END'
 * eEQ                  Expected '='
 * eEXPONENT            Error in exponent of real constant
 * eFILE                Expected file identifier declared in PROGRAM
 * eHUH                 Internal error (FATAL)
 * eIDENT               Expected identifier
 * eIMPLEMENTATION      Expected implementation section in unit file
 * eINCLUDE             Include file OPEN error (FATAL)
 * eINTCONST            Expected integer constant
 * eINTERFACE           Expected interface section in unit file
 * eINTOVF              Integer overflow (or underflow)
 * eINTVAR              Integer variable name expected
 * eINVALIDFUNC         Unrecognized built-in function
 * eINVALIDPROC         Unrecognized built-in procedure
 * eINVARG              Invalid procedure/function argument type
 * eINVCONST            Invalid constant in declaration
 * eINVSIGNEDCONST      Invalid constant after sign
 * eINVFACTOR           Invalid factor
 * eINVFILE             Invalid file identifier (as declared in PROGRAM)
 * eINVLABEL            Invalid label
 * eINVPTR              Invalid pointer type
 * eINVTYPE             Invalid type identifier
 * eINVVARPARM          Invalid/unsupported VAR parameter type
 * eLBRACKET            Expected '['
 * eLPAREN              Expected '('
 * eMULTLABEL           Attempt to multiply define label
 * eNOSQUOTE            EOL encounter in a string, probable missing "'"
 * eNOTYET              Not implemented yet
 * eNPARMS              Number of parameters in function or procedure
 *                      call does not match number in declaration
 * eOF                  Expected 'OF'
 * eOVF                 Internal table overflow (FATAL)
 * ePERIOD              Expected '.'
 * ePOFFCONFUSION       Internal logic error in POFF file generation
 * ePOFFWRITEERROR      Error writing to POFF file
 * ePROGRAM             Expected 'PROGRAM'
 * ePTRADR              Expected pointer address (probably got value) form
 * ePTRVAL              Expected pointer value form, got value form
 * eRBRACKET            Expected ']'
 * eRPAREN              Expected ')'
 * eRPARENorCOMMA       Expected ')' or ','
 * eSEEKFAIL            Seek to file position failed
 * eSEMICOLON           Expected ';'
 * eSTRING              Expected a string
 * eTHEN                Expected 'THEN'
 * eTOorDOWNTO          Expected 'TO' or 'DOWNTO' in for statement
 * eTRUNC               String truncated
 * eUNDECLABEL          Attempt to define an undeclared label.
 * eUNDEFILE            Undefined file buffer
 * eUNDEFLABEL          A declared label was not defined.
 * eUNDEFSYM            Undefined symbol
 * eUNTIL               Expected 'UNTIL'
 * eEXPRTYPE	        Illegal expression type for this operation
 * eTERMTYPE            Illegal term type for this operation
 * eFACTORTYPE          Illegal factor type for this operation
 * eREADPARM            Illegal parameter in READ statement
 * eWRITEPARM           Illegal parameter in WRITE statement
 * eARRAYTYPE           Illegal type for ARRAY OF
 * ePOINTERTYPE         Illegal pointer type
 * eVARPARMTYPE         Illegal VAR parameter type
 * eSUBRANGE            Expected ".."
 * eSUBRANGETYPE	Illegal subrange type
 * eSET                 Expected valid/consistent type for SET OF
 * sSETRANGE		Value out of range for SET OF
 * eSCALARTYPE          Illegal scalar type
 * eBADSHORTINT         Short integer is out of range
 * eSYMTABINTERNAL      Internal error in symbol table
 * eRECORDDECLARE       Error in RECORD declaration
 * eRECORDOBJECT        Expected a field of RECORD
 * eRECORDVAR           Expected a RECORD variable in WITH
 * eUNIT                Expected UNIT at beginning of unit file
 * eUNITNAME            File does not contain the expected UNIT
 * eARGIGNORED          An argument was provided, but ignored
 *
 * LINK TIME ERRORS
 * eUNDEFINEDSYMBOL     A necessary symbol was not defined
 * eMULTIDEFSYMBOL      A symbol was defined multiple times
 *
 * ERRORS WHICH MAY OCCUR AT EITHER COMPILATION, LINK OR RUNTIME
 * eNOMEMORY            Memory allocation failed
 * ePOFFREADERROR       Error reading a POFF file
 * ePOFFBADFORMAT       The file format does not like valid POFF
 * eRCVDSIGNAL          Received SIGSEGV (or other) signal.
 *
 * RUN TIME ERRORS
 * eBADPC		Program Counter is out-of-range
 * eBADSP		Stack reference is out-of-range
 * eSTRSTKOVERFLOW      String stack overflow
 * eILLEGALOPCODE	Non-executable instruction found
 * eEXIT		oEND P-Code encountered
 * eBADSYSIOFUNC	Illegal SYSIO sub-function
 * eBADSYSLIBCALL	Illegal runtime library call
 * eBADFPOPCODE		Illegal FLOAT POINT op-code
 * eFAILEDLIBCALL       Runtime library call returned failure
 * eINTEGEROVERFLOW     Integer overflow
 **********************************************************************/

/* This is the error code that indicates that no error has occurred */

#define eNOERROR       ((uint16_t) 0x00)

/* This is the maximum number of errors that will be reported before
 * compilation is terminated.
 */

#define MAX_ERRORS 100

/* COMPILATION TIME ERRORS */

#define eASSIGN          ((uint16_t) 0x01)
#define eBEGIN           ((uint16_t) 0x02)
#define eCOLON           ((uint16_t) 0x03)
#define eCOMMA           ((uint16_t) 0x04)
#define eCOUNT           ((uint16_t) 0x05)
#define eDO              ((uint16_t) 0x06)
#define eDUPFILE         ((uint16_t) 0x07)
#define eDUPSYM          ((uint16_t) 0x08)
#define eEND             ((uint16_t) 0x09)
#define eEQ              ((uint16_t) 0x0a)
#define eEXPONENT        ((uint16_t) 0x0b)
#define eFILE            ((uint16_t) 0x0c)
#define eHUH             ((uint16_t) 0x0d)
#define eIDENT           ((uint16_t) 0x0e)
#define eIMPLEMENTATION  ((uint16_t) 0x0f)

#define eINCLUDE         ((uint16_t) 0x10)
#define eINTCONST        ((uint16_t) 0x11)
#define eINTERFACE       ((uint16_t) 0x12)
#define eINTOVF          ((uint16_t) 0x13)
#define eINTVAR          ((uint16_t) 0x14)
#define eINVALIDFUNC     ((uint16_t) 0x15)
#define eINVALIDPROC     ((uint16_t) 0x16)
#define eINVARG          ((uint16_t) 0x17)
#define eINVCONST        ((uint16_t) 0x18)
#define eINVSIGNEDCONST  ((uint16_t) 0x19)
#define eINVFACTOR       ((uint16_t) 0x1a)
#define eINVFILE         ((uint16_t) 0x1b)
#define eINVLABEL        ((uint16_t) 0x1c)
#define eINVPTR          ((uint16_t) 0x1d)
#define eINVTYPE         ((uint16_t) 0x1e)
#define eINVVARPARM      ((uint16_t) 0x1f)

#define eLBRACKET        ((uint16_t) 0x20)
#define eLPAREN          ((uint16_t) 0x21)
#define eMULTLABEL       ((uint16_t) 0x22)
#define eNOSQUOTE        ((uint16_t) 0x23)
#define eNOTYET          ((uint16_t) 0x24)
#define eNPARMS          ((uint16_t) 0x25)
#define eOF              ((uint16_t) 0x26)
#define eOVF             ((uint16_t) 0x27)
#define ePERIOD          ((uint16_t) 0x28)
#define ePOFFCONFUSION   ((uint16_t) 0x29)
#define ePOFFWRITEERROR  ((uint16_t) 0x2a)
#define ePROGRAM         ((uint16_t) 0x2b)
#define ePTRADR          ((uint16_t) 0x2c)
#define ePTRVAL          ((uint16_t) 0x2d)
#define eRBRACKET        ((uint16_t) 0x2e)
#define eRPAREN          ((uint16_t) 0x2f)

#define eRPARENorCOMMA   ((uint16_t) 0x30)
#define eSEEKFAIL        ((uint16_t) 0x31)
#define eSEMICOLON       ((uint16_t) 0x32)
#define eSTRING          ((uint16_t) 0x33)
#define eTHEN            ((uint16_t) 0x34)
#define eTOorDOWNTO      ((uint16_t) 0x35)
#define eTRUNC           ((uint16_t) 0x36)
#define eUNDECLABEL      ((uint16_t) 0x37)
#define eUNDEFILE        ((uint16_t) 0x38)
#define eUNDEFLABEL      ((uint16_t) 0x39)
#define eUNDEFSYM        ((uint16_t) 0x3a)
#define eUNTIL           ((uint16_t) 0x3b)
#define eEXPRTYPE        ((uint16_t) 0x3c)
#define eTERMTYPE        ((uint16_t) 0x3d)
#define eFACTORTYPE      ((uint16_t) 0x3e)
#define eREADPARM        ((uint16_t) 0x3f)

#define eWRITEPARM       ((uint16_t) 0x40)
#define eARRAYTYPE       ((uint16_t) 0x41)
#define ePOINTERTYPE     ((uint16_t) 0x42)
#define eVARPARMTYPE     ((uint16_t) 0x43)
#define eSUBRANGE        ((uint16_t) 0x44)
#define eSUBRANGETYPE    ((uint16_t) 0x45)
#define eSET             ((uint16_t) 0x46)
#define eSETRANGE        ((uint16_t) 0x47)
#define eSCALARTYPE      ((uint16_t) 0x48)
#define eBADSHORTINT     ((uint16_t) 0x49)
#define eSYMTABINTERNAL  ((uint16_t) 0x4a)
#define eRECORDDECLARE   ((uint16_t) 0x4b)
#define eRECORDOBJECT    ((uint16_t) 0x4c)
#define eRECORDVAR       ((uint16_t) 0x4d)
#define eUNIT            ((uint16_t) 0x4e)
#define eUNITNAME        ((uint16_t) 0x4f)

#define eARGIGNORED      ((uint16_t) 0x50)

/* LINK TIME ERRORS */

#define eUNDEFINEDSYMBOL ((uint16_t) 0x60)
#define eMULTIDEFSYMBOL  ((uint16_t) 0x61)

/* ERRORS WHICH MAY OCCUR AT EITHER COMPILATION OR RUNTIME */

#define eNOMEMORY        ((uint16_t) 0x70)
#define ePOFFREADERROR   ((uint16_t) 0x71)
#define ePOFFBADFORMAT   ((uint16_t) 0x72)
#define eRCVDSIGNAL      ((uint16_t) 0x73)

/* RUN TIME ERRORS */

#define eBADPC	         ((uint16_t) 0x80)
#define eBADSP	         ((uint16_t) 0x81)
#define eSTRSTKOVERFLOW  ((uint16_t) 0x82)
#define eILLEGALOPCODE   ((uint16_t) 0x83)
#define eEXIT            ((uint16_t) 0x84)
#define eBADSYSIOFUNC    ((uint16_t) 0x85)
#define eBADSYSLIBCALL   ((uint16_t) 0x86)
#define eBADFPOPCODE     ((uint16_t) 0x87)
#define eINTEGEROVERFLOW ((uint16_t) 0x89)
#define eFAILEDLIBCALL   ((uint16_t) 0x90)

#endif /* __PEDEFS_H */
