/***********************************************************************
 * builtins.h
 * Definitions of built-in function calls.
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

#ifndef __BUILTINS_H
#define __BUILTINS_H

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdint.h>

/***********************************************************************
 * Pre-processor Definitions
 ***********************************************************************/

#define MAX_BUILTIN_ARGS 3

#define ILLEGAL_BUILTIN_INIT \
    { NULL, 0, 0, { 0 }}

/* SYSIO built-ins ****************y*************************************/

/* boolean _xeof(fileno); */
#define xEOF_INIT \
    { "_xeof", sBOOLEAN_SIZE, 1, { sINT_SIZE }}

/* boolean _xeoln(fileno); */
#define xEOLN_INIT \
    { "_xeoln", sBOOLEAN_SIZE, 1, { sINT_SIZE }}

/* void _xreset(int fileno); */
#define xRESET_INIT \
    { "_xreset", 0, 1, { sINT_SIZE }}

/* void _xrewrite(int fileno); */
#define xREWRITE_INIT \
    { "_xrewrite", 0, 1, { sINT_SIZE }}

/* void _xreadln(int fileno); */
#define xREADLN_INIT \
    { "_xreadln", 0, 1, { sINT_SIZE }}

#define xREAD_PAGE_INIT ILLEGAL_BUILTIN_INIT

/* void _xread_binary(int fileno, char *buffer, int buffersize); */
#define xREAD_BINARY_INIT \
    { "_xread_binary", 0, 3, { sINT_SIZE, sPTR_SIZE, sINT_SIZE }}

/* void _xread_int(int fileno, int *value); */
#define xREAD_INT_INIT \
    { "_xread_int", 0, 2, { sINT_SIZE, sPTR_SIZE }}

/* void _xread_char(int fileno, char *value); */
#define xREAD_CHAR_INIT \
    { "_xread_char", 0, 2, { sINT_SIZE, sPTR_SIZE }}

/* void _xread_string(int fileno, char *string, int stringsize); */
#define xREAD_STRING_INIT \
    { "_xread_string", 0, 3, { sINT_SIZE, sPTR_SIZE, sINT_SIZE }}

/* void _xread_real(int fileno, real *value); */
#define xREAD_REAL_INIT \
    { "_xread_real", 0, 2, { sINT_SIZE, sPTR_SIZE }}

/* void _xwriteln(int fileno); */
#define xWRITELN_INIT \
    { "_xwriteln", 0, 1, { sINT_SIZE }}

/* void _xwrite_page(int fileno); */
#define xWRITE_PAGE_INIT \
    { "_xwrite_page", 0, 1, { sINT_SIZE }}

/* void _write_binary(int fileno, char *buffer, int buffersize); */
#define xWRITE_BINARY_INIT \
    { "_xwrite_binary", 0, 3, { sINT_SIZE, sPTR_SIZE, sINT_SIZE }}

/* void _write_int(int fileno, int value); */
#define xWRITE_INT_INIT \
    { "_xwrite_int", 0, 2, { sINT_SIZE, sINT_SIZE }}

/* void _write_char(int fileno, int value); */
#define xWRITE_CHAR_INIT \
    { "_xwrite_char", 0, 2, { sINT_SIZE, sINT_SIZE }}

/* void _write_string(int fileno, char *string, int stringsize); */
#define xWRITE_STRING_INIT \
    { "_xwrite_string", 0, 3, { sINT_SIZE, sPTR_SIZE, sINT_SIZE }}

/* void _write_real(int fileno, real value); */
#define xWRITE_REAL_INIT \
    { "_xwrite_real", 0, 2, { sINT_SIZE, sREAL_SIZE }}

/* Runtime library built-ins *******************************************/
/* string _lbgetenv(string *name); */
#define lbGETENV_INIT \
    { "_lbgetenv", sRSTRING_SIZE, 1, { sRSTRING_SIZE }}

/* void _lbstr2str(string src, string dest); */
#define lbSTR2STR_INIT \
    { "_lbstgr2str", 0, 2, { sRSTRING_SIZE, sRSTRING_SIZE }}

/* void _lbcstr2str(string src, string dest); */
#define lbCSTR2STR_INIT \
    { "_lbcstr2str", 0, 2, { sRSTRING_SIZE, sRSTRING_SIZE }}

/* void _libstr2rstr(string src, string dest); */
#define lbSTR2RSTR_INIT \
    { "_lbstr2rstr", 0, 2, { sRSTRING_SIZE, sRSTRING_SIZE }}

/* void _lbcstr2str(string src, string dest); */
#define lbCSTR2RSTR_INIT \
    { "_lbcstr2rstr", 0, 2, { sRSTRING_SIZE, sRSTRING_SIZE }}

/* void _lbval(const string s, int *v, int *code); */
#define lbVAL_INIT \
    { "_lbval", 0, 2, { sRSTRING_SIZE, sPTR_SIZE, sPTR_SIZE }}

/* string _lbmkstk(void); */
#define lbMKSTK_INIT \
    { "_lbmkstk",  sRSTRING_SIZE, 0, { 0 }}

/* string mkstkstr(string name); */
#define lbMKSTKSTR_INIT \
    { "_lbmkstkstr",  sRSTRING_SIZE, 1, { sRSTRING_SIZE }}

/* string _lbmkstkc(int c); */
#define lbMKSTKC_INIT \
    { "_lbmkstkc",  sRSTRING_SIZE, 1, { sINT_SIZE }}

/* string _lbstgrcat(string name, int c); */
#define lbSTRCAT_INIT \
    { "_lbstrcat",  sRSTRING_SIZE, 2, { sRSTRING_SIZE, sINT_SIZE }}

/* string _lbstrcatc(string name, int c); */
#define lbSTRCATC_INIT \
    { "_lbstrcatc",  sRSTRING_SIZE, 2, { sRSTRING_SIZE, sINT_SIZE }}

/* int _lbstrcmp(string name1, string name2); */
#define lbSTRCMP_INIT \
    { "_lbstrcmp", sINT_SIZE, 2, { sRSTRING_SIZE, sRSTRING_SIZE }}

/* Floating point library built-ins ************************************/

/* real _fpfloat(int v); */
#define fpFLOAT_INIT \
    { "_fpfloat", sREAL_SIZE, 1, { sINT_SIZE }}

/* int _fptrunc(real v); */
#define fpTRUNC_INIT \
    { "_fptrunc", sINT_SIZE, 1, { sREAL_SIZE }}

/* int _fpround(real v); */
#define fpROUND_INIT \
    { "_fpround", sINT_SIZE, 1, { sREAL_SIZE }}

/* real _fpadd_rr(real a, real b); */
#define fpADD_RR_INIT \
    { "_fpadd_rr", sREAL_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* real _fpadd_ri(int a, real b); */
#define fpADD_RI_INIT \
    { "_fpadd_ri", sREAL_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* real _fpadd_ir(real a, int b); */
#define fpADD_IR_INIT \
    { "_fpadd_ir", sREAL_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* real _fpadd_ii(int a, int b); */
#define fpADD_II_INIT \
    { "_fpadd_ii", sREAL_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* real _fpsub_rr(real a, real b); */
#define fpSUB_RR_INIT \
    { "_fpsub_rr", sREAL_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* real _fpsub_ri(int a, real b); */
#define fpSUB_RI_INIT \
    { "_fpsub_ri", sREAL_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* real _fpsub_ir(real a, int b); */
#define fpSUB_IR_INIT \
    { "_fpsub_ir", sREAL_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* real _fpsub_ii(int a, int b); */
#define fpSUB_II_INIT \
    { "_fpsub_ii", sREAL_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* real _fpmul_rr(real a, real b); */
#define fpMUL_RR_INIT \
    { "_fpmul_rr", sREAL_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* real _fpmul_ri(int a, real b); */
#define fpMUL_RI_INIT \
    { "_fpmul_ri", sREAL_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* real _fpmul_ir(real a, int b); */
#define fpMUL_IR_INIT \
    { "_fpmul_ir",sREAL_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* real _fpmul_ii(int a, int b); */
#define fpMUL_II_INIT \
    { "_fpmul_ii", sREAL_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* real _fpdiv_rr(real a, real b); */
#define fpDIV_RR_INIT \
    { "_fpdiv_rr", sREAL_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* real _fpdiv_ri(int a, real b); */
#define fpDIV_RI_INIT \
    { "_fpdiv_ri",sREAL_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* real _fpdiv_ir(real a, int b); */
#define fpDIV_IR_INIT \
    { "_fpdiv_ir",sREAL_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* real _fpdiv_ii(int a, int b); */
#define fpDIV_II_INIT \
    { "_fpdiv_ii",sREAL_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* real _fpmod_rr(real a, real b); */
#define fpMOD_RR_INIT \
    { "_fpmod_rr", sREAL_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* real _fpmod_ri(int a, real b); */
#define fpMOD_RI_INIT \
    { "_fpmod_ri", sREAL_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* real _fpmod_ir(real a, int b); */
#define fpMOD_IR_INIT \
    { "_fpmod_ir", sREAL_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* real _fpmod_ii(int a, int b); */
#define fpMOD_II_INIT \
    { "_fpmod_ii", sREAL_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* boolean _fpequ_rr(real a, real b); */
#define fpEQU_RR_INIT \
    { "_fpequ_rr", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* boolean _fpequ_ri(int a, real b); */
#define fpEQU_RI_INIT \
    { "_fpequ_ri", sBOOLEAN_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* boolean _fpequ_ir(real a, int b); */
#define fpEQU_IR_INIT \
    { "_fpequ_ir", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* boolean _fpequ_ii(int a, int b); */
#define fpEQU_II_INIT \
    { "_fpequ_ii", sBOOLEAN_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* boolean _fpneq_rr(real a, real b); */
#define fpNEQ_RR_INIT \
    { "_fpneq_rr", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* boolean _fpneq_ri(int a, real b); */
#define fpNEQ_RI_INIT \
    { "_fpneq_ri", sBOOLEAN_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* boolean _fpneq_ir(real a, int b); */
#define fpNEQ_IR_INIT \
    { "_fpneq_ir", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* boolean _fpneq_ii(int a, int b); */
#define fpNEQ_II_INIT \
    { "_fpneq_ii", sBOOLEAN_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* boolean _fplt_rr(real a, real b); */
#define fpLT_RR_INIT \
    { "_fplt_rr", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* boolean _fplt_ri(int a, real b); */
#define fpLT_RI_INIT \
    { "_fplt_ri", sBOOLEAN_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* boolean _fplt_ir(real a, int b); */
#define fpLT_IR_INIT \
    { "_fplt_ir", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* boolean _fplt_ii(int a, int b); */
#define fpLT_II_INIT \
    { "_fplt_ii", sBOOLEAN_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* boolean _fpgte_rr(real a, real b); */
#define fpGTE_RR_INIT \
    { "_fpgte_rr", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* boolean _fpgte_ri(int a, real b); */
#define fpGTE_RI_INIT \
    { "_fpgte_ri", sBOOLEAN_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* boolean _fpgte_ir(real a, int b); */
#define fpGTE_IR_INIT \
    { "_fpgte_ir", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* boolean _fpgte_ii(int a, int b); */
#define fpGTE_II_INIT \
    { "_fpgte_ii", sBOOLEAN_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* boolean _fpgt_rr(real a, real b); */
#define fpGT_RR_INIT \
    { "_fpgt_rr", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* boolean _fpgt_ri(int a, real b); */
#define fpGT_RI_INIT \
    { "_fpgt_ri", sBOOLEAN_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* boolean _fpgt_ir(real a, int b); */
#define fpGT_IR_INIT \
    { "_fpgt_ir", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* boolean _fpgt_ii(int a, int b); */
#define fpGT_II_INIT \
    { "_fpgt_ii", sBOOLEAN_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* boolean _fplte_rr(real a, real b); */
#define fpLTE_RR_INIT \
    { "_fplte_rr", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sREAL_SIZE }}

/* boolean _fplte_ri(int a, real b); */
#define fpLTE_RI_INIT \
    { "_fplte_ri", sBOOLEAN_SIZE, 2, { sINT_SIZE, sREAL_SIZE }}

/* boolean _fplte_ir(real a, int b); */
#define fpLTE_IR_INIT \
    { "_fplte_ir", sBOOLEAN_SIZE, 2, { sREAL_SIZE, sINT_SIZE }}

/* boolean _fplte_ii(int a, int b); */
#define fpLTE_II_INIT \
    { "_fplte_ii", sBOOLEAN_SIZE, 2, { sINT_SIZE, sINT_SIZE }}

/* real _fpneg_r(real v); */
#define fpNEG_R_INIT \
    { "_fpneg_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpneg_i(int v); */
#define fpNEG_I_INIT \
    { "_fpneg_i", sREAL_SIZE, 1, { sINT_SIZE }}

/* real _fpabs_r(real v); */
#define fpABS_R_INIT \
    { "_fpabs_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpabs_i(int v); */
#define fpABS_I_INIT \
    { "_fpabs_i", sREAL_SIZE, 1, { sINT_SIZE }}

/* real _fpsqr_r(real v); */
#define fpSQR_R_INIT \
    { "_fpsqr_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpsqr_i(int v); */
#define fpSQR_I_INIT \
    { "_fpsqr_i", sREAL_SIZE, 1, { sINT_SIZE }}

/* real _fpsqrt_r(real v); */
#define fpSQRT_R_INIT \
    { "_fpsqrt_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpsqrt_i(int v); */
#define fpSQRT_I_INIT \
    { "_fpsqrt_i", sREAL_SIZE, 1, { sINT_SIZE }}

/* real _fpsin_r(real v); */
#define fpSIN_R_INIT \
    { "_fpsin_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpsin_i(int v); */
#define fpSIN_I_INIT \
    { "_fpsin_i", sREAL_SIZE, 1, { sINT_SIZE }}

/* real _fpcos_r(real v); */
#define fpCOS_R_INIT \
    { "_fpcos_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpcos_i(int v); */
#define fpCOS_I_INIT \
    { "_fpcos_i", sREAL_SIZE, 1, { sINT_SIZE }}

/* real _fpatan_r(real v); */
#define fpATAN_R_INIT \
    { "_fpatan_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpatan_i(int v); */
#define fpATAN_I_INIT \
    { "_fpatan_i", sREAL_SIZE, 1, { sINT_SIZE }}

/* real _fpln_r(real v); */
#define fpLN_R_INIT \
    { "_fpln_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpln_i(int v); */
#define fpLN_I_INIT \
    { "_fpln_i", sREAL_SIZE, 1, { sINT_SIZE }}

/* real _fpexp_r(real v); */
#define fpEXP_R_INIT \
    { "_fpexpr_r", sREAL_SIZE, 1, { sREAL_SIZE }}

/* real _fpexp_i(int v); */
#define fpEXP_I_INIT \
    { "_fpexpr_i", sREAL_SIZE, 1, { sINT_SIZE }}

/***********************************************************************
 * Public Types
 ***********************************************************************/

struct regm_builtin_s
{
  const char *szName;
  uint32_t    wRetSize;
  uint32_t    nParms;
  uint32_t    wArgSize[MAX_BUILTIN_ARGS];
};

#endif /* __BUILTINS_H */
