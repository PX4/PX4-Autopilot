/* linenoise.h -- VERSION 1.0
 *
 * Guerrilla line editing library against the idea that a line editing lib
 * needs to be 20,000 lines of C code.
 *
 * See linenoise.c for more information.
 *
 * ------------------------------------------------------------------------
 *
 * Copyright (c) 2010-2023, Salvatore Sanfilippo <antirez at gmail dot com>
 * Copyright (c) 2010-2013, Pieter Noordhuis <pcnoordhuis at gmail dot com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LINENOISE_H
#define __LINENOISE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h> /* For size_t. */

extern char *linenoiseEditMore;

typedef struct linenoiseState_s* linenoiseState;
typedef struct linenoiseCompletions_s* linenoiseCompletions;

struct linenoiseConfig {
    int fd_in;      /* linenoise will read from this file descriptor */
    int fd_out;     /* linenoise will write to this file descriptor */
    int fd_tty;     /* linenoise will detect and configure this fd as console, leave -1 if not used */
    char *buf;      /* user provided line buffer, storing any unfinished user input */
    size_t buf_len; /* size of above buffer */
};

/* Non blocking API. */
void linenoiseCreateState(linenoiseState *l, const struct linenoiseConfig *cfg);
void linenoiseDeleteState(linenoiseState l);
int linenoiseEditStart(struct linenoiseState_s *l, const char *prompt);
char *linenoiseEditFeed(linenoiseState l);
void linenoiseEditStop(linenoiseState l);
void linenoiseHide(linenoiseState l);
void linenoiseShow(linenoiseState l);

/* Completion API. */
typedef void(linenoiseCompletionCallback)(const char *, linenoiseCompletions);
typedef char*(linenoiseHintsCallback)(const char *, int *color, int *bold);
typedef void(linenoiseFreeHintsCallback)(void *);
void linenoiseSetCompletionCallback(linenoiseState l, linenoiseCompletionCallback *);
void linenoiseSetHintsCallback(linenoiseState l, linenoiseHintsCallback *);
void linenoiseSetFreeHintsCallback(linenoiseState l, linenoiseFreeHintsCallback *);
void linenoiseAddCompletion(linenoiseCompletions, const char *);

/* History API. */
int linenoiseHistoryAdd(linenoiseState l, const char *line);
int linenoiseHistorySetMaxLen(linenoiseState l, int len);
int linenoiseHistorySave(linenoiseState l, const char *filename);
int linenoiseHistoryLoad(linenoiseState l, const char *filename);

/* Other utilities. */
void linenoiseClearScreen(linenoiseState l);
void linenoiseSetMultiLine(linenoiseState l, int ml);
void linenoisePrintKeyCodes(linenoiseState l);
void linenoiseMaskModeEnable(linenoiseState l);
void linenoiseMaskModeDisable(linenoiseState l);

#ifdef __cplusplus
}
#endif

#endif /* __LINENOISE_H */
