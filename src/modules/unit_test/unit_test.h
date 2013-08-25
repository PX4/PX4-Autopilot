/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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
 * @file unit_test.h
 * A unit test library based on MinUnit (http://www.jera.com/techinfo/jtns/jtn002.html).
 *
 */

#ifndef UNIT_TEST_H_
#define UNIT_TEST_

#include <systemlib/err.h>


class __EXPORT UnitTest
{

public:
#define xstr(s) str(s)
#define str(s) #s
#define INLINE_GLOBAL(type,func) inline type& func() { static type x; return x; }

INLINE_GLOBAL(int, mu_tests_run)
INLINE_GLOBAL(int, mu_assertion)
INLINE_GLOBAL(int, mu_line)
INLINE_GLOBAL(const char*, mu_last_test)

#define mu_assert(message, test)                                           \
    do                                                                     \
    {                                                                      \
        if (!(test))                                                       \
            return __FILE__ ":" xstr(__LINE__) " " message " (" #test ")"; \
        else                                                               \
            mu_assertion()++;                                              \
    } while (0)


#define mu_run_test(test)                                                  \
do                                                                         \
{                                                                          \
    const char *message;                                                   \
    mu_last_test() = #test;                                                \
    mu_line() = __LINE__;                                                  \
    message = test();                                                      \
    mu_tests_run()++;                                                      \
    if (message)                                                           \
        return message;                                                    \
} while (0)


public:
	UnitTest();
    virtual ~UnitTest();

    virtual const char*     run_tests() = 0;
    virtual void            print_results(const char* result);
};



#endif /* UNIT_TEST_H_ */
