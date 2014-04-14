/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 */

#include <board.hpp>

struct A
{
    A()
    {
        board::setStatusLed(true);
        board::setErrorLed(false);
    }

    ~A()
    {
        board::setStatusLed(false);
        board::setErrorLed(true);
    }
};

static A a;

static long long zero_global;

static long long non_zero_global = 123456789123456789;

static long long post_initialized_global;

__attribute__((constructor))
static void foo()
{
    post_initialized_global = 987654321987654321;
}

int main()
{
    static long long zero_local;

    while (zero_global != 0) { } // BSS init check
    while (zero_local != 0) { }
    while (non_zero_global != 123456789123456789) { } // Data init check
    while (post_initialized_global != 987654321987654321) { } // Constructor check

    while (true)
    {
        for (volatile int i = 0; i < 1000000; i++) { __asm volatile ("nop"); }
        A a;
        for (volatile int i = 0; i < 1000000; i++) { __asm volatile ("nop"); }
    }
}
