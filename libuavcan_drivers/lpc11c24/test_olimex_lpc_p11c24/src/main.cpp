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

int main()
{
    while (true)
    {
        for (volatile int i = 0; i < 1000000; i++) { __asm volatile ("nop"); }
        A a;
        for (volatile int i = 0; i < 1000000; i++) { __asm volatile ("nop"); }
    }
}
