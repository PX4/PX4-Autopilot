/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdexcept>

#ifndef STRINGIZE
#  define STRINGIZE2(x)   #x
#  define STRINGIZE(x)    STRINGIZE2(x)
#endif
#define ENFORCE(x) if (!(x)) { throw std::runtime_error(__FILE__ ":" STRINGIZE(__LINE__) ": " #x); }

