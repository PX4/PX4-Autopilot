/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

namespace uavcan
{

/**
 * Fatal error handler.
 * Throws std::runtime_error() if exceptions are available, otherwise calls assert(0) then std::abort().
 */
#if __GNUC__
__attribute__ ((noreturn))
#endif
void handleFatalError(const char* msg);

}
