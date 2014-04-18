/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/impl_constants.hpp>

namespace uavcan
{
/**
 * Common error codes.
 * Functions that return signed integers may also return inverted error codes,
 * i.e. returned value should be inverted back to get the actual error code.
 */
enum
{
    ErrOk,
    ErrFailure,
    ErrInvalidParam,
    ErrMemory,
    ErrDriver,
    ErrUnknownDataType,
    ErrInvalidMarshalData,
    ErrInvalidTransferListener,
    ErrNotInited,
    ErrRecursiveCall,
    ErrLogic
};

/**
 * Fatal error handler.
 * Throws std::runtime_error() if exceptions are available, otherwise calls assert(0) then std::abort().
 */
#if __GNUC__
__attribute__ ((noreturn))
#endif
UAVCAN_EXPORT
// coverity[+kill]
void handleFatalError(const char* msg);

}
