/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/impl_constants.hpp>
#include <uavcan/stdint.hpp>

namespace uavcan
{
namespace
{
/**
 * Common error codes.
 * Functions that return signed integers may also return inverted error codes,
 * i.e. returned value should be inverted back to get the actual error code.
 */
const int16_t ErrOk                      = 0;
const int16_t ErrFailure                 = 1;
const int16_t ErrInvalidParam            = 2;
const int16_t ErrMemory                  = 3;
const int16_t ErrDriver                  = 4;
const int16_t ErrUnknownDataType         = 5;
const int16_t ErrInvalidMarshalData      = 6;
const int16_t ErrInvalidTransferListener = 7;
const int16_t ErrNotInited               = 8;
const int16_t ErrRecursiveCall           = 9;
const int16_t ErrLogic                   = 10;

}

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
