/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

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

}
