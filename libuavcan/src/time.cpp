/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/time.hpp>
#include <uavcan/Timestamp.hpp>

namespace uavcan
{
/*
 * UtcTime
 */
UtcTime::UtcTime(const Timestamp& ts)  // Implicit
{
    operator=(ts);
}

UtcTime& UtcTime::operator=(const Timestamp& ts)
{
    *this = fromUSec(ts.husec * Timestamp::USEC_PER_LSB);
    return *this;
}

UtcTime::operator Timestamp() const
{
    Timestamp ts;
    ts.husec = toUSec() / Timestamp::USEC_PER_LSB;
    return ts;
}

}
