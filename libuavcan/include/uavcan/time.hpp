/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <algorithm>
#include <limits>
#include <sstream>
#include <cstdio>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/Timestamp.hpp>

namespace uavcan
{

template <typename D>
class DurationBase
{
    int64_t usec_;

public:
    DurationBase()
    : usec_(0)
    {
        StaticAssert<(sizeof(D) == 8)>::check();
    }

    static D fromUSec(int64_t us)
    {
        D d;
        d.usec_ = us;
        return d;
    }
    static D fromMSec(int64_t ms) { return fromUSec(ms * 1000); }

    int64_t toUSec() const { return usec_; }
    int64_t toMSec() const { return usec_ / 1000; }

    D getAbs() const { return D::fromUSec(std::abs(usec_)); }

    bool isPositive() const { return usec_ > 0; }
    bool isNegative() const { return usec_ < 0; }
    bool isZero() const { return usec_ == 0; }

    bool operator==(const D& r) const { return usec_ == r.usec_; }
    bool operator!=(const D& r) const { return !operator==(r); }

    bool operator<(const D& r) const { return usec_ < r.usec_; }
    bool operator>(const D& r) const { return usec_ > r.usec_; }
    bool operator<=(const D& r) const { return usec_ <= r.usec_; }
    bool operator>=(const D& r) const { return usec_ >= r.usec_; }

    D operator+(const D &r) const { return fromUSec(usec_ + r.usec_); } // TODO: overflow check
    D operator-(const D &r) const { return fromUSec(usec_ - r.usec_); } // ditto

    D operator-() const { return fromUSec(-usec_); }

    D& operator+=(const D &r)
    {
        *this = *this + r;
        return *static_cast<D*>(this);
    }
    D& operator-=(const D &r)
    {
        *this = *this - r;
        return *static_cast<D*>(this);
    }

    template <typename Scale>
    D operator*(Scale scale)   const { return fromUSec(usec_ * scale); }

    template <typename Scale>
    D& operator*=(Scale scale)
    {
        *this = *this * scale;
        return *static_cast<D*>(this);
    }

    std::string toString() const;
};


template <typename T, typename D>
class TimeBase
{
    uint64_t usec_;

public:
    TimeBase()
    : usec_(0)
    {
        StaticAssert<(sizeof(T) == 8)>::check();
        StaticAssert<(sizeof(D) == 8)>::check();
    }

    static T fromUSec(uint64_t us)
    {
        T d;
        d.usec_ = us;
        return d;
    }
    static T fromMSec(uint64_t ms) { return fromUSec(ms * 1000); }

    uint64_t toUSec() const { return usec_; }
    uint64_t toMSec() const { return usec_ / 1000; }

    bool isZero() const { return usec_ == 0; }

    bool operator==(const T& r) const { return usec_ == r.usec_; }
    bool operator!=(const T& r) const { return !operator==(r); }

    bool operator<(const T& r) const { return usec_ < r.usec_; }
    bool operator>(const T& r) const { return usec_ > r.usec_; }
    bool operator<=(const T& r) const { return usec_ <= r.usec_; }
    bool operator>=(const T& r) const { return usec_ >= r.usec_; }

    T operator+(const D& r) const
    {
        if (r.isNegative())
        {
            if (uint64_t(r.getAbs().usec_) > usec_)
                return fromUSec(0);
        }
        else
        {
            if (uint64_t(usec_ + r.usec_) < usec_)
                return fromUSec(std::numeric_limits<uint64_t>::max());
        }
        return fromUSec(usec_ + r.usec_);
    }

    T operator-(const D& r) const
    {
        return *static_cast<const T*>(this) + (-r);
    }
    D operator-(const T& r) const
    {
        return D::fromUSec((usec_ > r.usec_) ? (usec_ - r.usec_) : -(r.usec_ - usec_));
    }

    T& operator+=(const D& r)
    {
        *this = *this + r;
        return *static_cast<T*>(this);
    }
    T& operator-=(const D& r)
    {
        *this = *this - r;
        return *static_cast<T*>(this);
    }

    std::string toString() const;
};


class MonotonicDuration : public DurationBase<MonotonicDuration> { };

class MonotonicTime : public TimeBase<MonotonicTime, MonotonicDuration> { };


class UtcDuration : public DurationBase<UtcDuration> { };

class UtcTime : public TimeBase<UtcTime, UtcDuration>
{
public:
    UtcTime() { }

    UtcTime(const Timestamp& ts)  // Implicit
    {
        operator=(ts);
    }

    UtcTime& operator=(const Timestamp& ts)
    {
        *this = fromUSec(ts.husec * Timestamp::USEC_PER_LSB);
        return *this;
    }

    operator Timestamp() const
    {
        Timestamp ts;
        ts.husec = toUSec() / Timestamp::USEC_PER_LSB;
        return ts;
    }
};


template <typename Stream, typename D>
inline Stream& operator<<(Stream& s, DurationBase<D> d)
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%06lu", static_cast<unsigned long>(std::abs(d.toUSec() % 1000000L)));
    if (d.isNegative())
        s << '-';
    s << std::abs(d.toUSec() / 1000000L) << '.' << buf;
    return s;
}

template <typename Stream, typename T, typename D>
inline Stream& operator<<(Stream& s, TimeBase<T, D> t)
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%06lu", static_cast<unsigned long>(t.toUSec() % 1000000L));
    s << (t.toUSec() / 1000000L) << '.' << buf;
    return s;
}


template <typename D>
inline std::string DurationBase<D>::toString() const
{
    std::ostringstream os;
    os << *static_cast<const D*>(this);
    return os.str();
}

template <typename T, typename D>
inline std::string TimeBase<T, D>::toString() const
{
    std::ostringstream os;
    os << *static_cast<const T*>(this);
    return os.str();
}

}
