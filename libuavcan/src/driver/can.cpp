/*
 * CAN bus driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <cassert>
#include <uavcan/driver/can.hpp>

namespace uavcan
{

const uint32_t CanFrame::MaskStdID;
const uint32_t CanFrame::MaskExtID;
const uint32_t CanFrame::FlagEFF;
const uint32_t CanFrame::FlagRTR;
const uint32_t CanFrame::FlagERR;


std::string CanFrame::toString(StringRepresentation mode) const
{
    using std::snprintf;

    assert(mode == StrTight || mode == StrAligned);

    static const int ASCII_COLUMN_OFFSET = 36;

    char buf[50];
    char* wpos = buf;
    char* const epos = buf + sizeof(buf);
    std::fill(buf, buf + sizeof(buf), 0);

    if (id & FlagEFF)
    {
        wpos += snprintf(wpos, epos - wpos, "0x%08x  ", unsigned(id & MaskExtID));
    }
    else
    {
        const char* const fmt = (mode == StrAligned) ? "     0x%03x  " : "0x%03x  ";
        wpos += snprintf(wpos, epos - wpos, fmt, unsigned(id & MaskStdID));
    }

    if (id & FlagRTR)
    {
        wpos += snprintf(wpos, epos - wpos, " RTR");
    }
    else if (id & FlagERR)
    {
        // TODO: print error flags
        wpos += snprintf(wpos, epos - wpos, " ERR");
    }
    else
    {
        for (int dlen = 0; dlen < dlc; dlen++)                                 // hex bytes
        {
            wpos += snprintf(wpos, epos - wpos, " %02x", unsigned(data[dlen]));
        }

        while (mode == StrAligned && wpos < buf + ASCII_COLUMN_OFFSET)        // alignment
        {
            *wpos++ = ' ';
        }

        wpos += snprintf(wpos, epos - wpos, "  \'");                           // ascii
        for (int dlen = 0; dlen < dlc; dlen++)
        {
            uint8_t ch = data[dlen];
            if (ch < 0x20 || ch > 0x7E)
            {
                ch = '.';
            }
            wpos += snprintf(wpos, epos - wpos, "%c", ch);
        }
        wpos += snprintf(wpos, epos - wpos, "\'");
    }
    return std::string(buf);
}

}
