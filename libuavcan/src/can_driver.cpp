/*
 * CAN bus driver interface.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <cassert>
#include <uavcan/can_driver.hpp>

namespace uavcan
{

std::string CanFrame::toString(StringRepresentation mode) const
{
    using std::snprintf;

    assert(mode == STR_TIGHT || mode == STR_ALIGNED);

    static const int ASCII_COLUMN_OFFSET = 36;

    char buf[50];
    char* wpos = buf, *epos = buf + sizeof(buf);
    std::fill(buf, buf + sizeof(buf), 0);

    if (id & FLAG_EFF)
    {
        wpos += snprintf(wpos, epos - wpos, "0x%08x  ", (unsigned int)(id & MASK_EXTID));
    }
    else
    {
        const char* const fmt = (mode == STR_ALIGNED) ? "     0x%03x  " : "0x%03x  ";
        wpos += snprintf(wpos, epos - wpos, fmt, (unsigned int)(id & MASK_STDID));
    }

    if (id & FLAG_RTR)
    {
        wpos += snprintf(wpos, epos - wpos, " RTR");
    }
    else
    {
        for (int dlen = 0; dlen < dlc; dlen++)                                 // hex bytes
            wpos += snprintf(wpos, epos - wpos, " %02x", (unsigned int)data[dlen]);

        while (mode == STR_ALIGNED && wpos < buf + ASCII_COLUMN_OFFSET)        // alignment
            *wpos++ = ' ';

        wpos += snprintf(wpos, epos - wpos, "  \'");                           // ascii
        for (int dlen = 0; dlen < dlc; dlen++)
        {
            uint8_t ch = data[dlen];
            if (ch < 0x20 || ch > 0x7E)
                ch = '.';
            wpos += snprintf(wpos, epos - wpos, "%c", ch);
        }
        wpos += snprintf(wpos, epos - wpos, "\'");
    }
    return std::string(buf);
}

}
