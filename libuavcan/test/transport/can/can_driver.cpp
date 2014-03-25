/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "can.hpp"
#include <uavcan/driver/can.hpp>

TEST(CanFrame, FrameProperties)
{
    EXPECT_TRUE(makeCanFrame(0, "", EXT).isExtended());
    EXPECT_FALSE(makeCanFrame(0, "", STD).isExtended());
    EXPECT_FALSE(makeCanFrame(0, "", EXT).isRemoteTransmissionRequest());
    EXPECT_FALSE(makeCanFrame(0, "", STD).isRemoteTransmissionRequest());

    uavcan::CanFrame frame = makeCanFrame(123, "", STD);
    frame.id |= uavcan::CanFrame::FlagRTR;
    EXPECT_TRUE(frame.isRemoteTransmissionRequest());
    EXPECT_FALSE(frame.isErrorFrame());
    frame.id |= uavcan::CanFrame::FlagERR;
    EXPECT_TRUE(frame.isErrorFrame());
}

TEST(CanFrame, ToString)
{
    uavcan::CanFrame frame = makeCanFrame(123, "\x01\x02\x03\x04" "1234", EXT);
    EXPECT_EQ("0x0000007b   01 02 03 04 31 32 33 34  '....1234'", frame.toString());
    EXPECT_TRUE(frame.toString() == frame.toString(uavcan::CanFrame::StrAligned));

    frame = makeCanFrame(123, "z", EXT);
    EXPECT_EQ("0x0000007b   7a                       'z'", frame.toString(uavcan::CanFrame::StrAligned));
    EXPECT_EQ("0x0000007b   7a  'z'", frame.toString());

    EXPECT_EQ("     0x141   61 62 63 64 aa bb cc dd  'abcd....'",
              makeCanFrame(321, "abcd" "\xaa\xbb\xcc\xdd", STD).toString(uavcan::CanFrame::StrAligned));

    EXPECT_EQ("     0x100                            ''",
              makeCanFrame(256, "", STD).toString(uavcan::CanFrame::StrAligned));

    EXPECT_EQ("0x100    ''",
              makeCanFrame(256, "", STD).toString());

    EXPECT_EQ("0x141   61 62 63 64 aa bb cc dd  'abcd....'",
              makeCanFrame(321, "abcd" "\xaa\xbb\xcc\xdd", STD).toString());
}
