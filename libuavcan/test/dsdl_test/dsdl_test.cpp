/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <root_ns_a/EmptyService.hpp>
#include <root_ns_a/EmptyMessage.hpp>
#include <root_ns_a/NestedMessage.hpp>
#include <root_ns_a/ReportBackSoldier.hpp>


TEST(Dsdl, Basic)
{
    uavcan::StaticTransferBuffer<100> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);
}
