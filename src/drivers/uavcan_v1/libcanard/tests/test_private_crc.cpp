// This software is distributed under the terms of the MIT License.
// Copyright (c) 2016-2020 UAVCAN Development Team.

#include "exposed.hpp"

TEST_CASE("TransferCRC")
{
    using exposed::crcAdd;
    std::uint16_t crc = 0xFFFFU;

    crc = crcAdd(crc, 1, "1");
    crc = crcAdd(crc, 1, "2");
    crc = crcAdd(crc, 1, "3");
    REQUIRE(0x5BCEU == crc);
    crc = crcAdd(crc, 6, "456789");
    REQUIRE(0x29B1U == crc);
}
