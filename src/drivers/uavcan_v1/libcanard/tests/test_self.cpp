// This software is distributed under the terms of the MIT License.
// Copyright (c) 2016-2020 UAVCAN Development Team.

#include "exposed.hpp"
#include "helpers.hpp"

TEST_CASE("TestAllocator")
{
    helpers::TestAllocator al;

    REQUIRE(0 == al.getNumAllocatedFragments());
    REQUIRE(std::numeric_limits<std::size_t>::max() == al.getAllocationCeiling());

    auto* a = al.allocate(123);
    REQUIRE(1 == al.getNumAllocatedFragments());
    REQUIRE(123 == al.getTotalAllocatedAmount());

    auto* b = al.allocate(456);
    REQUIRE(2 == al.getNumAllocatedFragments());
    REQUIRE(579 == al.getTotalAllocatedAmount());

    al.setAllocationCeiling(600);

    REQUIRE(nullptr == al.allocate(100));
    REQUIRE(2 == al.getNumAllocatedFragments());
    REQUIRE(579 == al.getTotalAllocatedAmount());

    auto* c = al.allocate(21);
    REQUIRE(3 == al.getNumAllocatedFragments());
    REQUIRE(600 == al.getTotalAllocatedAmount());

    al.deallocate(a);
    REQUIRE(2 == al.getNumAllocatedFragments());
    REQUIRE(477 == al.getTotalAllocatedAmount());

    auto* d = al.allocate(100);
    REQUIRE(3 == al.getNumAllocatedFragments());
    REQUIRE(577 == al.getTotalAllocatedAmount());

    al.deallocate(c);
    REQUIRE(2 == al.getNumAllocatedFragments());
    REQUIRE(556 == al.getTotalAllocatedAmount());

    al.deallocate(d);
    REQUIRE(1 == al.getNumAllocatedFragments());
    REQUIRE(456 == al.getTotalAllocatedAmount());

    al.deallocate(b);
    REQUIRE(0 == al.getNumAllocatedFragments());
    REQUIRE(0 == al.getTotalAllocatedAmount());
}
