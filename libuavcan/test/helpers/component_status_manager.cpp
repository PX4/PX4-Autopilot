/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/helpers/component_status_manager.hpp>


TEST(ComponentStatusManager, Basic)
{
    uavcan::ComponentStatusManager<4> csm;

    ASSERT_EQ(4, uavcan::ComponentStatusManager<4>::NumComponents);
    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_OK, csm.getWorstStatusCode());

    csm.setComponentStatus(3, uavcan::protocol::NodeStatus::STATUS_WARNING);
    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_WARNING, csm.getWorstStatusCode());

    csm.setComponentStatus(2, uavcan::protocol::NodeStatus::STATUS_CRITICAL);
    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_CRITICAL, csm.getWorstStatusCode());

    csm.setComponentStatus(3, uavcan::protocol::NodeStatus::STATUS_INITIALIZING);
    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_CRITICAL, csm.getWorstStatusCode());

    csm.setComponentStatus(2, uavcan::protocol::NodeStatus::STATUS_INITIALIZING);
    ASSERT_EQ(uavcan::protocol::NodeStatus::STATUS_INITIALIZING, csm.getWorstStatusCode());
}
