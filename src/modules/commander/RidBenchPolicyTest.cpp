/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>
#include "HealthAndArmingChecks/checks/openDroneIDCheck.hpp"
#include <uORB/Publication.hpp>

#ifndef CONFIG_RID_UOM_TEST_MOCK
#error This test must compile the actual health check with the bench firmware flag.
#endif

TEST(RidBenchPolicy, CompiledBenchCheckAcceptsMockOnlyWithCompleteReadiness)
{
	param_control_autosave(false);
	int32_t saved_product = 0, saved_arm = 0;
	ASSERT_EQ(param_get(param_find("COM_RID_UOM"), &saved_product), PX4_OK);
	ASSERT_EQ(param_get(param_find("COM_ARM_ODID"), &saved_arm), PX4_OK);
	const int32_t product = 1, arm_policy = 2;
	ASSERT_EQ(param_set(param_find("COM_RID_UOM"), &product), PX4_OK);
	ASSERT_EQ(param_set(param_find("COM_ARM_ODID"), &arm_policy), PX4_OK);

	const uint64_t now = hrt_absolute_time();
	rid_identity_s identity{};
	identity.timestamp = now;
	identity.result = rid_identity_s::RESULT_OK;
	identity.activation_state = rid_identity_s::ACTIVATION_ACTIVE;
	identity.boot_id = 123;
	identity.session_id = 456;
	identity.identity_revision = 7;
	identity.rid_boot_id = 100;
	identity.rid_node_id = 42;
	identity.target_system = 255;
	identity.target_component = 190;
	rid_uom_status_s uom{};
	uom.timestamp = now;
	uom.boot_id = identity.boot_id;
	uom.session_id = identity.session_id;
	uom.identity_revision = identity.identity_revision;
	uom.rid_boot_id = identity.rid_boot_id;
	uom.source_system = identity.target_system;
	uom.source_component = identity.target_component;
	uom.environment = rid_uom_status_s::ENVIRONMENT_MOCK;
	uom.state = rid_uom_status_s::STATE_READY;
	uom.flags = 15;
	open_drone_id_arm_status_s arm{};
	arm.timestamp = now;
	arm.node_id = 42;
	uORB::Publication<rid_identity_s> identity_pub{ORB_ID(rid_identity)};
	uORB::Publication<rid_uom_status_s> uom_pub{ORB_ID(rid_uom_status)};
	uORB::Publication<open_drone_id_arm_status_s> arm_pub{ORB_ID(open_drone_id_arm_status)};
	ASSERT_TRUE(identity_pub.publish(identity));
	ASSERT_TRUE(uom_pub.publish(uom));
	ASSERT_TRUE(arm_pub.publish(arm));
	vehicle_status_s status{};
	status.open_drone_id_system_present = true;
	status.open_drone_id_system_healthy = true;
	Context context{status};
	OpenDroneIDChecks check;
	check.updateParams();
	failsafe_flags_s flags{};
	Report ready{flags, 0};
	check.checkAndReport(context, ready);
	EXPECT_EQ(ready.armingCheckResults().can_arm, NavModes::All);
	EXPECT_FALSE(flags.uom_unhealthy);

	uom.flags = 7; // Bench mode cannot bypass authentication or telemetry validity.
	ASSERT_TRUE(uom_pub.publish(uom));
	Report incomplete{flags, 0};
	check.checkAndReport(context, incomplete);
	EXPECT_NE(incomplete.armingCheckResults().can_arm, NavModes::All);
	EXPECT_TRUE(flags.uom_unhealthy);
	EXPECT_EQ(param_set(param_find("COM_RID_UOM"), &saved_product), PX4_OK);
	EXPECT_EQ(param_set(param_find("COM_ARM_ODID"), &saved_arm), PX4_OK);
}
