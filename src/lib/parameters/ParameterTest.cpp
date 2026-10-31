/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <lib/tinybson/tinybson.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/uORBManager.hpp>

#include <gtest/gtest.h>
#include <fcntl.h>
#include <unistd.h>

class ParameterTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}
};


TEST_F(ParameterTest, testParamReadWrite)
{
	// GIVEN a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);

	// WHEN: we get the parameter
	float value = -999.f;
	int status = param_get(param, &value);

	// THEN it should be successful and have the default value
	EXPECT_EQ(0, status);
	EXPECT_FLOAT_EQ(-1.f, value);

	// WHEN: we set the parameter
	value = 42.f;
	status = param_set(param, &value);

	// THEN: it should be successful
	EXPECT_EQ(0, status);

	// WHEN: we get the parameter again
	float value2 = -1999.f;
	status = param_get(param, &value2);

	// THEN: it should be exactly the value we set
	EXPECT_EQ(0, status);
	EXPECT_FLOAT_EQ(42.f, value2);
}


TEST_F(ParameterTest, testUorbSendReceive)
{
	// GIVEN: a uOrb message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.min_distance = 1.f;
	message.max_distance = 10.f;

	// AND: a subscriber
	uORB::SubscriptionData<obstacle_distance_s> sub_obstacle_distance{ORB_ID(obstacle_distance)};

	// WHEN we send the message
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	ASSERT_TRUE(obstacle_distance_pub != nullptr);

	// THEN: the subscriber should receive the message
	sub_obstacle_distance.update();
	const obstacle_distance_s &obstacle_distance = sub_obstacle_distance.get();

	// AND: the values we got should be the same
	EXPECT_EQ(message.timestamp, obstacle_distance.timestamp);
	EXPECT_FLOAT_EQ(message.min_distance, obstacle_distance.min_distance);
	EXPECT_FLOAT_EQ(message.max_distance, obstacle_distance.max_distance);

	// AND: all the bytes should be equal
	EXPECT_EQ(0, memcmp(&message, &obstacle_distance, sizeof(message)));
}

// Every kind of DEFINE_PARAMETERS member, so the table loader is checked for each slot layout
class ParamTableModule : public ModuleParams
{
public:
	ParamTableModule() : ModuleParams(nullptr), _param_cp_delay(_ext_delay), _param_sys_autocfg(_ext_autoconfig) {}

	void update() { updateParams(); }

	float ext_delay() const { return _ext_delay; }
	int32_t ext_autoconfig() const { return _ext_autoconfig; }

	// declared before the parameters, so they exist before the table loads into them
	float _ext_delay{-1.f};
	int32_t _ext_autoconfig{-1};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CP_DIST>) _param_cp_dist,
		(ParamBool<px4::params::CP_GO_NO_DATA>) _param_cp_go_no_data,
		(ParamExtFloat<px4::params::CP_DELAY>) _param_cp_delay,
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
		(ParamExtInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autocfg,
		(ParamFloat<px4::params::CP_GUIDE_ANG>) _param_cp_guide_ang
	)
};

TEST_F(ParameterTest, testDefineParametersTable)
{
	const float dist = 3.5f, delay = 0.25f, guide_ang = 12.f;
	const int32_t go_no_data = 1, autostart = 4001, autoconfig = 2;

	// GIVEN: non-default values for each kind of parameter
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_DIST), &dist));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_GO_NO_DATA), &go_no_data));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_DELAY), &delay));
	ASSERT_EQ(0, param_set(param_handle(px4::params::SYS_AUTOSTART), &autostart));
	ASSERT_EQ(0, param_set(param_handle(px4::params::SYS_AUTOCONFIG), &autoconfig));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_GUIDE_ANG), &guide_ang));

	// WHEN: the module is constructed
	ParamTableModule module;

	// THEN: every member holds the stored value and is marked used
	EXPECT_FLOAT_EQ(dist, module._param_cp_dist.get());
	EXPECT_TRUE(module._param_cp_go_no_data.get());
	EXPECT_FLOAT_EQ(delay, module._param_cp_delay.get());
	EXPECT_FLOAT_EQ(delay, module.ext_delay());
	EXPECT_EQ(autostart, module._param_sys_autostart.get());
	EXPECT_EQ(autoconfig, module._param_sys_autocfg.get());
	EXPECT_EQ(autoconfig, module.ext_autoconfig());
	EXPECT_FLOAT_EQ(guide_ang, module._param_cp_guide_ang.get());
	EXPECT_TRUE(param_used(param_handle(px4::params::CP_DIST)));
	EXPECT_TRUE(param_used(param_handle(px4::params::CP_GUIDE_ANG)));

	// WHEN: the stored values change and the module updates
	const float dist2 = 7.f, delay2 = 0.5f, guide_ang2 = 20.f;
	const int32_t go_no_data2 = 0, autostart2 = 4002, autoconfig2 = 0;
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_DIST), &dist2));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_GO_NO_DATA), &go_no_data2));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_DELAY), &delay2));
	ASSERT_EQ(0, param_set(param_handle(px4::params::SYS_AUTOSTART), &autostart2));
	ASSERT_EQ(0, param_set(param_handle(px4::params::SYS_AUTOCONFIG), &autoconfig2));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_GUIDE_ANG), &guide_ang2));
	module.update();

	// THEN: every member holds the new value
	EXPECT_FLOAT_EQ(dist2, module._param_cp_dist.get());
	EXPECT_FALSE(module._param_cp_go_no_data.get());
	EXPECT_FLOAT_EQ(delay2, module.ext_delay());
	EXPECT_EQ(autostart2, module._param_sys_autostart.get());
	EXPECT_EQ(autoconfig2, module.ext_autoconfig());
	EXPECT_FLOAT_EQ(guide_ang2, module._param_cp_guide_ang.get());
}

// A parent and a child that each declare parameters, so each class has its own table
class ParamTableParent : public ModuleParams
{
public:
	ParamTableParent() : ModuleParams(nullptr) {}

	void update() { updateParams(); }

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamFloat<px4::params::CP_DIST>) _param_cp_dist,
					(ParamBool<px4::params::CP_GO_NO_DATA>) _param_cp_go_no_data
				       )
};

class ParamTableChild : public ParamTableParent
{
public:
	DEFINE_PARAMETERS_CUSTOM_PARENT(ParamTableParent,
					(ParamFloat<px4::params::CP_DELAY>) _param_cp_delay,
					(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart
				       )
};

TEST_F(ParameterTest, testDefineParametersTableCustomParent)
{
	const float dist = 3.5f, delay = 0.25f;
	const int32_t go_no_data = 1, autostart = 4001;

	// GIVEN: non-default values for the parent's and the child's parameters
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_DIST), &dist));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_GO_NO_DATA), &go_no_data));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_DELAY), &delay));
	ASSERT_EQ(0, param_set(param_handle(px4::params::SYS_AUTOSTART), &autostart));

	// WHEN: the child is constructed
	ParamTableChild child;

	// THEN: both tables are loaded
	EXPECT_FLOAT_EQ(dist, child._param_cp_dist.get());
	EXPECT_TRUE(child._param_cp_go_no_data.get());
	EXPECT_FLOAT_EQ(delay, child._param_cp_delay.get());
	EXPECT_EQ(autostart, child._param_sys_autostart.get());

	// WHEN: the stored values change and the child updates
	const float dist2 = 7.f, delay2 = 0.5f;
	const int32_t go_no_data2 = 0, autostart2 = 4002;
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_DIST), &dist2));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_GO_NO_DATA), &go_no_data2));
	ASSERT_EQ(0, param_set(param_handle(px4::params::CP_DELAY), &delay2));
	ASSERT_EQ(0, param_set(param_handle(px4::params::SYS_AUTOSTART), &autostart2));
	child.update();

	// THEN: the parent's and the child's parameters hold the new values
	EXPECT_FLOAT_EQ(dist2, child._param_cp_dist.get());
	EXPECT_FALSE(child._param_cp_go_no_data.get());
	EXPECT_FLOAT_EQ(delay2, child._param_cp_delay.get());
	EXPECT_EQ(autostart2, child._param_sys_autostart.get());
}

// A BSON null in an import resets the param (a delta tombstone on flash boards).
TEST_F(ParameterTest, testImportTombstoneResets)
{
	const param_t cp_dist = param_handle(px4::params::CP_DIST);

	float cp = 5.f;
	ASSERT_EQ(0, param_set(cp_dist, &cp));
	ASSERT_FALSE(param_value_is_default(cp_dist));

	bson_encoder_s encoder{};
	ASSERT_EQ(0, bson_encoder_init_buf(&encoder, nullptr, 0));
	ASSERT_EQ(0, bson_encoder_append_int32(&encoder, "RC1_REV", -1));
	ASSERT_EQ(0, bson_encoder_append_null(&encoder, "CP_DIST"));
	ASSERT_EQ(0, bson_encoder_fini(&encoder));

	char path[] = "/tmp/param_tombstone_XXXXXX";
	int fd = mkstemp(path);
	ASSERT_GE(fd, 0);
	ASSERT_EQ((ssize_t)bson_encoder_buf_size(&encoder), write(fd, bson_encoder_buf_data(&encoder),
			bson_encoder_buf_size(&encoder)));
	free(bson_encoder_buf_data(&encoder));
	ASSERT_EQ(0, lseek(fd, 0, SEEK_SET));

	EXPECT_EQ(0, param_import(fd));
	close(fd);
	unlink(path);

	EXPECT_TRUE(param_value_is_default(cp_dist));

	int32_t rev = 0;
	ASSERT_EQ(0, param_get(param_handle(px4::params::RC1_REV), &rev));
	EXPECT_EQ(-1, rev);
}
