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
#include <type_traits>
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

class PlainStructParams : public ModuleParams
{
public:
	PlainStructParams() : ModuleParams(nullptr) {}

	void update() { updateParams(); }

	PARAMETERS(
		CP_DIST,
		SYS_AUTOSTART,
		CP_GO_NO_DATA,
		CP_DELAY
	)
};

static_assert(std::is_same<decltype(PlainStructParams::ParameterValues::CP_DIST), float>::value, "float parameter");
static_assert(std::is_same<decltype(PlainStructParams::ParameterValues::SYS_AUTOSTART), int32_t>::value,
	      "int32 parameter");
static_assert(std::is_same<decltype(PlainStructParams::ParameterValues::CP_GO_NO_DATA), int32_t>::value,
	      "boolean parameter");

static void setParam(px4::params param, float value)
{
	ASSERT_EQ(0, param_set(param_handle(param), &value));
}

static void setParam(px4::params param, int32_t value)
{
	ASSERT_EQ(0, param_set(param_handle(param), &value));
}

TEST_F(ParameterTest, plainStructLoadsAtConstruction)
{
	setParam(px4::params::CP_DIST, 3.5f);
	setParam(px4::params::SYS_AUTOSTART, 4001);
	setParam(px4::params::CP_GO_NO_DATA, 1);

	PlainStructParams params;

	EXPECT_FLOAT_EQ(3.5f, params._param.CP_DIST);
	EXPECT_EQ(4001, params._param.SYS_AUTOSTART);
	EXPECT_EQ(1, params._param.CP_GO_NO_DATA);
	EXPECT_FLOAT_EQ(0.4f, params._param.CP_DELAY);

	EXPECT_TRUE(param_used(param_handle(px4::params::CP_DELAY)));
}

TEST_F(ParameterTest, plainStructChangesOnlyOnUpdate)
{
	PlainStructParams params;

	setParam(px4::params::CP_DIST, 7.f);
	setParam(px4::params::SYS_AUTOSTART, 13000);
	setParam(px4::params::CP_GO_NO_DATA, 1);
	setParam(px4::params::CP_DELAY, 1.5f);

	EXPECT_FLOAT_EQ(-1.f, params._param.CP_DIST);
	EXPECT_EQ(0, params._param.SYS_AUTOSTART);
	EXPECT_EQ(0, params._param.CP_GO_NO_DATA);
	EXPECT_FLOAT_EQ(0.4f, params._param.CP_DELAY);

	params.update();

	EXPECT_FLOAT_EQ(7.f, params._param.CP_DIST);
	EXPECT_EQ(13000, params._param.SYS_AUTOSTART);
	EXPECT_EQ(1, params._param.CP_GO_NO_DATA);
	EXPECT_FLOAT_EQ(1.5f, params._param.CP_DELAY);
}

TEST_F(ParameterTest, plainStructWritesBack)
{
	PlainStructParams params;

	// a float and an int32, stored with and without notification
	params._param.CP_DELAY = 2.f;
	EXPECT_TRUE(params._param.commit<px4::params::CP_DELAY>());
	params._param.SYS_AUTOSTART = 4001;
	EXPECT_TRUE(params._param.commit_no_notification<px4::params::SYS_AUTOSTART>());

	float delay = 0.f;
	ASSERT_EQ(0, param_get(param_handle(px4::params::CP_DELAY), &delay));
	EXPECT_FLOAT_EQ(2.f, delay);
	int32_t autostart = 0;
	ASSERT_EQ(0, param_get(param_handle(px4::params::SYS_AUTOSTART), &autostart));
	EXPECT_EQ(4001, autostart);

	// the neighbouring fields are untouched
	EXPECT_FLOAT_EQ(-1.f, params._param.CP_DIST);
	EXPECT_EQ(0, params._param.CP_GO_NO_DATA);

	// set and store only a new value
	EXPECT_FALSE(params._param.commit_no_notification<px4::params::CP_DELAY>(2.f));
	EXPECT_TRUE(params._param.commit_no_notification<px4::params::CP_DELAY>(3.f));
	EXPECT_FLOAT_EQ(3.f, params._param.CP_DELAY);
	ASSERT_EQ(0, param_get(param_handle(px4::params::CP_DELAY), &delay));
	EXPECT_FLOAT_EQ(3.f, delay);

	EXPECT_FALSE(params._param.commit_no_notification<px4::params::SYS_AUTOSTART>(4001));
	EXPECT_TRUE(params._param.commit_no_notification<px4::params::SYS_AUTOSTART>(13000));
	EXPECT_EQ(13000, params._param.SYS_AUTOSTART);

	// reload one field
	setParam(px4::params::CP_DIST, 8.f);
	setParam(px4::params::CP_DELAY, 5.f);
	EXPECT_TRUE(params._param.update<px4::params::CP_DIST>());
	EXPECT_FLOAT_EQ(8.f, params._param.CP_DIST);
	EXPECT_FLOAT_EQ(3.f, params._param.CP_DELAY);

	// back to the default
	params._param.reset<px4::params::CP_DIST>();
	EXPECT_FLOAT_EQ(-1.f, params._param.CP_DIST);
	EXPECT_TRUE(param_value_is_default(param_handle(px4::params::CP_DIST)));
}

class PlainStructParent : public ModuleParams
{
public:
	PlainStructParent() : ModuleParams(nullptr) {}

	void update() { updateParams(); }

	PARAMETERS_CUSTOM_PARENT(ModuleParams,
				 CP_DIST,
				 CP_DELAY
				)
};

class PlainStructChild : public PlainStructParent
{
public:
	float parentDistance() const { return PlainStructParent::_param.CP_DIST; }

	bool storeParentDistance(float distance)
	{
		PlainStructParent::_param.CP_DIST = distance;
		return PlainStructParent::_param.commit<px4::params::CP_DIST>();
	}

	PARAMETERS_CUSTOM_PARENT(PlainStructParent,
				 SYS_AUTOSTART
				)
};

TEST_F(ParameterTest, plainStructChildUpdatesParent)
{
	PlainStructChild child;

	EXPECT_FLOAT_EQ(-1.f, child.parentDistance());
	EXPECT_EQ(0, child._param.SYS_AUTOSTART);

	setParam(px4::params::CP_DIST, 9.f);
	setParam(px4::params::SYS_AUTOSTART, 4001);
	child.update();

	EXPECT_FLOAT_EQ(9.f, child.parentDistance());
	EXPECT_EQ(4001, child._param.SYS_AUTOSTART);

	// the child stores a parent's parameter through the parent's struct
	EXPECT_TRUE(child.storeParentDistance(4.f));
	float distance = 0.f;
	ASSERT_EQ(0, param_get(param_handle(px4::params::CP_DIST), &distance));
	EXPECT_FLOAT_EQ(4.f, distance);
}

// A class tree can convert one class at a time
class MemberParamsParent : public ModuleParams
{
public:
	MemberParamsParent() : ModuleParams(nullptr) {}

	void update() { updateParams(); }

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamFloat<px4::params::CP_DIST>) _param_cp_dist
				       )
};

class PlainStructChildOfMemberParams : public MemberParamsParent
{
public:
	float parentDistance() const { return _param_cp_dist.get(); }

	PARAMETERS_CUSTOM_PARENT(MemberParamsParent,
				 SYS_AUTOSTART
				)
};

TEST_F(ParameterTest, plainStructChildOfMemberParams)
{
	PlainStructChildOfMemberParams child;

	setParam(px4::params::CP_DIST, 9.f);
	setParam(px4::params::SYS_AUTOSTART, 4001);
	child.update();

	EXPECT_FLOAT_EQ(9.f, child.parentDistance());
	EXPECT_EQ(4001, child._param.SYS_AUTOSTART);
}

// Stands in for a library's parameter struct
struct LibraryParameters {
	uint8_t not_a_parameter{7};
	float distance{0.f};
	double also_not_a_parameter{1.0};
	int32_t autostart{0};
};

TEST_F(ParameterTest, memberTableLoadsLibraryStruct)
{
	static constexpr px4::ParameterMember<LibraryParameters> table[] = {
		PARAMETER_AT(CP_DIST, LibraryParameters, distance),
		PARAMETER_AT(SYS_AUTOSTART, LibraryParameters, autostart),
	};

	setParam(px4::params::CP_DIST, 3.5f);
	setParam(px4::params::SYS_AUTOSTART, 4001);

	// loads at construction
	LibraryParameters library;
	px4::ParameterMembers<table> params{&library};

	EXPECT_FLOAT_EQ(3.5f, params->distance);
	EXPECT_EQ(4001, params->autostart);
	EXPECT_EQ(7, params->not_a_parameter);
	EXPECT_DOUBLE_EQ(1.0, params->also_not_a_parameter);
	EXPECT_TRUE(param_used(param_handle(px4::params::CP_DIST)));

	// changes only on update
	setParam(px4::params::CP_DIST, 7.f);
	setParam(px4::params::SYS_AUTOSTART, 13000);
	EXPECT_FLOAT_EQ(3.5f, library.distance);
	EXPECT_TRUE(params.update<px4::params::CP_DIST>());
	EXPECT_FLOAT_EQ(7.f, library.distance);
	EXPECT_EQ(4001, library.autostart);
	params.update();
	EXPECT_EQ(13000, library.autostart);

	// writes back
	params->autostart = 4001;
	EXPECT_TRUE(params.commit<px4::params::SYS_AUTOSTART>());
	int32_t autostart = 0;
	ASSERT_EQ(0, param_get(param_handle(px4::params::SYS_AUTOSTART), &autostart));
	EXPECT_EQ(4001, autostart);

	EXPECT_FALSE(params.commit_no_notification<px4::params::CP_DIST>(7.f));
	EXPECT_TRUE(params.commit_no_notification<px4::params::CP_DIST>(2.f));
	EXPECT_FLOAT_EQ(2.f, library.distance);

	params.reset<px4::params::CP_DIST>();
	EXPECT_FLOAT_EQ(-1.f, library.distance);
}
