#include "GL40IIProtocol.hpp"

#include <cstring>
#include <gtest/gtest.h>

using namespace gl40ii;

TEST(GL40IIProtocol, AddressAllocation)
{
	EXPECT_EQ(commandId(Mode::MIT, 1), 0x001);
	EXPECT_EQ(commandId(Mode::PositionVelocity, 1), 0x101);
	EXPECT_EQ(commandId(Mode::Velocity, 6), 0x206);
	EXPECT_EQ(masterId(0), 0x011);
	EXPECT_EQ(masterId(5), 0x016);
	EXPECT_EQ(busIndex(0), 0);
	EXPECT_EQ(busIndex(1), 1);
	EXPECT_EQ(busIndex(4), 0);
	EXPECT_EQ(busIndex(5), 1);
}

TEST(GL40IIProtocol, SpecialCommands)
{
	Frame frame{};
	ASSERT_TRUE(packSpecial(Mode::MIT, 1, SpecialCommand::ClearFault, frame));
	EXPECT_EQ(frame.id, 0x001);
	EXPECT_EQ(frame.dlc, 8);

	for (int index = 0; index < 7; ++index) {
		EXPECT_EQ(frame.data[index], 0xFF);
	}

	EXPECT_EQ(frame.data[7], 0xFB);
	ASSERT_TRUE(packSpecial(Mode::Velocity, 6, SpecialCommand::Disable, frame));
	EXPECT_EQ(frame.id, 0x206);
	EXPECT_EQ(frame.data[7], 0xFD);
}

TEST(GL40IIProtocol, ManualMitPositionAndTorqueExamples)
{
	const Limits limits{12.5f, 200.f, 10.f};
	Frame frame{};

	ASSERT_TRUE(packMit(1, limits, 2.f, 0.f, 0.123f, 0.005f, 0.f, frame));
	const uint8_t positive_position[8] {0x94, 0x7A, 0x7F, 0xF0, 0x01, 0x00, 0x47, 0xFF};
	EXPECT_EQ(0, std::memcmp(frame.data, positive_position, sizeof(positive_position)));

	ASSERT_TRUE(packMit(1, limits, -2.f, 0.f, 0.123f, 0.005f, 0.f, frame));
	const uint8_t negative_position[8] {0x6B, 0x84, 0x7F, 0xF0, 0x01, 0x00, 0x47, 0xFF};
	EXPECT_EQ(0, std::memcmp(frame.data, negative_position, sizeof(negative_position)));

	ASSERT_TRUE(packMit(1, limits, 0.f, 0.f, 0.f, 0.f, 0.03f, frame));
	const uint8_t positive_torque[8] {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x08, 0x05};
	EXPECT_EQ(0, std::memcmp(frame.data, positive_torque, sizeof(positive_torque)));
}

TEST(GL40IIProtocol, ManualVelocityExampleUsesVmax250)
{
	const Limits limits{12.5f, 250.f, 10.f};
	Frame frame{};
	ASSERT_TRUE(packMit(1, limits, 0.f, 6.f, 0.f, 0.005f, 0.f, frame));
	const uint8_t expected[8] {0x7F, 0xFF, 0x83, 0x00, 0x00, 0x00, 0x47, 0xFF};
	EXPECT_EQ(0, std::memcmp(frame.data, expected, sizeof(expected)));
}

TEST(GL40IIProtocol, FloatModesAreExplicitLittleEndian)
{
	Frame frame{};
	ASSERT_TRUE(packPositionVelocity(1, 3.f, 1.f, frame));
	const uint8_t position_velocity[8] {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80, 0x3F};
	EXPECT_EQ(frame.id, 0x101);
	EXPECT_EQ(0, std::memcmp(frame.data, position_velocity, sizeof(position_velocity)));

	ASSERT_TRUE(packVelocity(1, -2.f, frame));
	const uint8_t velocity[4] {0x00, 0x00, 0x00, 0xC0};
	EXPECT_EQ(frame.id, 0x201);
	EXPECT_EQ(frame.dlc, 4);
	EXPECT_EQ(0, std::memcmp(frame.data, velocity, sizeof(velocity)));
}

TEST(GL40IIProtocol, FeedbackValidationAndDecode)
{
	const Limits limits{12.5f, 200.f, 10.f};
	Frame frame{};
	frame.id = 0x011;
	frame.dlc = 8;
	frame.data[0] = 0x11;
	frame.data[1] = 0x7F;
	frame.data[2] = 0xFF;
	frame.data[3] = 0x7F;
	frame.data[4] = 0xF7;
	frame.data[5] = 0xFF;
	frame.data[6] = 42;
	frame.data[7] = 35;

	Feedback feedback{};
	ASSERT_TRUE(parseFeedback(frame, 0x011, 1, limits, feedback));
	EXPECT_EQ(feedback.slave_id, 1);
	EXPECT_EQ(feedback.state, 1);
	EXPECT_NEAR(feedback.position, 0.f, 4e-4f);
	EXPECT_NEAR(feedback.velocity, 0.f, 0.1f);
	EXPECT_NEAR(feedback.torque, 0.f, 0.01f);
	EXPECT_EQ(feedback.driver_temperature, 42);
	EXPECT_EQ(feedback.motor_temperature, 35);

	EXPECT_FALSE(parseFeedback(frame, 0x012, 1, limits, feedback));
	EXPECT_FALSE(parseFeedback(frame, 0x011, 2, limits, feedback));
	frame.dlc = 7;
	EXPECT_FALSE(parseFeedback(frame, 0x011, 1, limits, feedback));
}

TEST(GL40IIProtocol, RejectsUnsafeInputs)
{
	const Limits limits{12.5f, 200.f, 10.f};
	Frame frame{};
	EXPECT_FALSE(packMit(1, limits, 13.f, 0.f, 0.f, 0.005f, 0.f, frame));
	EXPECT_FALSE(packMit(1, limits, 0.f, 0.f, 501.f, 0.005f, 0.f, frame));
	EXPECT_FALSE(packMit(0, limits, 0.f, 0.f, 0.f, 0.005f, 0.f, frame));
}
