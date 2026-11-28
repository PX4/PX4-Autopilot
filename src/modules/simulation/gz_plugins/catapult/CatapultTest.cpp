#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

using namespace std::chrono_literals;

namespace
{

class VelocityObserver : public gz::sim::System,
	public gz::sim::ISystemPreUpdate,
	public gz::sim::ISystemPostUpdate,
	public gz::sim::ISystemReset
{
public:
	void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &ecm) override
	{
		for (auto &entry : velocities) {
			const auto links = gz::sim::entitiesFromScopedName(entry.first + "::base_link", ecm);
			ASSERT_EQ(links.size(), 1u);
			gz::sim::Link(*links.begin()).EnableVelocityChecks(ecm);
		}
	}

	void PostUpdate(const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager &ecm) override
	{
		for (auto &entry : velocities) {
			const auto links = gz::sim::entitiesFromScopedName(entry.first + "::base_link", ecm);
			ASSERT_EQ(links.size(), 1u);
			entry.second = gz::sim::Link(*links.begin()).WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
		}
	}

	void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override {}

	std::map<std::string, gz::math::Vector3d> velocities;
};

std::string model(const std::string &name, const std::string &configuration, bool catapult = true)
{
	return "<model name='" + name + "'><pose>0 0 0 0 0 1.5707963267948966</pose>"
	       "<link name='base_link'><inertial><mass>2</mass>"
	       "<inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link>"
	       + (catapult ? std::string("<plugin filename='") + CATAPULT_PLUGIN_PATH
		  + "' name='custom::CatapultSystem'><link_name>base_link</link_name>"
		  + configuration + "</plugin>" : "") + "</model>";
}

void send(gz::transport::Node::Publisher &publisher, const gz::msgs::Actuators &message)
{
	const auto deadline = std::chrono::steady_clock::now() + 5s;

	while (!publisher.HasConnections() && std::chrono::steady_clock::now() < deadline) {
		std::this_thread::sleep_for(10ms);
	}

	ASSERT_TRUE(publisher.HasConnections());
	ASSERT_TRUE(publisher.Publish(message));
	std::this_thread::sleep_for(100ms);
}

}

TEST(Catapult, ClassicMotorTriggerForcePauseAndReset)
{
	const std::string pulse = "<force>50</force><duration>0.5</duration>";
	gz::sim::ServerConfig config;
	ASSERT_TRUE(config.SetSdfString(
			    "<sdf version='1.9'><world name='catapult'><gravity>0 0 0</gravity>"
			    "<physics name='physics' type='ignored'><max_step_size>0.001</max_step_size>"
			    "<real_time_factor>0</real_time_factor></physics>"
			    "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>"
			    + model("other", pulse) + model("untouched", "", false)
			    + model("motor", pulse + "<motorNumber>1</motorNumber>")
			    + model("defaults", "") + model("invalid", "<force>0</force>") + "</world></sdf>"));
	gz::sim::Server server(config);
	auto observer = std::make_shared<VelocityObserver>();

	for (const auto *name : {"other", "untouched", "motor", "defaults", "invalid"}) {
		observer->velocities.emplace(name, gz::math::Vector3d::Zero);
	}

	ASSERT_TRUE(server.AddSystem(observer).value_or(false));
	ASSERT_TRUE(server.Run(true, 10, false));
	gz::transport::Node node;
	auto motor_pub = node.Advertise<gz::msgs::Actuators>("/motor/command/motor_speed");
	auto defaults_pub = node.Advertise<gz::msgs::Actuators>("/defaults/command/motor_speed");
	gz::msgs::Actuators motors;
	send(motor_pub, motors);
	ASSERT_TRUE(server.Run(true, 10, false));
	motors.add_velocity(1000);
	send(motor_pub, motors);
	ASSERT_TRUE(server.Run(true, 10, false));
	motors.add_velocity(0);

	for (const double speed : {0.0, 100.0, std::numeric_limits<double>::quiet_NaN()}) {
		motors.set_velocity(1, speed);
		send(motor_pub, motors);
		ASSERT_TRUE(server.Run(true, 10, false));
	}

	for (const auto &entry : observer->velocities) {
		EXPECT_EQ(entry.second, gz::math::Vector3d::Zero) << entry.first;
	}

	motors.set_velocity(1, 101);
	send(motor_pub, motors);
	send(defaults_pub, motors);
	ASSERT_TRUE(server.Run(true, 1, false));
	EXPECT_EQ(observer->velocities["motor"], gz::math::Vector3d::Zero);
	ASSERT_TRUE(server.Run(true, 249, false));
	const gz::math::Vector3d direction = gz::math::Vector3d(1, 0, 2).Normalized();
	EXPECT_LT((observer->velocities["motor"] - direction * 6.225).Length(), 1e-8);
	const auto before_pause = observer->velocities["motor"];

	for (int i = 0; i < 50; ++i) {
		ASSERT_TRUE(server.RunOnce(true));
	}

	EXPECT_EQ(observer->velocities["motor"], before_pause);
	ASSERT_TRUE(server.Run(true, 400, false));
	EXPECT_LT((observer->velocities["motor"] - direction * 12.525).Length(), 1e-8);
	EXPECT_LT((observer->velocities["defaults"] - direction * 0.0055).Length(), 1e-8);
	EXPECT_EQ(observer->velocities["other"], gz::math::Vector3d::Zero);
	EXPECT_EQ(observer->velocities["untouched"], gz::math::Vector3d::Zero);
	EXPECT_EQ(observer->velocities["invalid"], gz::math::Vector3d::Zero);

	motors.set_velocity(1, 0);
	send(motor_pub, motors);
	ASSERT_TRUE(server.Run(true, 10, false));
	motors.set_velocity(1, 101);
	send(motor_pub, motors);
	ASSERT_TRUE(server.Run(true, 100, false));
	EXPECT_LT((observer->velocities["motor"] - direction * 12.525).Length(), 1e-8);

	gz::msgs::WorldControl reset;
	reset.mutable_reset()->set_all(true);
	gz::msgs::Boolean reply;
	bool result = false;
	ASSERT_TRUE(node.Request("/world/catapult/control", reset, 5000, reply, result));
	ASSERT_TRUE(result);
	ASSERT_TRUE(reply.data());
	ASSERT_TRUE(server.Run(true, 10, false));
	EXPECT_EQ(observer->velocities["motor"], gz::math::Vector3d::Zero);
	send(motor_pub, motors);
	ASSERT_TRUE(server.Run(true, 200, false));
	motors.set_velocity(1, 0);
	send(motor_pub, motors);
	ASSERT_TRUE(server.Run(true, 100, false));
	motors.set_velocity(1, 101);
	send(motor_pub, motors);
	ASSERT_TRUE(server.Run(true, 400, false));
	EXPECT_LT((observer->velocities["motor"] - direction * 10.025).Length(), 1e-8);
}
