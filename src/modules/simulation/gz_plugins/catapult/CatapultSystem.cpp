#include <atomic>
#include <chrono>
#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace custom
{

class CatapultSystem : public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate,
	public gz::sim::ISystemReset
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
	{
		const gz::sim::Model model(entity);

		if (!model.Valid(ecm) || model.Static(ecm)) {
			gzerr << "CatapultSystem requires a dynamic model\n";
			return;
		}

		sdf::Errors errors;
		const auto link_name = sdf->Get<std::string>(errors, "link_name", "").first;
		_force = sdf->Get<double>(errors, "force", 1.0).first;
		_duration = sdf->Get<double>(errors, "duration", 0.01).first;
		_direction = sdf->Get<gz::math::Vector3d>(errors, "direction", {1, 0, 2}).first;
		_motor_number = sdf->Get<int>(errors, "motorNumber", 0).first;
		const auto command_sub_topic = sdf->Get<std::string>(errors, "commandSubTopic", "/command/motor_speed").first;

		if (!errors.empty() || link_name.empty() || !std::isfinite(_force) || _force <= 0
		    || !std::isfinite(_duration) || _duration <= 0 || !_direction.IsFinite()
		    || !std::isfinite(_direction.Length()) || _direction.Length() <= 0 || _motor_number < 0) {
			gzerr << "CatapultSystem has invalid configuration\n";
			return;
		}

		const auto links = gz::sim::entitiesFromScopedName(link_name, ecm, entity);

		if (links.size() != 1 || !gz::sim::Link(*links.begin()).Valid(ecm)) {
			gzerr << "CatapultSystem cannot resolve link " << link_name << '\n';
			return;
		}

		_link = *links.begin();
		_direction.Normalize();

		_enabled = _node.Subscribe("/" + model.Name(ecm) + command_sub_topic, &CatapultSystem::motorCallback, this);

		if (!_enabled) {
			gzerr << "CatapultSystem cannot subscribe to motor speed commands\n";
		}
	}

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
	{
		if (info.dt < std::chrono::steady_clock::duration::zero()) {
			Reset(info, ecm);
		}

		if (!_enabled || info.paused || info.dt <= std::chrono::steady_clock::duration::zero() || _finished) {
			return;
		}

		if (!_trigger.load()) {
			return;
		}

		if (!_started) {
			_start_time = info.simTime;
			_started = true;
			return;
		}

		gz::sim::Link(_link).AddWorldForce(ecm, _direction * _force);

		if (std::chrono::duration<double>(info.simTime - _start_time).count() > _duration) {
			_finished = true;
		}
	}

	void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
	{
		_trigger.store(false);
		_started = false;
		_finished = false;
	}

private:
	void motorCallback(const gz::msgs::Actuators &message)
	{
		static constexpr double arm_rot_vel = 100.0;
		_trigger.store(_motor_number < message.velocity_size()
			       && std::isfinite(message.velocity(_motor_number))
			       && message.velocity(_motor_number) > arm_rot_vel);
	}

	gz::sim::Entity _link{gz::sim::kNullEntity};
	gz::math::Vector3d _direction;
	std::chrono::steady_clock::duration _start_time{};
	double _force{1.0};
	double _duration{0.01};
	int _motor_number{0};
	bool _enabled{false};
	bool _started{false};
	bool _finished{false};
	std::atomic<bool> _trigger{false};
	gz::transport::Node _node;
};

}

GZ_ADD_PLUGIN(custom::CatapultSystem, gz::sim::System,
	      custom::CatapultSystem::ISystemConfigure,
	      custom::CatapultSystem::ISystemPreUpdate,
	      custom::CatapultSystem::ISystemReset)
