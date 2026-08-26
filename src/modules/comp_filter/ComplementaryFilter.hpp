/**
 * @file ComplementaryFilter.hpp
 * This file is for Hello PX4 task
 *
 * @author KOTB
 */

#include <memory>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/comp_filter_status.h>

using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

struct imuSample {
	uint64_t    time_us{};                ///< timestamp of the measurement (uSec)
	Vector3f    gyro{};              ///< delta angle in body frame (integrated gyro measurements) (rad)
	Vector3f    acc{};              ///< delta velocity in body frame (integrated accelerometer measurements) (m/sec)
	float       dt{};           ///< delta angle integration period (sec)
};


class CompFilter final : public ModuleBase<CompFilter>, public px4::ScheduledWorkItem
{
public:
	CompFilter( const px4::wq_config_t &config);
	~CompFilter() override;

	// --- ModuleBase Boilerplate (Ensure all 4 of these are here!) ---
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    static CompFilter *instantiate(int argc, char *argv[]); // <-- This was the missing one!

	bool init();
	// px4::atomic_bool _task_should_exit{false};
private:
	void Run() override;
	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	Vector3f _angles{0.f, 0.f, 0.f};
	void updateGyroAngs(const Vector3f &gyro, const float &dt);
	void updateAccAngs(const Vector3f &acc);
	Vector3f _gyro_angles{0.f, 0.f, 0.f};
	Vector3f _pure_gyro_angles{0.f, 0.f, 0.f};
	Vector3f _acc_angles{0.f, 0.f, 0.f};
	uORB::Publication<comp_filter_status_s> _status_pub{ORB_ID(comp_filter_status)};
	// bool should_exit() const { return _task_should_exit.load(); }
};
