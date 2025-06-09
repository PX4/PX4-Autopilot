#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/quadruped_leg_command.h>

using namespace time_literals;

class Quadruped : public ModuleBase<Quadruped>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
        Quadruped() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
        {
        }

        ~Quadruped() override { }

        bool init() {
                ScheduleOnInterval(100_ms); // 10 Hz
                return true;
        }

        void Run() override {
                if (should_exit()) {
                        ScheduleClear();
                        exit_and_cleanup();
                        return;
                }

                quadruped_leg_command_s cmd{};
                cmd.timestamp = hrt_absolute_time();
                // currently send zeros
                _cmd_pub.publish(cmd);
        }

private:
        uORB::Publication<quadruped_leg_command_s> _cmd_pub{ORB_ID(quadruped_leg_command)};
};

int Quadruped_main(int argc, char *argv[])
{
        return Quadruped::main(argc, argv);
}
