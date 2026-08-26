/**
 * @file hello_kotb.hpp
 * This file is for Hello PX4 task
 *
 * @author KOTB
 */

#include <memory>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class HelloKotb final : public px4::ScheduledWorkItem
{
public:
	HelloKotb( const px4::wq_config_t &config);
private:
	void Run() override;
};
