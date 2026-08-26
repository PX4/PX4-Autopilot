#include "hello_kotb.hpp"

HelloKotb::HelloKotb(const px4::wq_config_t &config):
	ScheduledWorkItem(MODULE_NAME, config)
{

}
void HelloKotb::Run()
{
	printf("Hello Kotb!!!\n");
}
