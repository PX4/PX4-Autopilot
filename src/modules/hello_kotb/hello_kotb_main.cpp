#include "hello_kotb.hpp"
// #include <px4_platform_common/log.h>

extern "C" __EXPORT int hello_kotb_main(int argc, char *argv[])
{
	PX4_INFO("hello_kotb_main::WE ARE ONLINE!!!");
	static auto hello = std::make_unique<HelloKotb>(px4::wq_configurations::INS0);
	hello->ScheduleOnInterval(1000000);
	return 0;
}

