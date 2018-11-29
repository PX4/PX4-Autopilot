
#pragma once

#include <px4_app.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <string.h>

using namespace px4;

class WQueueScheduledTest : public px4::ScheduledWorkItem
{
public:
	WQueueScheduledTest() : px4::ScheduledWorkItem(px4::wq_configurations[test2]) {}
	~WQueueScheduledTest() = default;

	int main();

	void Run() override;

	static px4::AppState appState; /* track requests to terminate app */

private:
	int _iter{0};
};
