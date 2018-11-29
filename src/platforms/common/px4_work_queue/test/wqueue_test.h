
#pragma once

#include <px4_app.h>
#include <px4_work_queue/WorkItem.hpp>
#include <string.h>

using namespace px4;

class WQueueTest : public px4::WorkItem
{
public:
	WQueueTest() : px4::WorkItem(px4::wq_configurations[test1]) {}
	~WQueueTest() = default;

	int main();

	void Run() override;

	static px4::AppState appState; /* track requests to terminate app */

private:
	int _iter{0};
};
