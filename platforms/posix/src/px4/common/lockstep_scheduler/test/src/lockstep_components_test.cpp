#include <lockstep_scheduler/lockstep_components.h>
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace
{

class Waiter
{
public:
	explicit Waiter(LockstepComponents &components)
		: _thread([this, &components]() { components.wait_for_components(); _returned = true; })
	{}

	~Waiter() { _thread.join(); }

	bool returned_within(std::chrono::milliseconds timeout)
	{
		const auto deadline = std::chrono::steady_clock::now() + timeout;

		while (!_returned && std::chrono::steady_clock::now() < deadline) {
			std::this_thread::sleep_for(1ms);
		}

		return _returned;
	}

private:
	std::atomic<bool> _returned{false};
	std::thread _thread;
};

} // namespace

TEST(LockstepComponents, IdleComponentsBankAtMostOneRelease)
{
	LockstepComponents components;

	// A work queue picking up and finishing work while nothing waits yet, as before the simulator
	// engages lockstep.
	for (int i = 0; i < 10; ++i) {
		components.unregister_component(components.register_component());
	}

	const int busy = components.register_component();
	int passed_while_busy = 0;

	for (int i = 0; i < 3; ++i) {
		Waiter waiter(components);

		if (waiter.returned_within(100ms)) {
			++passed_while_busy;

		} else {
			components.unregister_component(busy);
			EXPECT_TRUE(waiter.returned_within(5s));
			break;
		}
	}

	EXPECT_LE(passed_while_busy, 1);
}
