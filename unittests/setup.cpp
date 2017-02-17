#include <systemlib/param/param.h>

#include "gtest/gtest.h"


class TestEnvironment: public ::testing::Environment {
public:
	/**
	 * Testing setup: this is called before the actual tests are executed.
	 */
	virtual void SetUp()
	{
		param_init();
	}
};

int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	// gtest takes ownership of the TestEnvironment ptr - we don't delete it.
	::testing::AddGlobalTestEnvironment(new TestEnvironment);
	return RUN_ALL_TESTS();
}
