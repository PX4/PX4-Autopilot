#include <gtest/gtest.h>

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	std::cout << "Run ECL gtests" << std::endl;
	return RUN_ALL_TESTS();
}
