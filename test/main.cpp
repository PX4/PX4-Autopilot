#include <gtest/gtest.h>

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  std::cout << "init test \n";
  return RUN_ALL_TESTS();
}
