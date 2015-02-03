#include <systemlib/visibility.h>
#include <systemlib/param/param.h>

#include "gtest/gtest.h"

static const struct param_info_s test_1 = {
	"TEST_1",
	PARAM_TYPE_INT32,
	.val.i = 2
};

struct param_info_s	param_array[256];
struct param_info_s	*param_info_base;
struct param_info_s	*param_info_limit;

TEST(ParamTest, ResetAll) {
	param_array[0] = test_1;
	param_info_base = (struct param_info_s *) &param_array[0];
	param_info_limit = (struct param_info_s *) &param_array[1];

	printf("diff: %i\n", (unsigned)(param_info_limit - param_info_base));
	
	param_t test_1 = param_find("TEST_1");
	ASSERT_NE(PARAM_INVALID, test_1) << "param_find failed";

	int32_t value;
	int result = param_get(test_1, &value);
	ASSERT_EQ(0, result) << "param_get failed";
	ASSERT_EQ(2, value) << "wrong param value";

	//ASSERT_TRUE(false) << "fail";
}
