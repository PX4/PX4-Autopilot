#include <systemlib/visibility.h>
#include <systemlib/param/param.h>

#include "gtest/gtest.h"


struct param_info_s	param_array[256];
struct param_info_s	*param_info_base;
struct param_info_s	*param_info_limit;

/*
 * Adds test parameters
 */
void _add_parameters() {
	struct param_info_s test_1 = {
		"TEST_1",
		PARAM_TYPE_INT32
	};
	test_1.val.i = 2;

	struct param_info_s test_2 = {
		"TEST_2",
		PARAM_TYPE_INT32
	};
	test_2.val.i = 4;

	param_array[0] = test_1;
	param_array[1] = test_2;
	param_info_base = (struct param_info_s *) &param_array[0];
	param_info_limit = (struct param_info_s *) &param_array[2];
}

TEST(ParamTest, SimpleFind) {
	_add_parameters();

	printf("diff: %i\n", (unsigned)(param_info_limit - param_info_base));
	
	param_t param = param_find("TEST_2");
	ASSERT_NE(PARAM_INVALID, param) << "param_find did not find parameter";

	int32_t value;
	int result = param_get(param, &value);
	ASSERT_EQ(0, result) << "param_get did not return parameter";
	ASSERT_EQ(4, value) << "value of returned parameter does not match";
}


