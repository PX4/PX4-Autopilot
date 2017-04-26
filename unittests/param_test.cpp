#include <systemlib/param/param.h>

#include "gtest/gtest.h"

/*
 * These will be used in param.c if compiling for unit tests
 */
struct param_info_s	param_array[256];
struct param_info_s	*param_info_base;
struct param_info_s	*param_info_limit;

/*
 * Adds test parameters
 */
void _add_parameters()
{
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

	struct param_info_s rc_x = {
		"RC_X",
		PARAM_TYPE_INT32
	};
	rc_x.val.i = 8;

	struct param_info_s rc2_x = {
		"RC2_X",
		PARAM_TYPE_INT32
	};
	rc2_x.val.i = 16;

	param_array[0] = rc_x;
	param_array[1] = rc2_x;
	param_array[2] = test_1;
	param_array[3] = test_2;
	param_info_base = (struct param_info_s *) &param_array[0];
	// needs to point at the end of the data,
	//  therefore number of params + 1
	param_info_limit = (struct param_info_s *) &param_array[4];

}

void _assert_parameter_int_value(param_t param, int32_t expected)
{
	int32_t value;
	int result = param_get(param, &value);
	ASSERT_EQ(0, result) << printf("param_get (%i) did not return parameter\n", (int)param);
	ASSERT_EQ(expected, value) << printf("value for param (%i) doesn't match default value\n", (int)param);
}

void _set_all_int_parameters_to(int32_t value)
{
	param_set((param_t)0, &value);
	param_set((param_t)1, &value);
	param_set((param_t)2, &value);
	param_set((param_t)3, &value);

	_assert_parameter_int_value((param_t)0, value);
	_assert_parameter_int_value((param_t)1, value);
	_assert_parameter_int_value((param_t)2, value);
	_assert_parameter_int_value((param_t)3, value);
}

TEST(ParamTest, SimpleFind)
{
	_add_parameters();

	param_t param = param_find("TEST_2");
	ASSERT_NE(PARAM_INVALID, param) << "param_find did not find parameter";

	int32_t value;
	int result = param_get(param, &value);
	ASSERT_EQ(0, result) << "param_get did not return parameter";
	ASSERT_EQ(4, value) << "value of returned parameter does not match";
}

TEST(ParamTest, ResetAll)
{
	_add_parameters();
	_set_all_int_parameters_to(50);

	param_reset_all();

	_assert_parameter_int_value((param_t)0, 8);
	_assert_parameter_int_value((param_t)1, 16);
	_assert_parameter_int_value((param_t)2, 2);
	_assert_parameter_int_value((param_t)3, 4);
}

TEST(ParamTest, ResetAllExcludesOne)
{
	_add_parameters();
	_set_all_int_parameters_to(50);

	const char *excludes[] = {"RC_X"};
	param_reset_excludes(excludes, 1);

	_assert_parameter_int_value((param_t)0, 50);
	_assert_parameter_int_value((param_t)1, 16);
	_assert_parameter_int_value((param_t)2, 2);
	_assert_parameter_int_value((param_t)3, 4);
}

TEST(ParamTest, ResetAllExcludesTwo)
{
	_add_parameters();
	_set_all_int_parameters_to(50);

	const char *excludes[] = {"RC_X", "TEST_1"};
	param_reset_excludes(excludes, 2);

	_assert_parameter_int_value((param_t)0, 50);
	_assert_parameter_int_value((param_t)1, 16);
	_assert_parameter_int_value((param_t)2, 50);
	_assert_parameter_int_value((param_t)3, 4);
}

TEST(ParamTest, ResetAllExcludesBoundaryCheck)
{
	_add_parameters();
	_set_all_int_parameters_to(50);

	const char *excludes[] = {"RC_X", "TEST_1"};
	param_reset_excludes(excludes, 1);

	_assert_parameter_int_value((param_t)0, 50);
	_assert_parameter_int_value((param_t)1, 16);
	_assert_parameter_int_value((param_t)2, 2);
	_assert_parameter_int_value((param_t)3, 4);
}

TEST(ParamTest, ResetAllExcludesWildcard)
{
	_add_parameters();
	_set_all_int_parameters_to(50);

	const char *excludes[] = {"RC*"};
	param_reset_excludes(excludes, 1);

	_assert_parameter_int_value((param_t)0, 50);
	_assert_parameter_int_value((param_t)1, 50);
	_assert_parameter_int_value((param_t)2, 2);
	_assert_parameter_int_value((param_t)3, 4);
}
