#include <unit_test/unit_test.h>

class ParameterTest : public UnitTest
{
public:
	virtual bool run_tests();

	ParameterTest()
	{
		p0 = param_find("TEST_RC_X");
		p1 = param_find("TEST_RC2_X");
		p2 = param_find("TEST_1");
		p3 = param_find("TEST_2");
	}

private:

	param_t p0{PARAM_INVALID};
	param_t p1{PARAM_INVALID};
	param_t p2{PARAM_INVALID};
	param_t p3{PARAM_INVALID};

	bool _assert_parameter_int_value(param_t param, int32_t expected);
	bool _set_all_int_parameters_to(int32_t value);

	bool SimpleFind();
	bool ResetAll();
	bool ResetAllExcludesOne();
	bool ResetAllExcludesTwo();
	bool ResetAllExcludesBoundaryCheck();
	bool ResetAllExcludesWildcard();
};

bool ParameterTest::_assert_parameter_int_value(param_t param, int32_t expected)
{
	int32_t value;
	int result = param_get(param, &value);
	ut_compare("param_get did not return parameter", 0, result);
	ut_compare("value for param doesn't match default value", expected, value);

	return true;
}

bool ParameterTest::_set_all_int_parameters_to(int32_t value)
{
	param_set(p0, &value);
	param_set(p1, &value);
	param_set(p2, &value);
	param_set(p3, &value);

	bool ret = false;

	ret = ret || _assert_parameter_int_value(p0, value);
	ret = ret || _assert_parameter_int_value(p1, value);
	ret = ret || _assert_parameter_int_value(p2, value);
	ret = ret || _assert_parameter_int_value(p3, value);

	return ret;
}

bool ParameterTest::SimpleFind()
{
	param_t param = param_find("TEST_2");

	ut_assert_true(PARAM_INVALID != param);

	int32_t value;
	int result = param_get(param, &value);

	ut_compare("param_get did not return parameter", 0, result);
	ut_compare("value of returned parameter does not match", 4, value);

	return true;
}

bool ParameterTest::ResetAll()
{
	_set_all_int_parameters_to(50);

	param_reset_all();

	bool ret = false;

	ret = ret || _assert_parameter_int_value(p0, 8);
	ret = ret || _assert_parameter_int_value(p1, 16);
	ret = ret || _assert_parameter_int_value(p2, 2);
	ret = ret || _assert_parameter_int_value(p3, 4);

	return ret;
}

bool ParameterTest::ResetAllExcludesOne()
{
	_set_all_int_parameters_to(50);

	const char *excludes[] = {"TEST_RC_X"};
	param_reset_excludes(excludes, 1);

	bool ret = false;

	ret = ret || _assert_parameter_int_value(p0, 50);
	ret = ret || _assert_parameter_int_value(p1, 16);
	ret = ret || _assert_parameter_int_value(p2, 2);
	ret = ret || _assert_parameter_int_value(p3, 4);

	return ret;
}

bool ParameterTest::ResetAllExcludesTwo()
{
	_set_all_int_parameters_to(50);

	const char *excludes[] = {"TEST_RC_X", "TEST_1"};
	param_reset_excludes(excludes, 2);

	bool ret = false;

	ret = ret || _assert_parameter_int_value(p0, 50);
	ret = ret || _assert_parameter_int_value(p1, 16);
	ret = ret || _assert_parameter_int_value(p2, 50);
	ret = ret || _assert_parameter_int_value(p3, 4);

	return ret;
}

bool ParameterTest::ResetAllExcludesBoundaryCheck()
{
	_set_all_int_parameters_to(50);

	const char *excludes[] = {"TEST_RC_X", "TEST_1"};
	param_reset_excludes(excludes, 1);

	bool ret = false;

	ret = ret || _assert_parameter_int_value(p0, 50);
	ret = ret || _assert_parameter_int_value(p1, 16);
	ret = ret || _assert_parameter_int_value(p2, 2);
	ret = ret || _assert_parameter_int_value(p3, 4);

	return ret;
}

bool ParameterTest::ResetAllExcludesWildcard()
{
	_set_all_int_parameters_to(50);

	const char *excludes[] = {"TEST_RC*"};
	param_reset_excludes(excludes, 1);

	bool ret = false;

	ret = ret || _assert_parameter_int_value(p0, 50);
	ret = ret || _assert_parameter_int_value(p1, 50);
	ret = ret || _assert_parameter_int_value(p2, 2);
	ret = ret || _assert_parameter_int_value(p3, 4);

	return ret;
}

bool ParameterTest::run_tests()
{
	ut_run_test(SimpleFind);
	ut_run_test(ResetAll);
	ut_run_test(ResetAllExcludesOne);
	ut_run_test(ResetAllExcludesTwo);
	ut_run_test(ResetAllExcludesBoundaryCheck);
	ut_run_test(ResetAllExcludesWildcard);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_parameters, ParameterTest)
