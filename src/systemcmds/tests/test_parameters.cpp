#include <unit_test.h>

#include <px4_defines.h>
#include <fcntl.h>

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
		p4 = param_find("TEST_3");
	}

private:

	param_t p0{PARAM_INVALID};
	param_t p1{PARAM_INVALID};
	param_t p2{PARAM_INVALID};
	param_t p3{PARAM_INVALID};
	param_t p4{PARAM_INVALID};

	bool _assert_parameter_int_value(param_t param, int32_t expected);
	bool _assert_parameter_float_value(param_t param, float expected);

	bool _set_all_int_parameters_to(int32_t value);

	// tests on the test parameters (TEST_RC_X, TEST_RC2_X, TEST_1, TEST_2, TEST_3)
	bool SimpleFind();
	bool ResetAll();
	bool ResetAllExcludesOne();
	bool ResetAllExcludesTwo();
	bool ResetAllExcludesBoundaryCheck();
	bool ResetAllExcludesWildcard();
	bool exportImport();

	// tests on system parameters
	// WARNING, can potentially trash your system
	bool exportImportAll();
};

bool ParameterTest::_assert_parameter_int_value(param_t param, int32_t expected)
{
	int32_t value;
	int result = param_get(param, &value);
	ut_compare("param_get did not return parameter", 0, result);
	ut_compare("value for param doesn't match default value", expected, value);

	return true;
}

bool ParameterTest::_assert_parameter_float_value(param_t param, float expected)
{
	float value;
	int result = param_get(param, &value);
	ut_compare("param_get did not return parameter", 0, result);
	ut_compare_float("value for param doesn't match default value", expected, value, 0.001);

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

bool ParameterTest::exportImport()
{
	static constexpr float MAGIC_FLOAT_VAL = 0.314159f;

	bool ret = true;

	param_t test_params[] = {p0, p1, p2, p3, p4};

	// set all params to corresponding param_t value
	for (auto p : test_params) {
		if (param_type(p) == PARAM_TYPE_INT32) {
			const int32_t set_val = p;

			if (param_set_no_notification(p, &set_val) != PX4_OK) {
				PX4_ERR("param_set_no_notification failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			int32_t get_val = 0;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", p, get_val);
		}

		if (param_type(p) == PARAM_TYPE_FLOAT) {
			const float set_val = (float)p + MAGIC_FLOAT_VAL;

			if (param_set_no_notification(p, &set_val) != PX4_OK) {
				PX4_ERR("param_set_no_notification failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			float get_val = 0.0f;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", p, (float)p + MAGIC_FLOAT_VAL);
		}
	}

	// save
	if (param_save_default() != PX4_OK) {
		PX4_ERR("param_save_default failed");
		return false;
	}

	// zero all params and verify, but don't save
	for (auto p : test_params) {
		if (param_type(p) == PARAM_TYPE_INT32) {
			const int32_t set_val = 0;

			if (param_set_no_notification(p, &set_val) != PX4_OK) {
				PX4_ERR("param_set_no_notification failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			int32_t get_val = -1;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", set_val, get_val);
		}

		if (param_type(p) == PARAM_TYPE_FLOAT) {
			const float set_val = 0.0f;

			if (param_set_no_notification(p, &set_val) != PX4_OK) {
				PX4_ERR("param_set_no_notification failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			float get_val = -1.0f;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare_float("value for param doesn't match default value", set_val, get_val, 0.001f);
		}
	}

	// load saved params
	if (param_load_default() != PX4_OK) {
		PX4_ERR("param_save_default failed");
		ret = true;
	}

	// check every param
	for (auto p : test_params) {
		if (param_type(p) == PARAM_TYPE_INT32) {

			int32_t get_val = 0.0f;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", p, get_val);
		}

		if (param_type(p) == PARAM_TYPE_FLOAT) {
			float get_val = 0.0f;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare_float("value for param doesn't match default value", p, (float)p + MAGIC_FLOAT_VAL, 0.001f);
		}
	}

	return ret;
}

bool ParameterTest::exportImportAll()
{
	static constexpr float MAGIC_FLOAT_VAL = 0.217828f;

	// backup current parameters
	const char *param_file_name = PX4_ROOTFSDIR "/fs/microsd/param_backup";
	int fd = open(param_file_name, O_WRONLY | O_CREAT, PX4_O_MODE_666);

	if (fd < 0) {
		PX4_ERR("open '%s' failed (%i)", param_file_name, errno);
		return false;
	}

	int result = param_export(fd, false);

	if (result != PX4_OK) {
		PX4_ERR("param_export failed");
		close(fd);
		return false;
	}

	close(fd);

	bool ret = true;

	int N = param_count();

	// set all params to corresponding param_t value
	for (unsigned i = 0; i < N; i++) {

		param_t p = param_for_index(i);

		if (p == PARAM_INVALID) {
			PX4_ERR("param invalid: %d(%d)", p, i);
			break;
		}

		if (param_type(p) == PARAM_TYPE_INT32) {
			const int32_t set_val = p;

			if (param_set_no_notification(p, &set_val) != PX4_OK) {
				PX4_ERR("param_set_no_notification failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			int32_t get_val = 0;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", p, get_val);
		}

		if (param_type(p) == PARAM_TYPE_FLOAT) {
			const float set_val = (float)p + MAGIC_FLOAT_VAL;

			if (param_set_no_notification(p, &set_val) != PX4_OK) {
				PX4_ERR("param_set_no_notification failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			float get_val = 0.0f;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", p, (float)p + MAGIC_FLOAT_VAL);
		}
	}

	// save
	if (param_save_default() != PX4_OK) {
		PX4_ERR("param_save_default failed");
		return false;
	}

	// zero all params and verify, but don't save
	for (unsigned i = 0; i < N; i++) {
		param_t p = param_for_index(i);

		if (param_type(p) == PARAM_TYPE_INT32) {

			const int32_t set_val = 0;

			if (param_set_no_notification(p, &set_val) != PX4_OK) {
				PX4_ERR("param set failed: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			int32_t get_val = -1;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", set_val, get_val);
		}

		if (param_type(p) == PARAM_TYPE_FLOAT) {
			float set_val = 0.0f;

			if (param_set_no_notification(p, &set_val) != PX4_OK) {
				PX4_ERR("param set failed: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			float get_val = -1.0f;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", set_val, get_val);
		}
	}

	// load saved params
	if (param_load_default() != PX4_OK) {
		PX4_ERR("param_save_default failed");
		ret = true;
	}

	// check every param
	for (unsigned i = 0; i < N; i++) {
		param_t p = param_for_index(i);

		if (param_type(p) == PARAM_TYPE_INT32) {

			int32_t get_val = 0;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", p, get_val);
		}

		if (param_type(p) == PARAM_TYPE_FLOAT) {
			float get_val = 0.0f;

			if (param_get(p, &get_val) != PX4_OK) {
				PX4_ERR("param_get failed for: %d", p);
				ut_assert("param_set_no_notification failed", false);
			}

			ut_compare("value for param doesn't match default value", p, (float)p + MAGIC_FLOAT_VAL);
		}
	}

	param_reset_all();

	// restore original params
	fd = open(param_file_name, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("open '%s' failed (%i)", param_file_name, errno);
		return false;
	}

	result = param_import(fd);
	close(fd);

	if (result < 0) {
		PX4_ERR("importing from '%s' failed (%i)", param_file_name, result);
		return false;
	}

	return ret;
}

bool ParameterTest::run_tests()
{
	param_control_autosave(false);

	ut_run_test(ResetAll);
	ut_run_test(SimpleFind);
	ut_run_test(ResetAll);
	ut_run_test(ResetAllExcludesOne);
	ut_run_test(ResetAllExcludesTwo);
	ut_run_test(ResetAllExcludesBoundaryCheck);
	ut_run_test(ResetAllExcludesWildcard);
	ut_run_test(exportImport);

	// WARNING, can potentially trash your system
#ifdef __PX4_POSIX
	ut_run_test(exportImportAll);
#endif /* __PX4_POSIX */

	param_control_autosave(true);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_parameters, ParameterTest)
