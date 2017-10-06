#include <unit_test.h>
#include <version/version.h>

class VersioningTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool _is_correct_version_tag(const char *version_tag, uint32_t result_goal);
	bool _is_correct_version_tag_vendor(const char *version_tag, uint32_t result_goal);

	bool _test_flight_version();
	bool _test_vendor_version();
};

bool VersioningTest::_is_correct_version_tag(const char *version_tag, uint32_t result_goal)
{
	uint32_t result = version_tag_to_number(version_tag);

	if (result == result_goal) {
		return true;

	} else {
		PX4_ERR("Wrong version: tag: %s, got: 0x%x, expected: 0x%x", version_tag, result, result_goal);
		return false;
	}
}

bool VersioningTest::_is_correct_version_tag_vendor(const char *version_tag, uint32_t result_goal)
{
	uint32_t result = version_tag_to_vendor_version_number(version_tag);

	if (result == result_goal) {
		return true;

	} else {
		PX4_ERR("Wrong version: tag: %s, got: 0x%x, expected: 0x%x", version_tag, result, result_goal);
		return false;
	}
}

bool VersioningTest::run_tests()
{
	ut_run_test(_test_flight_version);
	ut_run_test(_test_vendor_version);

	return (_tests_failed == 0);
}

bool VersioningTest::_test_flight_version()
{
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3", 0xB2D63ff));
	ut_assert_true(_is_correct_version_tag("v1.2.3", 0x010203ff));
	ut_assert_true(_is_correct_version_tag("v255.255.255", 0xffffffff));
	ut_assert_true(_is_correct_version_tag("v255.255.255-11", 0xffffff00));
	ut_assert_true(_is_correct_version_tag("v1.2.3-111", 0x01020300));
	ut_assert_true(_is_correct_version_tag("v1.2.3-11-abababab", 0x01020300));
	ut_assert_true(_is_correct_version_tag("11.45.99-1.2.3", 0x0B2D63ff));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3rc3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3rc4", 0x0B2D63C0));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3alpha3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3alpha4", 0x0B2D6340));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3beta3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3beta4", 0x0B2D6380));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3dev4", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag("v11.45.99-1.2.3dev3-7-g7e282f57", 0xB2D6300));
	ut_assert_true(_is_correct_version_tag("0.45.99-1.2.3beta4", 0x002D6380));
	ut_assert_true(_is_correct_version_tag("0.0.0-1.2.3beta4", 0x00000080));
	ut_assert_true(_is_correct_version_tag("0.0.0-1.2.3dev4", 0x00000000));
	ut_assert_true(_is_correct_version_tag("v1.6.2-1.0.0", 0x010602ff));
	ut_assert_true(_is_correct_version_tag("v1.6.2-1.0.0rc2", 0x010602C0));
	ut_assert_true(_is_correct_version_tag("v1.6.2-1.0.0-rc2", 0x010602C0));
	ut_assert_true(_is_correct_version_tag("v1.6.2-1.0.0-rc2-abababab", 0x01060200));
	ut_assert_true(_is_correct_version_tag("v1.6.2-rc2", 0x010602C0));
	ut_assert_true(_is_correct_version_tag("v1.6.2rc1", 0x010602C0));
	ut_assert_true(_is_correct_version_tag("v1.6.10-100-g890c415", 0x01060A00));
	ut_assert_true(_is_correct_version_tag("v1.6.2-0.8.7-67-g1d5e979", 0x01060200));

	return true;
}

bool VersioningTest::_test_vendor_version()
{
	ut_assert_true(_is_correct_version_tag_vendor("alpha", 0x00000000));
	ut_assert_true(_is_correct_version_tag_vendor("alpha23", 0x00000000));
	ut_assert_true(_is_correct_version_tag_vendor("v11.45.99-34.56.88", 0x223858FF));
	ut_assert_true(_is_correct_version_tag_vendor("v11.45.99-1.2.3", 0x010203FF));
	ut_assert_true(_is_correct_version_tag_vendor("1.2.3-11.45.99", 0x0B2D63FF));
	ut_assert_true(_is_correct_version_tag_vendor("v1.6.2-1.0.0", 0x010000FF));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-255.255.255", 0xFFFFFFFF));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-255.255.255-11", 0xFFFFFFFF));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3", 0x000000FF));
	ut_assert_true(_is_correct_version_tag_vendor("v1.6.2-rc2", 0x000000C0));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11", 0x00000000));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45", 0x00000000));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11-abababab", 0x00000000));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99rc3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99rc4", 0x0B2D63C0));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99alpha3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99alpha4", 0x0B2D6340));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99beta3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99beta4", 0x0B2D6380));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99dev4", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag_vendor("v1.2.3-11.45.99dev3-7-g7e282f57", 0x0B2D6300));
	ut_assert_true(_is_correct_version_tag_vendor("v1.6.2-1.0.0-rc2-abababab", 0x01000000));
	ut_assert_true(_is_correct_version_tag_vendor("v1.6.2-1.0.0-rc2-23-abababab", 0x010000C0));
	ut_assert_true(_is_correct_version_tag_vendor("v1.6.2-0.8.7-67-g1d5e979", 0x00080700));
	ut_assert_true(_is_correct_version_tag_vendor("v1.6.0-100-g890c415", 0x00000000));
	ut_assert_true(_is_correct_version_tag_vendor("1.2.3-0.45.99beta4", 0x002D6380));
	ut_assert_true(_is_correct_version_tag_vendor("1.2.3-0.0.0beta4", 0x00000080));
	ut_assert_true(_is_correct_version_tag_vendor("1.2.3-0.0.0dev4", 0x00000000));

	return true;
}

ut_declare_test_c(test_versioning, VersioningTest);
