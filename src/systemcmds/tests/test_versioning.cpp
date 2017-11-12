#include <unit_test.h>
#include <version/version.h>

class VersioningTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool _test_tag_to_version_number(const char *version_tag, uint32_t flight_version_target,
					 uint32_t vendor_version_target);
};

bool VersioningTest::_test_tag_to_version_number(const char *version_tag, uint32_t flight_version_target,
		uint32_t vendor_version_target)
{
	uint32_t flight_version_result = version_tag_to_number(version_tag);
	uint32_t vendor_version_result = version_tag_to_vendor_version_number(version_tag);

	if (flight_version_target == flight_version_result) {
		if (vendor_version_target == vendor_version_result) {
			return true;

		} else {
			PX4_ERR("Wrong vendor version: tag: %s, got: 0x%x, expected: 0x%x", version_tag, vendor_version_result,
				vendor_version_target);
			return false;
		}

	} else {
		PX4_ERR("Wrong flight version: tag: %s, got: 0x%x, expected: 0x%x", version_tag, flight_version_result,
			flight_version_target);
		return false;
	}
}

bool VersioningTest::run_tests()
{
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3", 					0x0B2D63FF, 0x010203FF));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-01.02.03", 				0x0B2D63FF, 0x010203FF));
	ut_assert_true(_test_tag_to_version_number("v011.045.099-001.002.003", 			0x0B2D63FF, 0x010203FF));
	ut_assert_true(_test_tag_to_version_number("v011.045.099-1.2.3",	 			0x0B2D63FF, 0x010203FF));
	ut_assert_true(_test_tag_to_version_number("v1.2.3", 							0x010203FF, 0x000000FF));
	ut_assert_true(_test_tag_to_version_number("v255.255.255", 						0xFFFFFFFF, 0x000000FF));
	ut_assert_true(_test_tag_to_version_number("v255.255.255-11", 					0xFFFFFF00, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v255.255.255-255.255.255", 			0xFFFFFFFF, 0xFFFFFFFF));
	ut_assert_true(_test_tag_to_version_number("v255.255.255-0.0.0", 				0xFFFFFFFF, 0x000000FF));
	ut_assert_true(_test_tag_to_version_number("v0.0.0-0.0.0", 						0x000000FF, 0x000000FF));
	ut_assert_true(_test_tag_to_version_number("v0.0.0-255.255.255", 				0x000000FF, 0xFFFFFFFF));
	ut_assert_true(_test_tag_to_version_number("v1.2.3-111", 						0x01020300, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.2.3-11-gabababab", 				0x01020300, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("11.45.99-1.2.3", 					0x0B2D63FF, 0x010203FF));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3rc3-7-g7e282f57", 	0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3-rc3-7-g7e282f57",	0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3rc4", 				0x0B2D63C0, 0x010203C0));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3alpha3-7-g7e282f57", 0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3alpha4", 			0x0B2D6340, 0x01020340));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3beta3-7-g7e282f57", 	0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3beta4", 				0x0B2D6380, 0x01020380));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3dev4", 				0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3dev3-7-g7e282f57", 	0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("0.45.99-1.2.3beta4", 				0x002D6380, 0x01020380));
	ut_assert_true(_test_tag_to_version_number("0.0.0-1.2.3beta4", 					0x00000080, 0x01020380));
	ut_assert_true(_test_tag_to_version_number("0.0.0-1.2.3dev4", 					0x00000000, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-1.0.0", 						0x010602FF, 0x010000FF));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-1.0.0rc2", 					0x010602C0, 0x010000C0));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-1.0.0-rc2", 					0x010602C0, 0x010000C0));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-1.0.0-rc2-gabababab", 		0x01060200, 0x01000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-rc2", 						0x010602C0, 0x000000C0));
	ut_assert_true(_test_tag_to_version_number("v1.6.2rc1", 						0x010602C0, 0x000000C0));
	ut_assert_true(_test_tag_to_version_number("v1.6.10-100-g890c415", 				0x01060A00, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.10-99999999999999-g890c415",	0x01060A00, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-0.8.7-67-g1d5e979", 			0x01060200, 0x00080700));
	ut_assert_true(_test_tag_to_version_number("randomtext", 						0x00000000, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("randomtextwithnumber12", 			0x00000000, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("12randomtextwithnumber", 			0x00000000, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("12.12-randomtextwithnumber", 		0x00000000, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v12.12-randomtextwithnumber", 		0x00000000, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("...-...", 							0x00000000, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v...-...", 							0x00000000, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.2.3-dirty", 						0x01020300, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3-dirty", 			0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v1.2.3-111-dirty", 					0x01020300, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.2.3-11-gabababab-dirty", 		0x01020300, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.2.3-11-g1d5e979-dirty", 			0x01020300, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.4.9-1.2.3rc3-7-g7e282f57-dirty", 0x01040900, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v1.4.9-1.2.3-rc3-7-g7e282f57-dirty", 0x01040900, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v1.4.9-1.2.3alpha-7-g7e282f5-dirty", 0x01040900, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3alpha4-dirty", 		0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v1.4.9-1.2.3beta-7-g7e282f57-dirty", 0x01040900, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3beta4-dirty", 		0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v11.45.99-1.2.3dev4-dirty", 		0x0B2D6300, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v1.4.9-1.2.3dev3-7-g7e282f57-dirty", 0x01040900, 0x01020300));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-1.0.0rc2-dirty", 			0x01060200, 0x01000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-1.0.0-rc2-dirty", 			0x01060200, 0x01000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-1.0.0-rc2-gabababab-dirty", 	0x01060200, 0x01000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-1.0.0rc2-gabababab-dirty", 	0x01060200, 0x01000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-rc2-dirty", 					0x01060200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2rc1-dirty", 					0x01060200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2rc1-67-g1d5e979-dirty", 		0x01060200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-rc1-67-g1d5e979-dirty", 		0x01060200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.6.2-0.8.7-67-g1d5e979-dirty", 	0x01060200, 0x00080700));
	ut_assert_true(_test_tag_to_version_number("v1.8.2alpha4-dirty", 				0x01080200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.8.2alpha4-67-g1d5e979-dirty", 	0x01080200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.8.2beta4-dirty", 				0x01080200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.8.2beta4-67-g1d5e979-dirty", 	0x01080200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.8.2dev4-dirty", 					0x01080200, 0x00000000));
	ut_assert_true(_test_tag_to_version_number("v1.8.2dev4-67-g1d5e979-dirty", 		0x01080200, 0x00000000));

	return (_tests_failed == 0);
}

ut_declare_test_c(test_versioning, VersioningTest);
