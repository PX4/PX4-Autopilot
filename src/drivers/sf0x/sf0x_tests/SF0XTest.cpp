#include <unit_test/unit_test.h>

#include <drivers/sf0x/sf0x_parser.h>

#include <systemlib/err.h>

#include <stdio.h>
#include <unistd.h>

extern "C" __EXPORT int sf0x_tests_main(int argc, char *argv[]);

class SF0XTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool sf0xTest();
};

bool SF0XTest::run_tests()
{
	ut_run_test(sf0xTest);

	return (_tests_failed == 0);
}

bool SF0XTest::sf0xTest()
{
	const char _LINE_MAX = 20;
	//char _linebuf[_LINE_MAX];
	//_linebuf[0] = '\0';

	const char *lines[] = {"0.01\r\n",
			       "0.02\r\n",
			       "0.03\r\n",
			       "0.04\r\n",
			       "0",
			       ".",
			       "0",
			       "5",
			       "\r",
			       "\n",
			       "0",
			       "3\r",
			       "\n",
			       "\r\n",
			       "0.06",
			       "\r\n"
			      };

	enum SF0X_PARSE_STATE state = SF0X_PARSE_STATE0_UNSYNC;
	float dist_m;
	char _parserbuf[_LINE_MAX];
	unsigned _parsebuf_index = 0;

	for (unsigned l = 0; l < sizeof(lines) / sizeof(lines[0]); l++) {
		//printf("\n%s", _linebuf);

		int parse_ret;

		for (int i = 0; i < strlen(lines[l]); i++) {
			parse_ret = sf0x_parser(lines[l][i], _parserbuf, &_parsebuf_index, &state, &dist_m);

			if (parse_ret == 0) {
				if (l == 0) {
					ut_test(dist_m - 0.010000f < 0.001f);

				} else if (l == 1) {
					ut_test(dist_m - 0.020000f < 0.001f);

				} else if (l == 2) {
					ut_test(dist_m - 0.030000f < 0.001f);

				} else if (l == 3) {
					ut_test(dist_m - 0.040000f < 0.001f);
				}

				//printf("\nparsed: %f %s\n", dist_m, (parse_ret == 0) ? "OK" : "");
			}
		}

		//printf("%s", lines[l]);

	}

	return true;
}

ut_declare_test_c(sf0x_tests_main, SF0XTest)

