#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <drivers/sf0x/sf0x_parser.h>
#include <systemlib/err.h>

#include "gtest/gtest.h"

TEST(SF0XTest, SF0X)
{
	const char _LINE_MAX = 20;
	char _linebuf[_LINE_MAX];
	_linebuf[0] = '\0';

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
			       "\n"
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
				//printf("\nparsed: %f %s\n", dist_m, (parse_ret == 0) ? "OK" : "");
			}
		}

		//printf("%s", lines[l]);

	}

	warnx("test finished");
}
