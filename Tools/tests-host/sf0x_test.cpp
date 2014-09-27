
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

enum SF0X_PARSE_STATE {
	SF0X_PARSE_STATE0_UNSYNC = 0,
	SF0X_PARSE_STATE1_SYNC,
	SF0X_PARSE_STATE2_GOT_DIGIT0,
	SF0X_PARSE_STATE3_GOT_DOT,
	SF0X_PARSE_STATE4_GOT_DIGIT1,
	SF0X_PARSE_STATE5_GOT_DIGIT2,
	SF0X_PARSE_STATE6_GOT_CARRIAGE_RETURN
};

int sf0x_parser(char c, char *parserbuf, unsigned *parserbuf_index, enum *SF0X_PARSE_STATE state, float *dist)
{
	int ret = -1;

	unsigned len = strlen(parserbuf);
	switch (state) {
		case SF0X_PARSE_STATE0_UNSYNC:
			if (c == '\n') {
				*state = SF0X_PARSE_STATE1_SYNC;
				*parserbuf_index = 0;
			}
			break;

		case SF0X_PARSE_STATE1_SYNC:
			if (c >= '0' && c <= '9') {
				*state = SF0X_PARSE_STATE2_GOT_DIGIT0;
				parserbuf[parserbuf_index] = c;
				parserbuf_index++;
			}
			break;

		case SF0X_PARSE_STATE2_GOT_DIGIT0:
			if (c >= '0' && c <= '9') {
				*state = SF0X_PARSE_STATE2_GOT_DIGIT0;
				parserbuf[parserbuf_index] = c;
				parserbuf_index++;
			} else if (c == '.') {
				*state = SF0X_PARSE_STATE3_GOT_DOT;
				parserbuf[parserbuf_index] = c;
				parserbuf_index++;
			} else {
				*state = SF0X_PARSE_STATE0_UNSYNC;
			}
			break;

		case SF0X_PARSE_STATE3_GOT_DOT:
			if (c >= '0' && c <= '9') {
				*state = SF0X_PARSE_STATE4_GOT_DIGIT1;
				parserbuf[parserbuf_index] = c;
				parserbuf_index++;
			} else {
				*state = SF0X_PARSE_STATE0_UNSYNC;
			}
			break;

		case SF0X_PARSE_STATE4_GOT_DIGIT1:
			if (c >= '0' && c <= '9') {
				*state = SF0X_PARSE_STATE4_GOT_DIGIT2;
				parserbuf[parserbuf_index] = c;
				parserbuf_index++;
			} else {
				*state = SF0X_PARSE_STATE0_UNSYNC;
			}
			break;

		case SF0X_PARSE_STATE5_GOT_DIGIT2:
			if (c == '\r') {
				*state = SF0X_PARSE_STATE6_GOT_CARRIAGE_RETURN;
			} else {
				*state = SF0X_PARSE_STATE0_UNSYNC;
			}
			break;

		case SF0X_PARSE_STATE6_GOT_CARRIAGE_RETURN:
			if (c == '\n') {
				parserbuf[parserbuf_index] = '\0';
				*dist = strtod(linebuf, &end);
				*state = SF0X_PARSE_STATE0_SYNC;
				*parserbuf_index = 0;
				ret = 0;
			} else {
				*state = SF0X_PARSE_STATE0_UNSYNC;
			}
			break;
	}



	return ret;
}

int main(int argc, char *argv[]) {
	warnx("SF0X test started");

	if (argc < 2)
		errx(1, "Need a filename for the input file");

	warnx("loading data from: %s", argv[1]);

	FILE *fp;
	
	fp = fopen(argv[1],"rt");

	if (!fp)
		errx(1, "failed opening file");

	int ret = 0;

	const char LINE_MAX = 20;
	char _linebuf[LINE_MAX];
	_linebuf[0] = '\0';

	char *end;

	enum SF0X_PARSE_STATE state = SF0X_PARSE_STATE0_UNSYNC;
	float dist_m;
	char _parserbuf[LINE_MAX];
	unsigned _parsebuf_index = 0;

	while (fgets(_linebuf, LINE_MAX, fp) != NULL) {

		printf("\n%s", _linebuf);

		int parse_ret;

		for (int i = 0; i < strlen(_linebuf); i++)
		{
			printf("%0x ", _linebuf[i]);
			parse_ret = sf0x_parser(_linebuf[i], _parserbuf, &_parsebuf_index, &state, &dist_m);

			if (parse_ret == 0) {
				printf("PARSED!");
			}
		}

		printf("\nparsed: %f %s\n", dist_m, (parse_ret == 0) ? "OK" : "");
	}

	// Init the parser
	

	if (ret == EOF) {
		warnx("Test finished, reached end of file");
	} else {
		warnx("Test aborted, errno: %d", ret);
	}

	return ret;
}
