
CC=g++
CFLAGS=-I. -I../../src/modules -I ../../src/include -I../../src/drivers \
	-I../../src -I../../src/lib -D__EXPORT="" -Dnullptr="0" -lm

all: mixer_test sbus2_test autodeclination_test st24_test sf0x_test

MIXER_FILES=../../src/systemcmds/tests/test_mixer.cpp \
		../../src/systemcmds/tests/test_conv.cpp \
		../../src/modules/systemlib/mixer/mixer_simple.cpp \
		../../src/modules/systemlib/mixer/mixer_multirotor.cpp \
		../../src/modules/systemlib/mixer/mixer.cpp \
		../../src/modules/systemlib/mixer/mixer_group.cpp \
		../../src/modules/systemlib/mixer/mixer_load.c \
		../../src/modules/systemlib/pwm_limit/pwm_limit.c \
		hrt.cpp \
		mixer_test.cpp

SBUS2_FILES=../../src/modules/px4iofirmware/sbus.c \
		hrt.cpp \
		sbus2_test.cpp

ST24_FILES=../../src/lib/rc/st24.c \
		hrt.cpp \
		st24_test.cpp

SF0X_FILES= \
		hrt.cpp \
		sf0x_test.cpp \
		../../src/drivers/sf0x/sf0x_parser.cpp

AUTODECLINATION_FILES= ../../src/lib/geo_lookup/geo_mag_declination.c \
		hrt.cpp \
		autodeclination_test.cpp

mixer_test: $(MIXER_FILES)
	$(CC) -o mixer_test $(MIXER_FILES) $(CFLAGS)

sbus2_test: $(SBUS2_FILES)
	$(CC) -o sbus2_test $(SBUS2_FILES) $(CFLAGS)

sf0x_test: $(SF0X_FILES)
	$(CC) -o sf0x_test $(SF0X_FILES) $(CFLAGS)

autodeclination_test: $(SBUS2_FILES)
	$(CC) -o autodeclination_test $(AUTODECLINATION_FILES) $(CFLAGS)

st24_test: $(ST24_FILES)
	$(CC) -o st24_test $(ST24_FILES) $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ mixer_test sbus2_test autodeclination_test st24_test sf0x_test
