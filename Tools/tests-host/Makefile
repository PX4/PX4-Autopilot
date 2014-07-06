
CC=g++
CFLAGS=-I. -I../../src/modules -I ../../src/include -I../../src/drivers \
	-I../../src -I../../src/lib -D__EXPORT="" -Dnullptr="0" -lm

all: mixer_test sbus2_test autodeclination_test

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

AUTODECLINATION_FILES= ../../src/lib/geo/geo_mag_declination.c \
		hrt.cpp \
		autodeclination_test.cpp

mixer_test: $(MIXER_FILES)
	$(CC) -o mixer_test $(MIXER_FILES) $(CFLAGS)

sbus2_test: $(SBUS2_FILES)
	$(CC) -o sbus2_test $(SBUS2_FILES) $(CFLAGS)

autodeclination_test: $(SBUS2_FILES)
	$(CC) -o autodeclination_test $(AUTODECLINATION_FILES) $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ mixer_test sbus2_test autodeclination_test