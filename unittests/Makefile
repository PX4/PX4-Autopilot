
CC=$(CXX)
CFLAGS=-I. -I../src/modules -I ../src/include -I../src/drivers \
	-I../src -I../src/lib -D__EXPORT="" -Dnullptr="0" -lm -std=c++11

# Points to the root of Google Test, relative to where this file is.
# Remember to tweak this if you move this file.
GTEST_DIR = gtest

# Flags passed to the preprocessor.
# Set Google Test's header directory as a system directory, such that
# the compiler doesn't generate warnings in Google Test headers.
CFLAGS += -isystem $(GTEST_DIR)/include

# All Google Test headers.  Usually you shouldn't change this
# definition.
GTEST_HEADERS = $(GTEST_DIR)/include/gtest/*.h \
                $(GTEST_DIR)/include/gtest/internal/*.h

# Usually you shouldn't tweak such internal variables, indicated by a
# trailing _.
GTEST_SRCS_ = $(GTEST_DIR)/src/*.cc $(GTEST_DIR)/src/*.h $(GTEST_HEADERS)

# For simplicity and to avoid depending on Google Test's
# implementation details, the dependencies specified below are
# conservative and not optimized.  This is fine as Google Test
# compiles fast and for ordinary users its source rarely changes.
gtest-all.o: $(GTEST_SRCS_)
	$(CC) $(CFLAGS) -I$(GTEST_DIR) -c \
            $(GTEST_DIR)/src/gtest-all.cc

gtest_main.o: $(GTEST_SRCS_)
	$(CC) $(CFLAGS) -I$(GTEST_DIR) -c \
            $(GTEST_DIR)/src/gtest_main.cc

gtest.a: gtest-all.o
	$(AR) $(ARFLAGS) $@ $^

gtest_main.a: gtest-all.o gtest_main.o
	$(AR) $(ARFLAGS) $@ $^


all: mixer_test sbus2_test st24_test sf0x_test

MIXER_FILES=../src/systemcmds/tests/test_mixer.cpp \
		../src/systemcmds/tests/test_conv.cpp \
		../src/modules/systemlib/mixer/mixer_simple.cpp \
		../src/modules/systemlib/mixer/mixer_multirotor.cpp \
		../src/modules/systemlib/mixer/mixer.cpp \
		../src/modules/systemlib/mixer/mixer_group.cpp \
		../src/modules/systemlib/mixer/mixer_load.c \
		../src/modules/systemlib/pwm_limit/pwm_limit.c \
		hrt.cpp \
		mixer_test.cpp

include ../src/modules/systemlib/mixer/multi_tables.mk

SBUS2_FILES=../src/modules/px4iofirmware/sbus.c \
		hrt.cpp \
		sbus2_test.cpp

ST24_FILES=../src/lib/rc/st24.c \
		hrt.cpp \
		st24_test.cpp

SF0X_FILES= \
		hrt.cpp \
		sf0x_test.cpp \
		../src/drivers/sf0x/sf0x_parser.cpp

mixer_test: $(MIXER_FILES)
	$(CC) -o mixer_test $(MIXER_FILES) $(CFLAGS)

sbus2_test: $(SBUS2_FILES)
	$(CC) -o sbus2_test $(SBUS2_FILES) $(CFLAGS)

sf0x_test: $(SF0X_FILES)
	$(CC) -o sf0x_test $(SF0X_FILES) $(CFLAGS)

st24_test: $(ST24_FILES)
	$(CC) -o st24_test $(ST24_FILES) $(CFLAGS)

cmake_gtests:
	mkdir -p build
	cd build && CC=gcc cmake .. && $(MAKE) && $(MAKE) test

unittests: clean mixer_test sbus2_test sf0x_test st24_test cmake_gtests
	./mixer_test
	./sbus2_test
	./sf0x_test
	./st24_test

.PHONY: clean
clean:
	rm -rf gtest.a gtest_main.a *.o $(ODIR)/*.o *~ core $(INCDIR)/*~ sample_unittest mixer_test sbus2_test st24_test sf0x_test build
