
CC=g++
CFLAGS=-I. -I../../src/modules -I ../../src/include -I../../src/drivers -I../../src -D__EXPORT="" -Dnullptr="0"

ODIR=obj
LDIR =../lib

LIBS=-lm

#_DEPS = test.h
#DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = mixer_test.o test_mixer.o mixer_simple.o mixer_multirotor.o \
	mixer.o mixer_group.o mixer_load.o test_conv.o pwm_limit.o hrt.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

#$(DEPS)
$(ODIR)/%.o: %.cpp
	mkdir -p obj
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.o: ../../src/systemcmds/tests/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.o: ../../src/modules/systemlib/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.o: ../../src/modules/systemlib/mixer/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.o: ../../src/modules/systemlib/pwm_limit/%.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.o: ../../src/modules/systemlib/pwm_limit/%.c
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.o: ../../src/modules/systemlib/mixer/%.c
	$(CC) -c -o $@ $< $(CFLAGS)

#
mixer_test: $(OBJ)
	g++ -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~