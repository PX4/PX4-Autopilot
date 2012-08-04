CFLAGS = -g -Wall -Werror -Os
TESTPROTOCOL = common
ALLPROTOCOLS = minimal test common pixhawk ardupilotmega slugs ualberta

all: 
	for p in ${ALLPROTOCOLS}; do make -f Makefile build TESTPROTOCOL=$$p; done

test:
	for p in ${ALLPROTOCOLS}; do make -f Makefile testprogs TESTPROTOCOL=$$p || exit 1; done

valgrindtest:
	for p in ${ALLPROTOCOLS}; do make -f Makefile valgrindprogs TESTPROTOCOL=$$p || exit 1; done

build: testmav0.9_${TESTPROTOCOL} testmav1.0_${TESTPROTOCOL}

testprogs: testmav0.9_${TESTPROTOCOL} testmav1.0_${TESTPROTOCOL}
	./testmav0.9_${TESTPROTOCOL}
	./testmav1.0_${TESTPROTOCOL}

valgrindprogs: testmav0.9_${TESTPROTOCOL} testmav1.0_${TESTPROTOCOL}
	valgrind -q ./testmav0.9_${TESTPROTOCOL}
	valgrind -q ./testmav1.0_${TESTPROTOCOL}

clean:
	rm -rf *.o *~ testmav1.0* testmav0.9*

testmav1.0_${TESTPROTOCOL}: testmav.c
	$(CC) $(CFLAGS) -I../../include_v1.0 -I../../include_v1.0/${TESTPROTOCOL} -o $@ testmav.c

testmav0.9_${TESTPROTOCOL}: testmav.c
	$(CC) $(CFLAGS) -I../../include_v0.9 -I../../include_v0.9/${TESTPROTOCOL} -o $@ testmav.c
