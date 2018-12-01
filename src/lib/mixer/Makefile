
.PHONY: all tests
all: test_mixer_multirotor

test_mixer_multirotor: test_mixer_multirotor.cpp mixer_multirotor.cpp mixer.cpp
	g++ $^ -I .. -DMIXER_MULTIROTOR_USE_MOCK_GEOMETRY -o $@

tests: test_mixer_multirotor
	python mixer_multirotor.py --test --mixer-multirotor-binary ./$^
