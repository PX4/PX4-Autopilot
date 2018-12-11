
.PHONY: all tests clean
all: test_mixer_multirotor

test_mixer_multirotor: test_mixer_multirotor.cpp mixer_multirotor.cpp mixer.cpp
	@g++ $^ -std=c++11 -I .. -DMIXER_MULTIROTOR_USE_MOCK_GEOMETRY -o $@

tests: test_mixer_multirotor
	@echo "Testing Mixer Multirotor"
	@python mixer_multirotor.py --test --mixer-multirotor-binary ./$^

clean:
	@rm test_mixer_multirotor
