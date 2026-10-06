CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O3 -Wall -Wextra -Wpedantic
PYTHON   ?= python3
BUILD    := build
SYN_MAP  := data/synthetic_rooms.map

# Query used in the course report on 64room_005 (x y x y).
QUERY    := 3 510 490 496
PARAMS   := --k 50 --probe-budget 500 --weighted-w 1.5

.PHONY: all test test-cpp test-py bench-synthetic clean

all: $(BUILD)/dash_bench $(BUILD)/dash_tests

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/dash_bench: cpp/src/bench.cpp cpp/include/dash.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -Icpp/include $< -o $@

$(BUILD)/dash_tests: cpp/tests/test_dash.cpp cpp/include/dash.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -Icpp/include $< -o $@

test: test-cpp test-py

test-cpp: $(BUILD)/dash_tests
	./$(BUILD)/dash_tests

test-py:
	cd python && $(PYTHON) -m unittest discover -s tests -v

$(SYN_MAP): scripts/make_room_map.py
	$(PYTHON) scripts/make_room_map.py --out $@

# Reproducible benchmark on a generated room map (not the MovingAI file).
bench-synthetic: $(BUILD)/dash_bench $(SYN_MAP)
	./$(BUILD)/dash_bench $(SYN_MAP) $(QUERY) $(PARAMS)

clean:
	rm -rf $(BUILD)
