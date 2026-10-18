
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
OPENCV   := $(shell pkg-config --cflags --libs opencv4)

all: site_detector

site_detector: main.cpp detector.cpp detector.hpp geometry.hpp tracker.hpp
	$(CXX) $(CXXFLAGS) -o $@ main.cpp detector.cpp $(OPENCV)


test_core: test_core.cpp geometry.hpp tracker.hpp
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror -o $@ test_core.cpp

test: test_core
	./test_core

clean:
	rm -f site_detector test_core

.PHONY: all test clean
