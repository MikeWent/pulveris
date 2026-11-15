CXX ?= g++
CPPFLAGS += -D_TIME_BITS=64 -D_FILE_OFFSET_BITS=64 -I/usr/include/opencv4 $(shell pkg-config --cflags cpp-httplib libgpiod)
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Wno-psabi -pthread
LDLIBS += -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core $(shell pkg-config --libs cpp-httplib libgpiod) -pthread

SOURCES = native/main.cpp native/vision.cpp native/control.cpp native/gpio.cpp native/web.cpp native/api.cpp
OBJECTS = $(SOURCES:.cpp=.o)

.PHONY: all test clean
all: pulveris

pulveris: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@.tmp $^ $(LDLIBS)
	mv $@.tmp $@

native/%.o: native/%.cpp native/core.hpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

native/gpio.o native/test.o: native/debounce.hpp native/blink.hpp

# The HTTP/JSON headers are expensive to optimize on a 256 MB NanoPi.
native/web.o native/api.o native/web_test.o: CXXFLAGS += -O0

native-test: native/test.o native/vision.o native/control.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

native-web-test: native/web_test.o native/web.o native/api.o native/control.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

test: native-test native-web-test
	./native-test
	./native-web-test

clean:
	$(RM) pulveris pulveris.tmp native-test native-web-test native/*.o
