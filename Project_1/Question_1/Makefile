CXX ?= g++

CXXFLAGS += -c -Wall $(shell pkg-config --cflags opencv)
LDFLAGS += $(shell pkg-config --libs --static opencv)
SOURCES = main.cpp dlt.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = image_rect

all: $(SOURCES) $(EXECUTABLE) clean

$(EXECUTABLE): $(OBJECTS); $(CXX) $(OBJECTS) -o $@ $(LDFLAGS)

%.o: %.cpp; $(CXX) $< -o $@ $(CXXFLAGS)

clean: ; rm -f main.o dlt.o
