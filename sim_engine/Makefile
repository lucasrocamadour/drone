CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -pedantic -Ithird_party

ENGINE_SOURCES := engine.cpp
BATCH_SOURCES := batch.cpp $(ENGINE_SOURCES)

.PHONY: all build batch run-batch clean

all: batch

build: batch

batch: batch.bin

batch.bin: $(BATCH_SOURCES)
	$(CXX) $(CXXFLAGS) $(BATCH_SOURCES) -o batch.bin

run-batch: batch.bin
	./batch.bin

clean:
	rm -f batch.bin
