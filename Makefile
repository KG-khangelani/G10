CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
SRCDIR   = src
HEADERS  = $(SRCDIR)/types.h $(SRCDIR)/parser.h $(SRCDIR)/physics.h $(SRCDIR)/output.h

.PHONY: all clean run1 run2 run3 run4 run

all: level1 level2 level3 level4

level1: $(SRCDIR)/level1.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCDIR)/level1.cpp

level2: $(SRCDIR)/level2.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCDIR)/level2.cpp

level3: $(SRCDIR)/level3.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCDIR)/level3.cpp

level4: $(SRCDIR)/level4.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCDIR)/level4.cpp

run1: level1
	./level1 inputFiles/level1input.txt > submission_level1.txt

run2: level2
	./level2 inputFiles/level2input.txt > submission_level2.txt

run3: level3
	./level3 inputFiles/level3input.txt > submission_level3.txt

run4: level4
	./level4 inputFiles/level4input.txt > submission_level4.txt

run: run1 run2 run3 run4

clean:
	$(RM) level1 level2 level3 level4
