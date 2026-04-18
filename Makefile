CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
TARGET   = bot
SRCDIR   = src
SRCS     = $(SRCDIR)/main.cpp

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^

run: $(TARGET)
	./$(TARGET) inputFiles/level1input.txt > submission_level1.txt

clean:
	$(RM) $(TARGET)
