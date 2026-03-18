# Name: Akash Shridhar Shetty, Skandhan Madhusudhana
# Date: March 2026
# File: Makefile
#
# Purpose:
# Build system for the Calibration and Augmented Reality project.
# Links calibration and ar modules with OpenCV.

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall -I./include $(shell pkg-config --cflags opencv4)
LIBS = $(shell pkg-config --libs opencv4)

# Target executable name
TARGET = ar_project

# Source files (Now including ar.cpp)
SRCS = src/main.cpp src/calibration.cpp src/ar.cpp
OBJS = $(SRCS:.cpp=.o)

# Default rule
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

# Rule for object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean