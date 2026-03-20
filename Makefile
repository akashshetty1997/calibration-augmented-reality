# Name: Akash Shridhar Shetty, Skandhan Madhusudhana
# Date: March 2026
# File: Makefile
#
# Purpose:
# Build system for the Calibration and Augmented Reality project.
# Links calibration and ar modules with OpenCV.

CXX = g++
CXXFLAGS = -std=c++17 -Wall $(shell pkg-config --cflags opencv4)
LDFLAGS = $(shell pkg-config --libs opencv4)

all: ar_project feature_detection

ar_project: src/main.cpp src/calibration.cpp src/ar.cpp
	$(CXX) $(CXXFLAGS) -o ar_project src/main.cpp src/calibration.cpp src/ar.cpp $(LDFLAGS)

feature_detection: src/feature_detection.cpp
	$(CXX) $(CXXFLAGS) -o feature_detection src/feature_detection.cpp $(LDFLAGS)

clean:
	rm -f ar_project feature_detection *.png