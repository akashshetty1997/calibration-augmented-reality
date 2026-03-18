# Project 4: Camera Calibration and Augmented Reality

**Name:** Akash Shridhar Shetty, Skandhan Madhusudhana  
**Date:** March 2026  
**Course:** CS 5330 - Pattern Recognition & Computer Vision  



## Overview
This project implements a complete pipeline for camera calibration and real-time Augmented Reality (AR). The system calibrates a camera using a checkerboard target to determine intrinsic parameters and then uses those parameters to estimate the camera's 3D pose, allowing for the projection of virtual 3D objects onto the scene.

## Key Features
* **Target Detection:** Robust detection of a 9x6 checkerboard pattern with sub-pixel refinement.
* **Calibration Engine:** Calculates camera matrix and distortion coefficients using at least 5 calibration frames.
* **Pose Estimation:** Real-time 3D rotation and translation calculation using `cv::solvePnP`.
* **AR Rendering:** * 3D Coordinate Axes (X: Red, Y: Green, Z: Blue).
    * Complex Virtual Object (3D Wireframe Cube/House).
* **Persistent Storage:** Saves and loads calibration parameters from `intrinsic_params.yml`.



## Setup & Installation

### Prerequisites
* OpenCV 4.x
* C++ Compiler (g++)
* `pkg-config`

### Compilation
Use the provided `Makefile` to build the project:
```bash
make
```

### Running the Application
Ensure your webcam is connected and run the executable:
```bash
./ar_project
```



## Usage Instructions

| Key | Action |
| : | : |
| **'s'** | **Save Frame:** Stores current checkerboard corners for calibration. |
| **'c'** | **Calibrate:** Runs the calibration math (requires 5+ saved frames). |
| **'w'** | **Write File:** Saves the calibrated parameters to `intrinsic_params.yml`. |
| **'p'** | **Photo:** Saves a screenshot for the project report. |
| **'q'** | **Quit:** Exits the application. |



## Project Structure
* `src/main.cpp`: Main video loop and user input handling.
* `src/calibration.cpp`: Implementation of chessboard detection and calibration math.
* `src/ar.cpp`: Implementation of 3D pose estimation and virtual object rendering.
* `include/`: Header files for all modules.
* `intrinsic_params.yml`: Storage for the camera's intrinsic parameters (generated after calibration).



## Acknowledgements
* OpenCV Documentation for `calibrateCamera` and `solvePnP`.
* Checkerboard target provided by the course materials.


