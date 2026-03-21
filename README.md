# Project 4: Camera Calibration and Augmented Reality

**Name:** Akash Shridhar Shetty, Skandhan Madhusudhana  
**Date:** March 2026  
**Course:** CS 5330 - Pattern Recognition & Computer Vision  

## Submission Info
**Videos:** No videos submitted.  
**Time Travel Days:** Not using any time travel days.



## Overview
This project implements a complete pipeline for camera calibration and real-time Augmented Reality (AR). The system calibrates a camera using a checkerboard target to determine intrinsic parameters and then uses those parameters to estimate the camera's 3D pose, allowing for the projection of virtual 3D objects onto the scene.

## Key Features
* **Target Detection:** Robust detection of a 9x6 checkerboard pattern with sub-pixel refinement.
* **Calibration Engine:** Calculates camera matrix and distortion coefficients using at least 5 calibration frames.
* **Pose Estimation:** Real-time 3D rotation and translation calculation using `cv::solvePnP`.
* **AR Rendering:**
    * 3D Coordinate Axes (X: Red, Y: Green, Z: Blue).
    * Complex Virtual Object (3D Wireframe House with roof, door, and windows).
* **Persistent Storage:** Saves and loads calibration parameters from `intrinsic_params.yml`.
* **Feature Detection:** Separate program for Harris corner and ORB feature detection with live threshold controls.
* **Extensions:** Multiple target detection (house + tower on two boards simultaneously).



## Setup & Installation

### Prerequisites
* OpenCV 4.x
* C++ Compiler (g++)
* `pkg-config`

### Compilation
Use the provided `Makefile` to build both programs:
```bash
make
```
This produces two executables: `ar_project` and `feature_detection`.



## Running the AR System

Ensure your webcam is connected and run:
```bash
./ar_project
```

If `intrinsic_params.yml` exists, calibration parameters are loaded automatically and the system starts in AR mode. Otherwise it starts in Calibration mode.

### AR System Controls

| Key | Action |
|:---:|:---|
| **'s'** | **Save Frame:** Stores current checkerboard corners for calibration. |
| **'c'** | **Calibrate:** Runs the calibration (requires 5+ saved frames). |
| **'w'** | **Write File:** Saves calibrated parameters to `intrinsic_params.yml`. |
| **'p'** | **Photo:** Saves a screenshot to the current directory. |
| **'q'** | **Quit:** Exits the application. |

### Calibration Workflow
1. Delete or rename `intrinsic_params.yml` to start fresh.
2. Run `./ar_project` — the system starts in Calibration mode.
3. Point the webcam at a 9x6 checkerboard. Colored dots will appear on the corners.
4. Press `s` from at least 5 different angles to save calibration frames.
5. Press `c` to run calibration. The reprojection error is printed to the terminal.
6. Press `w` to save the calibration to `intrinsic_params.yml`.
7. The system automatically switches to AR mode.

### AR Mode
Once calibrated, point the webcam at the checkerboard to see:
* 3D coordinate axes anchored to the board origin.
* A wireframe house (cyan walls, orange roof, green windows) floating above the board.
* Live rotation `R:[x, y, z]` and translation `T:[x, y, z]` values at the bottom of the window.
* `Targets: N` counter showing how many boards are currently detected.

### Multiple Target Extension
Place two 9x6 checkerboards in the camera's field of view simultaneously. The system will place a **house** on the first detected board and a **tower** on the second.



## Running Feature Detection

```bash
./feature_detection
```

A separate window titled **"Feature Detection"** opens showing the live camera feed with detected features overlaid.

### Feature Detection Controls

| Key | Action |
|:---:|:---|
| **'h'** | Switch to **Harris Corner** mode (red circles). |
| **'o'** | Switch to **ORB Feature** mode (green circles with orientation). |
| **'+'** | Show **more** features (lower Harris threshold / more ORB keypoints). |
| **'-'** | Show **fewer** features (higher Harris threshold / fewer ORB keypoints). |
| **'s'** | Save a screenshot to the current directory. |
| **'q'** | Quit. |

### Tips for Feature Detection
* Start in Harris mode and press `+` several times until red circles appear on the checkerboard inner corners.
* Switch to ORB mode with `o` — green circles with orientation lines will appear.
* Point at non-checkerboard objects (keyboard, face, textured surfaces) to see features on natural scenes.
* Screenshots are saved as `feature_screenshot_0.png`, `feature_screenshot_1.png`, etc.



## Project Structure
```
.
├── src/
│   ├── main.cpp              # Main AR loop, calibration workflow, multi-target detection
│   ├── calibration.cpp       # Chessboard detection and camera calibration
│   ├── ar.cpp                # Pose estimation and virtual object rendering
│   └── feature_detection.cpp # Harris and ORB feature detection (Task 7)
├── include/
│   ├── ar.h                  # AR function declarations
│   └── calibration.h         # Calibration function declarations
├── Makefile                  # Builds ar_project and feature_detection
└── intrinsic_params.yml      # Saved camera calibration parameters
```



## Acknowledgements
* OpenCV Documentation for `calibrateCamera`, `solvePnP`, `cornerHarris`, and `ORB`.
* Checkerboard target provided by the course materials.
* Claude AI for debugging assistance.
