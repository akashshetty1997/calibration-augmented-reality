/*
 * Name: Akash Shridhar Shetty, Skandhan Madhusudhana
 * Date: March 2026
 * File: src/calibration.cpp
 *
 * Purpose:
 * Implementation of camera calibration and target detection functions.
 * Handles finding checkerboard corners, sub-pixel accuracy refinement,
 * and executing OpenCV's camera calibration to find intrinsic parameters.
 */

#include "../include/calibration.h"
#include <iostream>

/**
 * detectAndDrawTarget - Finds checkerboard corners and draws them
 *
 * @param frame        Input/Output image frame (BGR)
 * @param patternSize  Number of inner corners (e.g., cv::Size(9, 6))
 * @param corners      Output vector to store detected corner coordinates
 * @return             True if all corners are found, false otherwise
 *
 * Steps:
 * 1. Convert frame to grayscale (required for sub-pixel refinement).
 * 2. Find chessboard corners using cv::findChessboardCorners.
 * 3. If found, refine corner positions using cv::cornerSubPix.
 * 4. Draw the corners on the original color frame.
 */
bool detectAndDrawTarget(cv::Mat &frame, cv::Size patternSize, std::vector<cv::Point2f> &corners) {
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame.clone();
    }

    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    bool found = cv::findChessboardCorners(frame, patternSize, corners, flags);

    if (found) {
        cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
    }

    cv::drawChessboardCorners(frame, patternSize, corners, found);
    return found;
}

/**
 * executeCalibration - Calibrates the camera using collected 2D and 3D points
 *
 * @param point_list    Vector of 3D world points for each saved frame
 * @param corner_list   Vector of 2D image points for each saved frame
 * @param imageSize     Size of the image frames
 * @param cameraMatrix  Output 3x3 intrinsic matrix (CV_64FC1)
 * @param distCoeffs    Output vector of distortion coefficients
 * @param rvecs         Output rotation vectors
 * @param tvecs         Output translation vectors
 * @return              The final re-projection error
 */
double executeCalibration(const std::vector<std::vector<cv::Vec3f>> &point_list,
                          const std::vector<std::vector<cv::Point2f>> &corner_list,
                          cv::Size imageSize,
                          cv::Mat &cameraMatrix,
                          cv::Mat &distCoeffs,
                          std::vector<cv::Mat> &rvecs,
                          std::vector<cv::Mat> &tvecs) {

    // Step 1: Initialize the 3x3 Camera Matrix (must be CV_64FC1 / double)
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 2) = imageSize.width / 2.0;  // u0
    cameraMatrix.at<double>(1, 2) = imageSize.height / 2.0; // v0

    // Initialize distortion coefficients to zeros (starting with no distortion)
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::cout << "\nInitial Camera Matrix:\n" << cameraMatrix << std::endl;

    // Step 2: Run Calibration
    // Use CALIB_FIX_ASPECT_RATIO as recommended by the assignment
    int flags = cv::CALIB_FIX_ASPECT_RATIO;

    double error = cv::calibrateCamera(point_list, corner_list, imageSize,
                                       cameraMatrix, distCoeffs, rvecs, tvecs, flags);

    // Step 3: Print Results
    std::cout << "\n--- Calibration Results ---" << std::endl;
    std::cout << "Final Re-projection Error: " << error << std::endl;
    std::cout << "\nCalibrated Camera Matrix:\n" << cameraMatrix << std::endl;
    std::cout << "\nDistortion Coefficients:\n" << distCoeffs << std::endl;
    std::cout << "---------------------------\n" << std::endl;

    return error;
}