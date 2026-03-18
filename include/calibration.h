/*
 * Name: Akash Shridhar Shetty, Skandhan Madhusudhana
 * Date: March 2026
 * File: include/calibration.h
 *
 * Purpose:
 * Declarations for camera calibration and target detection functions.
 * Handles finding checkerboard corners, sub-pixel accuracy refinement,
 * and executing OpenCV's camera calibration to find intrinsic parameters.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

/**
 * detectAndDrawTarget - Finds checkerboard corners and draws them on the frame
 *
 * @param frame        Input/Output image frame from the video stream
 * @param patternSize  Number of inner corners per a chessboard row and column (e.g., 9x6)
 * @param corners      Output vector of detected corner coordinates (Point2f)
 * @return             True if the complete checkerboard pattern was found, false otherwise
 */
bool detectAndDrawTarget(cv::Mat &frame, cv::Size patternSize, std::vector<cv::Point2f> &corners);

/**
 * executeCalibration - Calibrates the camera using collected 2D and 3D points
 *
 * @param point_list    Vector of 3D world points for each saved frame
 * @param corner_list   Vector of 2D image points for each saved frame
 * @param imageSize     Size of the image frames used for calibration
 * @param cameraMatrix  Output 3x3 intrinsic camera matrix
 * @param distCoeffs    Output vector of distortion coefficients
 * @param rvecs         Output rotation vectors for each frame
 * @param tvecs         Output translation vectors for each frame
 * @return              The final re-projection error (double)
 *
 * Steps:
 * 1. Initialize the camera matrix with a guess (focal lengths = 1, center = image center).
 * 2. Run cv::calibrateCamera.
 * 3. Print out the before and after matrices and the final error.
 */
double executeCalibration(const std::vector<std::vector<cv::Vec3f>> &point_list,
                          const std::vector<std::vector<cv::Point2f>> &corner_list,
                          cv::Size imageSize,
                          cv::Mat &cameraMatrix,
                          cv::Mat &distCoeffs,
                          std::vector<cv::Mat> &rvecs,
                          std::vector<cv::Mat> &tvecs);

#endif // CALIBRATION_H