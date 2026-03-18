/*
 * Name: Akash Shridhar Shetty, Skandhan Madhusudhana
 * Date: March 2026
 * File: src/main.cpp
 *
 * Purpose:
 * Integrated Calibration and AR system. 
 * Allows for data collection ('s'), calibration ('c'), saving ('w'),
 * and real-time AR object projection once calibrated.
 */

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "../include/calibration.h"
#include "../include/ar.h"

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera." << std::endl;
        return -1;
    }

    // === Step 1: Setup Variables ===
    cv::Size patternSize(9, 6);
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>> point_list;
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    bool calibrated = false;

    // Build the constant 3D world points for the 9x6 checkerboard
    std::vector<cv::Vec3f> fixed_point_set;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            fixed_point_set.push_back(cv::Vec3f(j, -i, 0.0f));
        }
    }

    // === Step 2: Try to load existing calibration ===
    cv::FileStorage fs_read("intrinsic_params.yml", cv::FileStorage::READ);
    if (fs_read.isOpened()) {
        fs_read["camera_matrix"] >> cameraMatrix;
        fs_read["distortion_coefficients"] >> distCoeffs;
        fs_read.release();
        calibrated = true;
        std::cout << "Loaded existing calibration parameters." << std::endl;
    }

    std::cout << "=== AR System Ready ===" << std::endl;
    std::cout << "Commands: [s] Save Frame, [c] Calibrate, [w] Write File, [q] Quit" << std::endl;

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        std::vector<cv::Point2f> corner_set;
        bool found = detectAndDrawTarget(frame, patternSize, corner_set);

        // === Step 3: AR Projection (If Calibrated) ===
        if (calibrated && found) {
            cv::Mat rvec, tvec;
            if (estimatePose(fixed_point_set, corner_set, cameraMatrix, distCoeffs, rvec, tvec)) {
                drawVirtualObject(frame, cameraMatrix, distCoeffs, rvec, tvec);
                
                // Task: Print translation for report verification
                // std::cout << "\rTVec: " << tvec.t() << std::flush;
            }
        }

        // Display Info Overlay
        std::string mode = calibrated ? "Mode: AR" : "Mode: Calibration";
        std::string count = "Saved: " + std::to_string(corner_list.size());
        cv::putText(frame, mode, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        cv::putText(frame, count, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

        cv::imshow("AR Project", frame);

        char key = (char)cv::waitKey(10);
        if (key == 'q' || key == 27) break;

        // --- Handle Input Logic ---
        if (key == 's' && found) {
            corner_list.push_back(corner_set);
            point_list.push_back(fixed_point_set);
            std::cout << "Frame " << corner_list.size() << " collected." << std::endl;
        } 
        else if (key == 'c') {
            if (corner_list.size() >= 5) {
                executeCalibration(point_list, corner_list, frame.size(), cameraMatrix, distCoeffs, rvecs, tvecs);
                calibrated = true;
            } else {
                std::cout << "Need at least 5 frames. Current: " << corner_list.size() << std::endl;
            }
        } 
        else if (key == 'w' && calibrated) {
            cv::FileStorage fs_write("intrinsic_params.yml", cv::FileStorage::WRITE);
            fs_write << "camera_matrix" << cameraMatrix;
            fs_write << "distortion_coefficients" << distCoeffs;
            fs_write.release();
            std::cout << "Saved to intrinsic_params.yml" << std::endl;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}