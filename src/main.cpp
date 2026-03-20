/*
 * Name: Akash Shridhar Shetty, Skandhan Madhusudhana
 * Date: March 2026
 * File: src/main.cpp
 *
 * Purpose:
 * Integrated Calibration and AR system.
 * Extension: Multiple target detection — places a house on the first
 * checkerboard and a tower on the second checkerboard simultaneously.
 */

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "../include/calibration.h"
#include "../include/ar.h"

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera." << std::endl;
        return -1;
    }

    // === Setup ===
    cv::Size patternSize(9, 6);
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>> point_list;
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    bool calibrated = false;

    // 3D world points for a 9x6 board (same for every target)
    std::vector<cv::Vec3f> fixed_point_set;
    for (int i = 0; i < patternSize.height; i++)
        for (int j = 0; j < patternSize.width; j++)
            fixed_point_set.push_back(cv::Vec3f(j, -i, 0.0f));

    // Load calibration if it exists
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
    std::cout << "Extension: Place TWO checkerboards in view for multiple-target AR!" << std::endl;

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // ── Extension: Multi-target detection ────────────────────
        // Strategy: find the first target in the full frame, then mask
        // it out and search again for a second target.

        int targetsFound = 0;

        // --- Target 1: search the whole frame ---
        cv::Mat frame1 = frame.clone(); // work on a copy for detection
        std::vector<cv::Point2f> corners1;
        bool found1 = detectAndDrawTarget(frame, patternSize, corners1);

        if (found1) {
            targetsFound++;
            if (calibrated) {
                cv::Mat rvec1, tvec1;
                if (estimatePose(fixed_point_set, corners1, cameraMatrix,
                                 distCoeffs, rvec1, tvec1)) {
                    // Object 0 = house
                    drawVirtualObject(frame, cameraMatrix, distCoeffs,
                                      rvec1, tvec1, 0);

                    // Display rotation and translation on screen (Task 4 + report)
                    std::ostringstream oss_r, oss_t;
                    oss_r << std::fixed << std::setprecision(2)
                          << "R:[" << rvec1.at<double>(0) << ", "
                          << rvec1.at<double>(1) << ", "
                          << rvec1.at<double>(2) << "]";
                    oss_t << std::fixed << std::setprecision(2)
                          << "T:[" << tvec1.at<double>(0) << ", "
                          << tvec1.at<double>(1) << ", "
                          << tvec1.at<double>(2) << "]";
                    cv::putText(frame, oss_r.str(),
                                cv::Point(20, frame.rows - 60),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(255, 255, 0), 2);
                    cv::putText(frame, oss_t.str(),
                                cv::Point(20, frame.rows - 30),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(255, 255, 0), 2);
                }
            }

            // --- Target 2: mask out the first board, search again ---
            // Build a convex hull around the first board's corners and
            // black it out so findChessboardCorners ignores it.
            cv::Mat masked = frame1.clone();
            std::vector<cv::Point> hull_pts;
            for (auto &p : corners1)
                hull_pts.push_back(cv::Point((int)p.x, (int)p.y));

            std::vector<std::vector<cv::Point>> hull_contour(1);
            cv::convexHull(hull_pts, hull_contour[0]);
            // Expand the mask slightly so edge corners are covered
            cv::drawContours(masked, hull_contour, 0,
                             cv::Scalar(128,128,128), cv::FILLED);

            std::vector<cv::Point2f> corners2;
            bool found2 = detectAndDrawTarget(frame, patternSize, corners2);

            // Make sure corners2 is not the same board as corners1
            // (check centroid distance — same board would be very close)
            if (found2 && !corners2.empty() && !corners1.empty()) {
                cv::Point2f c1(0,0), c2(0,0);
                for (auto &p : corners1) c1 += p;
                for (auto &p : corners2) c2 += p;
                c1 *= 1.f / corners1.size();
                c2 *= 1.f / corners2.size();
                float dist = cv::norm(c1 - c2);

                if (dist < 50) {
                    // Too close — same board detected twice, ignore
                    found2 = false;
                }
            }

            if (found2) {
                targetsFound++;
                if (calibrated) {
                    cv::Mat rvec2, tvec2;
                    if (estimatePose(fixed_point_set, corners2, cameraMatrix,
                                     distCoeffs, rvec2, tvec2)) {
                        // Object 1 = tower
                        drawVirtualObject(frame, cameraMatrix, distCoeffs,
                                          rvec2, tvec2, 1);
                        std::cout << "  Target 2 found" << std::flush;
                    }
                }
            }
        }

        // ── Overlay ───────────────────────────────────────────────
        std::string mode  = calibrated ? "Mode: AR" : "Mode: Calibration";
        std::string count = "Saved: " + std::to_string(corner_list.size());
        std::string tstr  = "Targets: " + std::to_string(targetsFound);
        cv::putText(frame, mode,  cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);
        cv::putText(frame, count, cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);
        cv::putText(frame, tstr,  cv::Point(20, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);

        cv::imshow("AR Project", frame);

        char key = (char)cv::waitKey(10);
        if (key == 'q' || key == 27) break;

        if (key == 's' && found1) {
            corner_list.push_back(corners1);
            point_list.push_back(fixed_point_set);
            std::cout << "Frame " << corner_list.size() << " collected." << std::endl;
        } else if (key == 'c') {
            if (corner_list.size() >= 5) {
                executeCalibration(point_list, corner_list, frame.size(),
                                   cameraMatrix, distCoeffs, rvecs, tvecs);
                calibrated = true;
            } else {
                std::cout << "Need at least 5 frames. Current: "
                          << corner_list.size() << std::endl;
            }
        } else if (key == 'w' && calibrated) {
            cv::FileStorage fs_write("intrinsic_params.yml",
                                     cv::FileStorage::WRITE);
            fs_write << "camera_matrix" << cameraMatrix;
            fs_write << "distortion_coefficients" << distCoeffs;
            fs_write.release();
            std::cout << "Saved to intrinsic_params.yml" << std::endl;
        } else if (key == 'p') {
            static int shotCount = 0;
            std::string fname = "screenshot_" + std::to_string(shotCount++) + ".png";
            cv::imwrite(fname, frame);
            std::cout << "Saved: " << fname << std::endl;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}