/*
 * Name: Akash Shridhar Shetty, Skandhan Madhusudhana
 * Date: March 2026
 * File: src/feature_detection.cpp
 *
 * Purpose:
 * Task 7 - Detect and display robust features (Harris Corners + ORB)
 * on a live video stream. Uses non-maximum suppression for Harris so
 * the screen doesn't flood with detections.
 *
 * Controls:
 *   'h' - Switch to Harris Corner mode
 *   'o' - Switch to ORB feature mode
 *   '+' - More features
 *   '-' - Fewer features
 *   's' - Save screenshot
 *   'q' - Quit
 */

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <string>
#include <vector>

void detectHarris(cv::Mat &frame, double thresh) {
    cv::Mat gray, dst, dst_norm, dilated;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gray, CV_32F);

    cv::cornerHarris(gray, dst, 2, 3, 0.04);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);

    // Dilation trick: pixel == local max means it IS a corner peak
    cv::dilate(dst_norm, dilated, cv::Mat());

    int count = 0;
    int minDist = 8;
    cv::Mat drawn = cv::Mat::zeros(dst_norm.size(), CV_8U);

    for (int i = 5; i < dst_norm.rows - 5; i++) {
        for (int j = 5; j < dst_norm.cols - 5; j++) {
            float val = dst_norm.at<float>(i, j);
            if (val > (float)thresh && val == dilated.at<float>(i, j)) {
                cv::Rect roi(std::max(0, j - minDist),
                             std::max(0, i - minDist),
                             2 * minDist, 2 * minDist);
                roi &= cv::Rect(0, 0, drawn.cols, drawn.rows);
                if (cv::countNonZero(drawn(roi)) == 0) {
                    cv::circle(frame, cv::Point(j, i), 6,
                               cv::Scalar(0, 0, 255), 2);
                    drawn.at<uchar>(i, j) = 255;
                    count++;
                }
            }
        }
    }

    cv::putText(frame, "Mode: Harris Corners", cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(frame, "Threshold: " + std::to_string((int)thresh),
                cv::Point(20, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(frame, "Features: " + std::to_string(count),
                cv::Point(20, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(frame, "[+] more  [-] fewer  [h] Harris  [o] ORB  [s] save",
                cv::Point(20, frame.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
}

void detectORB(cv::Mat &frame, int nFeatures) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::ORB> orb = cv::ORB::create(nFeatures);
    std::vector<cv::KeyPoint> keypoints;
    orb->detect(gray, keypoints);

    cv::drawKeypoints(frame, keypoints, frame,
                      cv::Scalar(0, 255, 0),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::putText(frame, "Mode: ORB Features", cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(frame, "Max Features: " + std::to_string(nFeatures),
                cv::Point(20, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(frame, "Detected: " + std::to_string((int)keypoints.size()),
                cv::Point(20, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(frame, "[+] more  [-] fewer  [h] Harris  [o] ORB  [s] save",
                cv::Point(20, frame.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera." << std::endl;
        return -1;
    }

    bool harrisMode     = true;
    double harrisThresh = 180;
    int orbFeatures     = 300;
    int screenshotCount = 0;

    std::cout << "=== Feature Detection ===" << std::endl;
    std::cout << "[h] Harris  [o] ORB  [+] more  [-] fewer  [s] save  [q] quit" << std::endl;

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        if (harrisMode)
            detectHarris(frame, harrisThresh);
        else
            detectORB(frame, orbFeatures);

        cv::imshow("Feature Detection", frame);

        char key = (char)cv::waitKey(10);
        if (key == 'q' || key == 27) break;

        if (key == 'h') {
            harrisMode = true;
            std::cout << "Harris mode  threshold=" << harrisThresh << std::endl;
        } else if (key == 'o') {
            harrisMode = false;
            std::cout << "ORB mode  maxFeatures=" << orbFeatures << std::endl;
        } else if (key == '+' || key == '=') {
            if (harrisMode) {
                harrisThresh = std::max(10.0, harrisThresh - 10);
                std::cout << "Harris threshold: " << harrisThresh << std::endl;
            } else {
                orbFeatures += 100;
                std::cout << "ORB max features: " << orbFeatures << std::endl;
            }
        } else if (key == '-') {
            if (harrisMode) {
                harrisThresh = std::min(250.0, harrisThresh + 10);
                std::cout << "Harris threshold: " << harrisThresh << std::endl;
            } else {
                orbFeatures = std::max(50, orbFeatures - 100);
                std::cout << "ORB max features: " << orbFeatures << std::endl;
            }
        } else if (key == 's') {
            std::string fname = "feature_screenshot_" +
                                std::to_string(screenshotCount++) + ".png";
            cv::imwrite(fname, frame);
            std::cout << "Saved: " << fname << std::endl;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}