/*
 * Name: Akash Shridhar Shetty, Skandhan Madhusudhana
 * Date: March 2026
 * File: src/ar.cpp
 *
 * Purpose:
 * Implementation of AR functions including 3D axes and a complex virtual object.
 */

#include "../include/ar.h"

bool estimatePose(const std::vector<cv::Vec3f> &point_set,
                  const std::vector<cv::Point2f> &corner_set,
                  const cv::Mat &cameraMatrix,
                  const cv::Mat &distCoeffs,
                  cv::Mat &rvec,
                  cv::Mat &tvec) {
    if (corner_set.size() != point_set.size() || corner_set.empty()) return false;
    return cv::solvePnP(point_set, corner_set, cameraMatrix, distCoeffs, rvec, tvec);
}

void drawVirtualObject(cv::Mat &frame,
                       const cv::Mat &cameraMatrix,
                       const cv::Mat &distCoeffs,
                       const cv::Mat &rvec,
                       const cv::Mat &tvec) {

    // --- 1. Draw 3D Axes (Task: Project Outside Corners or 3D Axes) ---
    // X = Red, Y = Green, Z = Blue
    std::vector<cv::Vec3f> axisPoints;
    axisPoints.push_back(cv::Vec3f(0, 0, 0)); // Origin
    axisPoints.push_back(cv::Vec3f(3, 0, 0)); // X-axis end
    axisPoints.push_back(cv::Vec3f(0, -3, 0)); // Y-axis end
    axisPoints.push_back(cv::Vec3f(0, 0, 3)); // Z-axis end (pointing up)

    std::vector<cv::Point2f> axisImgPoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, axisImgPoints);

    cv::line(frame, axisImgPoints[0], axisImgPoints[1], cv::Scalar(0, 0, 255), 4); // X: Red
    cv::line(frame, axisImgPoints[0], axisImgPoints[2], cv::Scalar(0, 255, 0), 4); // Y: Green
    cv::line(frame, axisImgPoints[0], axisImgPoints[3], cv::Scalar(255, 0, 0), 4); // Z: Blue

    // --- 2. Draw 3D Cube (Task: Create a Virtual Object) ---
    // Let's place a 2x2x2 cube at the center of the grid (corners 3,3 to 5,5)
    std::vector<cv::Vec3f> cubePoints;
    // Bottom square (Z=0)
    cubePoints.push_back(cv::Vec3f(3, -3, 0));
    cubePoints.push_back(cv::Vec3f(5, -3, 0));
    cubePoints.push_back(cv::Vec3f(5, -5, 0));
    cubePoints.push_back(cv::Vec3f(3, -5, 0));
    // Top square (Z=2)
    cubePoints.push_back(cv::Vec3f(3, -3, 2));
    cubePoints.push_back(cv::Vec3f(5, -3, 2));
    cubePoints.push_back(cv::Vec3f(5, -5, 2));
    cubePoints.push_back(cv::Vec3f(3, -5, 2));

    std::vector<cv::Point2f> cubeImgPoints;
    cv::projectPoints(cubePoints, rvec, tvec, cameraMatrix, distCoeffs, cubeImgPoints);

    cv::Scalar cubeColor(0, 255, 255); // Yellow cube
    // Draw bottom, top, and vertical pillars
    for (int i = 0; i < 4; i++) {
        cv::line(frame, cubeImgPoints[i], cubeImgPoints[(i + 1) % 4], cubeColor, 2);     // Bottom
        cv::line(frame, cubeImgPoints[i + 4], cubeImgPoints[((i + 1) % 4) + 4], cubeColor, 2); // Top
        cv::line(frame, cubeImgPoints[i], cubeImgPoints[i + 4], cubeColor, 2);           // Pillars
    }
}