/*
 * Name: Akash Shridhar Shetty, Skandhan Madhusudhana
 * Date: March 2026
 * File: src/ar.cpp
 *
 * Purpose:
 * Implementation of AR functions.
 * Extension: Complex virtual object (house with roof, door, windows)
 * and multiple target support (house + tower on two boards).
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

static std::vector<cv::Point2f> project(const std::vector<cv::Vec3f> &pts3d,
                                         const cv::Mat &rvec, const cv::Mat &tvec,
                                         const cv::Mat &K, const cv::Mat &D) {
    std::vector<cv::Point2f> pts2d;
    cv::projectPoints(pts3d, rvec, tvec, K, D, pts2d);
    return pts2d;
}

static void ln(cv::Mat &f, const std::vector<cv::Point2f> &p,
               int a, int b, cv::Scalar color, int thickness = 2) {
    cv::line(f, p[a], p[b], color, thickness);
}

void drawVirtualObject(cv::Mat &frame,
                       const cv::Mat &cameraMatrix,
                       const cv::Mat &distCoeffs,
                       const cv::Mat &rvec,
                       const cv::Mat &tvec) {

    // ── 1. 3D Axes — same as original code ───────────────────────
    // Z=3 points UP toward viewer (same convention as original)
    std::vector<cv::Vec3f> axisPoints = {
        {0,0,0}, {3,0,0}, {0,-3,0}, {0,0,3}
    };
    std::vector<cv::Point2f> axisImgPoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, axisImgPoints);
    cv::line(frame, axisImgPoints[0], axisImgPoints[1], cv::Scalar(0,0,255),   4); // X red
    cv::line(frame, axisImgPoints[0], axisImgPoints[2], cv::Scalar(0,255,0),   4); // Y green
    cv::line(frame, axisImgPoints[0], axisImgPoints[3], cv::Scalar(255,128,0), 4); // Z blue

    // ── 2. House (Extension: complex object) ─────────────────────
    // Based on original cube which used Z=0 (base) and Z=2 (top).
    // House footprint: X 3-5, Y -2 to -4 (same area as original cube)
    // Walls go from Z=0 up to Z=2, roof peak at Z=3
    float x0=3.0f, x1=5.0f;
    float y0=-2.0f, y1=-4.0f;
    float mx=(x0+x1)*0.5f;  // 4.0
    float my=(y0+y1)*0.5f;  // -3.0

    std::vector<cv::Vec3f> h3d = {
        // base ring Z=0   (0-3)
        {x0,y0,0},{x1,y0,0},{x1,y1,0},{x0,y1,0},
        // top ring  Z=2   (4-7)
        {x0,y0,2},{x1,y0,2},{x1,y1,2},{x0,y1,2},
        // roof peak Z=3.5 (8)
        {mx, my, 3.5f},
        // door base (9,10) door top (11,12)
        {3.7f,y0,0},{4.3f,y0,0},
        {3.7f,y0,1.2f},{4.3f,y0,1.2f},
        // left window (13-16)
        {3.2f,y0,0.6f},{3.7f,y0,0.6f},
        {3.2f,y0,1.4f},{3.7f,y0,1.4f},
        // right window (17-20)
        {4.3f,y0,0.6f},{4.8f,y0,0.6f},
        {4.3f,y0,1.4f},{4.8f,y0,1.4f},
    };

    auto h = project(h3d, rvec, tvec, cameraMatrix, distCoeffs);

    cv::Scalar wall(  0, 220, 255); // cyan walls
    cv::Scalar roof(  0, 140, 255); // orange roof
    cv::Scalar door(255, 180,  50); // door
    cv::Scalar win (180, 255, 100); // windows

    // Base square
    ln(frame,h,0,1,wall); ln(frame,h,1,2,wall);
    ln(frame,h,2,3,wall); ln(frame,h,3,0,wall);
    // Top square
    ln(frame,h,4,5,wall); ln(frame,h,5,6,wall);
    ln(frame,h,6,7,wall); ln(frame,h,7,4,wall);
    // Vertical pillars
    ln(frame,h,0,4,wall); ln(frame,h,1,5,wall);
    ln(frame,h,2,6,wall); ln(frame,h,3,7,wall);
    // Roof edges to peak
    ln(frame,h,4,8,roof); ln(frame,h,5,8,roof);
    ln(frame,h,6,8,roof); ln(frame,h,7,8,roof);
    // Door
    ln(frame,h, 9,10,door); ln(frame,h,11,12,door);
    ln(frame,h, 9,11,door); ln(frame,h,10,12,door);
    // Left window + cross
    ln(frame,h,13,14,win); ln(frame,h,15,16,win);
    ln(frame,h,13,15,win); ln(frame,h,14,16,win);
    ln(frame,h,13,16,win); ln(frame,h,14,15,win);
    // Right window + cross
    ln(frame,h,17,18,win); ln(frame,h,19,20,win);
    ln(frame,h,17,19,win); ln(frame,h,18,20,win);
    ln(frame,h,17,20,win); ln(frame,h,18,19,win);
}

// Overload for multiple targets: 0=house, 1=tower
void drawVirtualObject(cv::Mat &frame,
                       const cv::Mat &cameraMatrix,
                       const cv::Mat &distCoeffs,
                       const cv::Mat &rvec,
                       const cv::Mat &tvec,
                       int objectID) {
    if (objectID == 0) {
        drawVirtualObject(frame, cameraMatrix, distCoeffs, rvec, tvec);
        return;
    }

    // Tower: 1x1 footprint, height 5 units, spire to Z=6
    std::vector<cv::Vec3f> t3d = {
        {3.8f,-2.3f,0},{4.2f,-2.3f,0},{4.2f,-2.7f,0},{3.8f,-2.7f,0},       // base 0-3
        {3.8f,-2.3f,2},{4.2f,-2.3f,2},{4.2f,-2.7f,2},{3.8f,-2.7f,2},        // mid  4-7
        {3.9f,-2.4f,3.5f},{4.1f,-2.4f,3.5f},{4.1f,-2.6f,3.5f},{3.9f,-2.6f,3.5f}, // top 8-11
        {4.0f,-2.5f,5.0f},                                                   // spire 12
        {3.8f,-2.3f,1.0f},{4.2f,-2.3f,1.0f},{4.2f,-2.7f,1.0f},{3.8f,-2.7f,1.0f}, // band 13-16
    };

    auto t = project(t3d, rvec, tvec, cameraMatrix, distCoeffs);
    cv::Scalar base(255, 50,200);
    cv::Scalar spire(50,200,255);
    cv::Scalar band(180,255,180);

    ln(frame,t,0,1,base); ln(frame,t,1,2,base); ln(frame,t,2,3,base); ln(frame,t,3,0,base);
    ln(frame,t,4,5,base); ln(frame,t,5,6,base); ln(frame,t,6,7,base); ln(frame,t,7,4,base);
    ln(frame,t,0,4,base); ln(frame,t,1,5,base); ln(frame,t,2,6,base); ln(frame,t,3,7,base);
    ln(frame,t,8,9,spire); ln(frame,t,9,10,spire); ln(frame,t,10,11,spire); ln(frame,t,11,8,spire);
    ln(frame,t,4,8,spire); ln(frame,t,5,9,spire); ln(frame,t,6,10,spire); ln(frame,t,7,11,spire);
    ln(frame,t,8,12,spire); ln(frame,t,9,12,spire); ln(frame,t,10,12,spire); ln(frame,t,11,12,spire);
    ln(frame,t,13,14,band); ln(frame,t,14,15,band); ln(frame,t,15,16,band); ln(frame,t,16,13,band);
}