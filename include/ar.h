/*
 * Name: Akash Shridhar Shetty, Skandhan Madhusudhana
 * Date: March 2026
 * File: include/ar.h
 */

#ifndef AR_H
#define AR_H

#include <opencv2/opencv.hpp>
#include <vector>

bool estimatePose(const std::vector<cv::Vec3f> &point_set,
                  const std::vector<cv::Point2f> &corner_set,
                  const cv::Mat &cameraMatrix,
                  const cv::Mat &distCoeffs,
                  cv::Mat &rvec,
                  cv::Mat &tvec);

void drawVirtualObject(cv::Mat &frame,
                       const cv::Mat &cameraMatrix,
                       const cv::Mat &distCoeffs,
                       const cv::Mat &rvec,
                       const cv::Mat &tvec);

#endif