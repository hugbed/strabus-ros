#ifndef EYE_CENTER_H
#define EYE_CENTER_H

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
cv::Point findEyeCenter(cv::Mat eye, cv::Mat mask, float * confidence);
cv::Point findCenterWithBlobs(cv::Mat eye, cv::Mat mask, cv::SimpleBlobDetector* detector);
void scaleToFastSize(const cv::Mat &src,cv::Mat &dst);
cv::Point unscalePoint(cv::Point, cv::Rect);
cv::Point scalePoint(cv::Point, cv::Rect);
int scaleLength(float l, cv::Rect);
cv::Point findCenterWithBlobs(cv::Mat eye, cv::Mat mask);
void equalizeForTube(cv::Mat eyeROI, cv::Mat mask);
#endif
