#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "kalmanFilter.h"
#include <cv_bridge/cv_bridge.h>
#include <queue>

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "std_srvs/Empty.h"
//#include <sensor_msgs/image_encodings.h>


class TickMeter
{
public:
    TickMeter();
    void start();
    void stop();

    int64 getTimeTicks() const;
    double getTimeMicro() const;
    double getTimeMilli() const;
    double getTimeSec()   const;
    int64 getCounter() const;

    void reset();
private:
    int64 counter;
    int64 sumTime;
    int64 startTime;
};

class EyeTracker {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
	KalmanFilter filter;
    std::vector<cv::Point> point_history;
    std::queue<cv::Point> calibration_points;
    std::queue<int> calibration_radii;
    cv::Point lens_center;
    TickMeter tm;
    cv::Mat mask;
    cv::Mat mask_fullsize;
    cv::Mat overlay;
    bool maskCreated;
    bool tracking_active;
    bool overlay_active;
    bool calibrate_center_on_next_frame;
    int overlayCounter;
    float lens_radius;
    bool clockwiseRotation;
    std::string text_x;
    std::string text_y;
    ros::ServiceServer tracking_toggle_service;
    ros::ServiceServer overlay_toggle_service;
    ros::ServiceServer calibration_toggle_service;

public:
    EyeTracker();
    ~EyeTracker();
private:
    void frameCallback(const sensor_msgs::ImageConstPtr &msg);
    void createMask(cv::Mat src, cv::Mat src_full, cv::Point center, int radius, cv::Rect eyeRect);
    void drawOverlay(cv::Mat src, cv::Point offset);
    void drawText(cv::Mat frame, cv::Point offset);
    bool toggleOverlay(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool toggleTracking(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool calibrateCenterTrigger(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void calibrateCenter(cv::Mat frame);
    void initLensCenter(cv::Mat frame);

};
