#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "kalmanFilter.h"
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
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
    TickMeter tm;
    cv::Mat mask;
    cv::Mat overlay;
    bool maskCreated;
    int overlayCounter;
    std::string text_x;
    std::string text_y;

public:
    EyeTracker();
    ~EyeTracker();
private:
    void frameCallback(const sensor_msgs::ImageConstPtr &msg);
    void createMask(cv::Mat src);
    void drawOverlay(cv::Mat src, cv::Point offset);

};