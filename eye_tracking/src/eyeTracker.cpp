#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>


#include <iostream>
#include <queue>
#include <stdio.h>
#include <math.h>


#include "constants.h"
#include "findEyeCenter.h"
#include "eyeTracker.h"
#include <eye_tracking/Position.h>

static const std::string OPENCV_WINDOW = "Debug window";

EyeTracker::EyeTracker()
        : it_(nh_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
                               &EyeTracker::frameCallback, this);
    image_pub_ = it_.advertise("/tracking/eye_tracking_feed", 1);
    position_pub_ = nh_.advertise<eye_tracking::Position>("/tracking/position", 1);
    offset_pub_ = nh_.advertise<eye_tracking::Position>("/tracking/offset", 1);
    filtered_position_pub_ = nh_.advertise<eye_tracking::Position>("/tracking/filtered_position", 1);

}

EyeTracker::~EyeTracker() {

}

eye_tracking::Position getOffset(cv::Point point, cv::Mat frame ){
    eye_tracking::Position offset;
    offset.x_pos = (int)abs(frame.cols/2 -point.x);
    offset.y_pos = (int)abs(frame.rows/2 -point.y);
    return offset;
}

void EyeTracker::frameCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Apply the classifier to the frame
    if (!cv_ptr->image.empty()) {
        cv::Mat frame = cv_ptr->image;
        if (!filter.isInit()){
            filter.initialize(frame);
            std::cout << "Initializing filter"<< std::endl;
        }
        cv::flip(frame, frame, 1);
        std::vector<cv::Mat> rgbChannels(3);
        cv::Mat frame_gray = rgbChannels[2];
        cv_bridge::CvImagePtr return_ptr;
        cv::split(frame, rgbChannels);
        cvtColor(frame, frame_gray, CV_BGR2GRAY);
        equalizeHist(frame_gray, frame_gray);
        float confidence = 0.2;
        cv::Point position = findEyeCenter(frame_gray, "Test", &confidence);
        //circle(frame_gray, position, 5, 1234);

        cv::Point filteredPos = filter.filterPosition(position, confidence);

        circle(frame_gray, filteredPos, 4, 1234);

        eye_tracking::Position p;
        p.x_pos = position.x;
        p.y_pos = position.y;

        eye_tracking::Position fp;
        fp.x_pos = filteredPos.x;
        fp.y_pos = filteredPos.y;

        eye_tracking::Position offset = getOffset(filteredPos, frame);
        // Update GUI Window

        //delete &cv_ptr->image;
        cv_ptr->image = frame_gray;
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);


        image_pub_.publish(cv_ptr->toImageMsg());
        position_pub_.publish(p);
        filtered_position_pub_.publish(fp);
        offset_pub_.publish(offset);

    } else {
        printf(" --(!) No captured frame -- Break!");
    }


    cv::waitKey(3);
}
