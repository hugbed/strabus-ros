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

using namespace cv;

EyeTracker::EyeTracker()
        : it_(nh_) {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1,
                               &EyeTracker::frameCallback, this);
    image_pub_ = it_.advertise("/tracking/eye_tracking_feed", 1);
    maskCreated = false;
    overlayCounter = 0;
}

EyeTracker::~EyeTracker() {
}

cv::Point getOffset(cv::Point point, cv::Mat frame ){
    cv::Point offset;
    offset.x = (int)abs(frame.cols/2 -point.x);
    offset.y = (int)abs(frame.rows/2 -point.y);
    return offset;
}

void EyeTracker::drawOverlay(cv::Mat frame, cv::Point offset){

    cv::Scalar color_green(100, 200, 0, 1);
    cv::line(frame, cv::Point(frame.cols/2,0), cv::Point(frame.cols/2,frame.rows), color_green, 1);
    cv::line(frame, cv::Point(0,frame.rows/2), cv::Point(frame.cols,frame.rows/2), color_green, 1);
    overlayCounter ++;

    if (overlayCounter %20 ==0) {
        std::stringstream ss;
        ss << "X offset : " << offset.x;
        text_x = ss.str();
        ss.str("");
        ss << "Y offset : " << offset.y;
        text_y = ss.str();
    }

    int fontFace = FONT_HERSHEY_DUPLEX;
    double fontScale = 1;
    int thickness = 1;
    int baseline = 0;

    Size textSize_x = getTextSize(text_x, fontFace,
                                  fontScale, thickness, &baseline);

    Size textSize_y = getTextSize(text_y, fontFace,
                                  fontScale, thickness, &baseline);
    // place text in top right
    Point textOrg_y((frame.cols - textSize_y.width - 10), 10 + textSize_y.height);

    // place text in top left
    Point textOrg_x(10, 10 + textSize_x.height);

    // Draw text
    putText(frame, text_x, textOrg_x, fontFace, fontScale,
            Scalar::all(255), thickness, 8);
    // Draw text
    putText(frame, text_y, textOrg_y, fontFace, fontScale,
            Scalar::all(255), thickness, 8);

}

void EyeTracker::createMask(cv::Mat eyeROI){
    //Create mask
    mask = cv::Mat(eyeROI.size(), eyeROI.type());
    mask = cv::Scalar(0);
    cv::circle(mask, cv::Point(eyeROI.cols/2, eyeROI.rows/2), (int) eyeROI.cols/5, 1234, -1);
}

void EyeTracker::frameCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    tm.stop();
    std::cout << "Framerate " <<  tm.getCounter()/tm.getTimeSec() <<std::endl;
    tm.reset();
    tm.start();

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


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

        cv::Mat eye;
        scaleToFastSize(frame_gray, eye);

        if(!maskCreated){
            createMask(eye);
            maskCreated = true;
        }

        //Get the eye center using gradient algorithm
        float confidence = 0.8;
        cv::Point position = findEyeCenter(eye, mask, &confidence);
        cv::Rect eyeRect(0, 0, frame.cols, frame.rows);
        position = unscalePoint(position,eyeRect);

        //Kalman filter eye center for light smoothing
        cv::Point filteredPos = filter.filterPosition(position, confidence);

        //Draw position trail
        circle(frame, filteredPos, 4, 1234);
        std::vector<cv::Point>::iterator it;
        it = point_history.begin();
        point_history.insert(it,filteredPos);
        if (point_history.size() > 10){
            point_history.pop_back();
        }
        int movementThreshold = 20;
        cv::Scalar color(0, 0, 255, 0.5); //red
        for(int i = 0; i < point_history.size() - 1; ++i)
        {
            cv::Point diff = point_history[i] - point_history[i+1];
            int difference = std::sqrt(diff.dot(diff));
            //std::cout <<  "Difference " << difference << std::endl;
            int redness = std::min(255, difference);
            if (difference > movementThreshold){
                redness = 128;
            }
            cv::Scalar color(128-redness, 128- redness, redness*2, 0.5);
            cv::line(frame, point_history[i], point_history[i+1], color, std::max( 1, int(point_history.size() - i)/2) );
        }

        // Draw GUI on frame
        cv::Point offset = getOffset(filteredPos, frame);
        drawOverlay(frame, offset);

        //Publish frame
        cv_ptr->image = frame;
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        image_pub_.publish(cv_ptr->toImageMsg());

    } else {
        printf(" --(!) No captured frame -- Break!");
    }


    cv::waitKey(3);
}



TickMeter::TickMeter()
{
    reset();
}
int64 TickMeter::getTimeTicks() const
{
    return sumTime;
}
double TickMeter::getTimeMicro() const
{
    return  getTimeMilli()*1e3;
}
double TickMeter::getTimeMilli() const
{
    return getTimeSec()*1e3;
}
double TickMeter::getTimeSec() const
{
    return (double)getTimeTicks()/cv::getTickFrequency();
}
int64 TickMeter::getCounter() const
{
    return counter;
}
void TickMeter::reset()
{
    startTime = 0;
    sumTime = 0;
    counter = 0;
}

void TickMeter::start()
{
    startTime = cv::getTickCount();
}
void TickMeter::stop()
{
    int64 time = cv::getTickCount();
    if ( startTime == 0 )
        return;
    ++counter;
    sumTime += ( time - startTime );
    startTime = 0;
}
