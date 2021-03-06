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
static const int CALIBRATION_COUNT = 1;

using namespace cv;
using namespace std;

EyeTracker::EyeTracker()
        : it_(nh_) {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("camera/image", 1, &EyeTracker::frameCallback, this);
    //image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &EyeTracker::frameCallback, this);
    image_pub_ = it_.advertise("tracking/image", 1);
    maskCreated = false;
    overlayCounter = 0;
    tracking_active = false;
    overlay_active = false;
    calibrate_center_on_next_frame = true;
    tracking_toggle_service = nh_.advertiseService("tracking/toggle_tracking", &EyeTracker::toggleTracking,this);
    overlay_toggle_service = nh_.advertiseService("tracking/toggle_overlay", &EyeTracker::toggleOverlay,this);
    calibration_toggle_service = nh_.advertiseService("tracking/calibrate", &EyeTracker::calibrateCenterTrigger,this);
    std::string ns = ros::this_node::getNamespace();
    clockwiseRotation = true;
    if(ns.find("left") == std::string::npos){
        clockwiseRotation = false;
    }
}

EyeTracker::~EyeTracker() {
}

cv::Point getOffset(cv::Point point, cv::Mat frame ){
    cv::Point offset;
    offset.x = (int)abs(frame.cols/2 -point.x);
    offset.y = (int)abs(frame.rows/2 -point.y);
    return offset;
}

bool EyeTracker::toggleTracking(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    tracking_active = !tracking_active;
    return true;
}

bool EyeTracker::toggleOverlay(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    overlay_active = !overlay_active;
    return true;
}

bool EyeTracker::calibrateCenterTrigger(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    cout << "calibration request received" <<endl;
    calibrate_center_on_next_frame = true;
    return true;
}

void rotateImage90(cv::Mat &src, bool clockwise){
    if (clockwise){
        transpose(src, src);
        flip(src, src,1); //transpose+flip(1)=CW
    } else {
        transpose(src, src);
        flip(src, src, 0); //transpose+flip(0)=CCW
    }
}

void EyeTracker::drawText(cv::Mat frame, cv::Point offset){
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
    double fontScale = 0.4;
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

void EyeTracker::drawOverlay(cv::Mat frame, cv::Point offset){
    if (!overlay_active){
        return;
    }

    cv::Scalar color(100, 200, 0, 1);
    if (calibrate_center_on_next_frame){
        color = cv::Scalar(0, 0, 200);
    }
    cv::line(frame, cv::Point(lens_center.x,0), cv::Point(lens_center.x,frame.rows), color, 1);
    cv::line(frame, cv::Point(0,lens_center.y), cv::Point(frame.cols,lens_center.y), color, 1);
    circle( frame, lens_center, lens_radius, color, 1, 8, 0 );
}

void EyeTracker::createMask(cv::Mat eyeROI, cv::Mat eyeFull, cv::Point lens_center, int lens_radius, cv::Rect eyeRect){
    //Create mask
    int lens_radius_scaled = scaleLength(lens_radius, eyeRect);
    cv::Point lens_center_scaled = scalePoint(lens_center, eyeRect);

    mask = cv::Mat(eyeROI.size(), eyeROI.type());
    mask = cv::Scalar(0);

    mask_fullsize = cv::Mat(eyeFull.size(), eyeFull.type());
    mask_fullsize = cv::Scalar(0);

    cv::circle(mask, lens_center_scaled, lens_radius_scaled, 1234, -1);
    cv::circle(mask_fullsize, lens_center, lens_radius, 1234, -1);
    maskCreated = true;
}

void EyeTracker::calibrateCenter(cv::Mat frame_gray){
    GaussianBlur( frame_gray, frame_gray, Size(9, 9), 2, 2 );

    vector<Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles( frame_gray, circles, CV_HOUGH_GRADIENT, 1, frame_gray.rows/8, 75, 50, 0, 0 );

    double pixel_min, pixel_max;
    minMaxLoc(frame_gray,&pixel_min, &pixel_max);

    //threshold( frame_gray, frame_gray,(int) pixel_min +50, 200,0 );
    /// Draw the circles detected

    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( frame_gray, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( frame_gray, center, radius, Scalar(0,0,255), 3, 8, 0 );
        calibration_points.push(center);
        calibration_radii.push(radius);
    }
    int center_count = calibration_points.size();
    if (center_count > CALIBRATION_COUNT){
        cv::Point p;
        float r;
        lens_center = cv::Point(0,0);
        lens_radius = 0.0f;
        for(int i = 0; i < center_count ; i++) {
            p = calibration_points.front();
            calibration_points.pop();
            r = (float) calibration_radii.front();
            calibration_radii.pop();

            lens_center.x += p.x/center_count;
            lens_center.y += p.y/center_count;
            lens_radius += r/center_count;
          }
        lens_radius = lens_radius/1.15   ;
        //So new mask gets generated
        maskCreated = false;

        //Stop trying to calibrate
        calibrate_center_on_next_frame = false;
    }

   //imshow( "Hough Circle Transform Demo", frame_gray );

}

void EyeTracker::initLensCenter(cv::Mat frame){
    lens_center = cv::Point(frame.cols/2, frame.rows/2);
    lens_radius = frame.cols/4;
}

cv::Mat copyFrameROI(cv::Mat src, int width, int height, cv::Point center){
    if (src.cols <width || src.rows <height ){
        return src;
    }
    int x = std::max(center.x - width/2, 0);
    int y = std::max(center.y - height/2, 0);

    if (center.x + width/2 > src.cols){
        x = src.cols - width/2;
    }
    if (center.y + height/2 > src.rows){
        y = src.rows - height/2;
    }
    return src(cv::Rect(x,y, width, height));
}

void EyeTracker::frameCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;

    // Framerate logging (for performance measurement purposes)
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
        cv::Rect eyeRect(0, 0, frame.cols, frame.rows);
        std::vector <cv::Mat> rgbChannels(3);
        cv::Mat frame_gray = rgbChannels[2];
        cv::split(frame, rgbChannels);
        cvtColor(frame, frame_gray, CV_BGR2GRAY);

        if (!filter.isInit()){
            filter.initialize(frame);
            initLensCenter(frame);
            std::cout << "Initializing filter"<< std::endl;
        }

        if (calibrate_center_on_next_frame){
            calibrateCenter(frame_gray);
        }

        if (!maskCreated) {
            cv::Mat eye;
            scaleToFastSize(frame_gray, eye);
            createMask(eye, frame_gray, lens_center, lens_radius, eyeRect);
        }
        //imshow( "mask", mask_fullsize );

        bool converted = false;
        if (tracking_active) {
            cv::Mat eye;
            scaleToFastSize(frame_gray, eye);

            //Get the eye center using gradient algorithm
            float confidence = 0.8;
            cv::Point position = findEyeCenter(eye, mask, &confidence);
            position = unscalePoint(position, eyeRect);

            //Kalman filter eye center for light smoothing
            cv::Point filteredPos = filter.filterPosition(position, confidence);

            if (cv::norm(filteredPos - lens_center) <lens_radius ){
                //Draw position trail
                frame = frame_gray;
                cvtColor(frame, frame, CV_GRAY2RGB);
                converted = true;

                circle(frame, filteredPos, 4, 1234);
                std::vector<cv::Point>::iterator it;
                it = point_history.begin();
                point_history.insert(it, filteredPos);
                if (point_history.size() > 10) {
                    point_history.pop_back();
                }
            }

            int movementThreshold = 15;
            cv::Scalar color(0, 0, 255, 0.5);
            for (int i = 0; i < point_history.size() - 1; ++i) {
                cv::Point diff = point_history[i] - point_history[i + 1];
                int difference = std::sqrt(diff.dot(diff));
                //std::cout <<  "Difference " << difference << std::endl;
                int redness = std::min(255, difference);
                if (difference > movementThreshold) {
                    redness = 128;
                }
                cv::Scalar color(128 - redness, 128 - redness, redness * 2, 0.5);
                cv::line(frame, point_history[i], point_history[i + 1], color,
                         std::max(1, int(point_history.size() - i) / 2));
            }
        }
        if (!converted){
            cvtColor(frame_gray, frame, CV_GRAY2RGB);
        }
        // Draw GUI on frame
        cv::Point offset;
        drawOverlay(frame, offset);
        cv::Mat out = copyFrameROI(frame,200,200, lens_center);
        rotateImage90(out, clockwiseRotation);
        //Publish frame
        cv_ptr->image = out;
        //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
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
