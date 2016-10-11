#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_viewer_fs/Angle.h>
#include <image_viewer_fs/Filename.h>
#include <image_viewer_fs/Scale.h>
#include <map>
#include <mutex>

const uint WIDTH = 320;
const uint HEIGHT = 240;

float g_scale = 1.0f;
float g_angle = 90.0f;
std::string g_filename = "";

std::map<std::string, cv::Mat> g_images;
std::mutex g_mapMutex; // write: unique access, read: shared access


bool imageIsLoaded(std::string filename);
bool loadImage(std::string filename, cv::Mat &out);
bool getImage(std::string filename, cv::Mat &out);
cv::Mat rotate(cv::Mat src, float angle);
cv::Mat scale(cv::Mat src, float scale);
bool updateImage(std::string filename);
void angleCallback(const image_viewer_fs::Angle::ConstPtr& msg);
void filenameCallback(const image_viewer_fs::Filename::ConstPtr& msg);
void scaleCallback(const image_viewer_fs::Scale::ConstPtr& msg);


bool imageIsLoaded(std::string filename)
{
    std::lock_guard<std::mutex> lock(g_mapMutex);
    return g_images.end() != g_images.find(filename);
}

bool loadImage(std::string filename, cv::Mat &out)
{
    // load image if it's not already loaded
    if (!imageIsLoaded(filename)) {
        out = cv::imread(g_filename, CV_LOAD_IMAGE_COLOR);

        // insert in dict if loading succeeds
        if (out.data){
            std::lock_guard<std::mutex> lock(g_mapMutex);
            g_images.insert(std::pair<std::string, cv::Mat>(g_filename, out));
            return true;
        }
    }

    // file not found
    return false;
}

bool getImage(std::string filename, cv::Mat &out)
{
    // return loaded image
    if (imageIsLoaded(filename)) {
        std::lock_guard<std::mutex> lock(g_mapMutex);
        out = g_images.find(g_filename)->second;
        return true;
    }
    // load image if not already loaded and return it
    else if (loadImage(filename, out)) {
        return true;
    }

    // image not found
    return false;
}

cv::Mat rotate(cv::Mat src, float angle)
{
    // scale down the input image if bigger than destination
    cv::Mat tmp;
    float diagonalSquared = src.cols*src.cols + src.rows*src.rows;
    float maxDSquared = WIDTH < HEIGHT ? WIDTH*WIDTH : HEIGHT*HEIGHT;

    if (diagonalSquared > maxDSquared) {
        float s = (float)sqrt(maxDSquared/diagonalSquared);
        cv::resize(src, tmp, cv::Size((int)(s * src.cols), (int)(s*src.rows)), CV_INTER_AREA);
    }
    else {
        src.copyTo(tmp);
    }

    // scale the image before adding it to the background
    tmp = scale(tmp, g_scale);

    cv::Point2f center(tmp.cols/2.0f, tmp.rows/2.0f);

    cv::Mat R = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::Rect bBox = cv::RotatedRect(center, tmp.size(), angle).boundingRect();
    R.at<double>(0,2) += bBox.width/2.0 - center.x;
    R.at<double>(1,2) += bBox.height/2.0 - center.y;

    cv::Mat dst;
    cv::warpAffine(tmp, dst, R, bBox.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

    cv::Mat background = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    background.setTo(cv::Scalar(255,255,255)); // white background
    dst.copyTo(background(cv::Rect((WIDTH-bBox.width)/2, (HEIGHT-bBox.height)/2, dst.cols, dst.rows)));

    return background;
}

cv::Mat scale(cv::Mat src, float scale)
{
    cv::Mat out;
    cv::resize(src, out, cv::Size(int(scale*src.cols), int(scale*src.rows)), CV_INTER_AREA);
    return out;
}

bool updateImage(std::string filename)
{
    cv::Mat image;
    if (getImage(g_filename, image)) {
        cv::imshow("view", rotate(image, g_angle));
        cv::waitKey(30);
        return true;
    }
    return false;
}

void angleCallback(const image_viewer_fs::Angle::ConstPtr& msg)
{
    float oldAngle = g_angle;
    g_angle = msg->angle;

    if (oldAngle != g_angle) {
        updateImage(g_filename);
    }
}

void filenameCallback(const image_viewer_fs::Filename::ConstPtr& msg)
{
    std::string oldFilename = g_filename;
    g_filename = msg->filename;

    if (oldFilename != g_filename) {
        updateImage(g_filename);
    }
}

void scaleCallback(const image_viewer_fs::Scale::ConstPtr& msg)
{
    float oldScale = g_scale;
    g_scale = std::min(std::max(0.0f, msg->scale), 1.0f);

    if (oldScale != g_scale) {
        updateImage(g_filename);
    }
}

int main(int argc, char **argv)
    {
    ros::init(argc, argv, "image_viewer_fs");
    ros::NodeHandle nh;

    ros::Subscriber angleSub = nh.subscribe("imageAngle", 1000, angleCallback);
    ros::Subscriber filenameSub = nh.subscribe("imageFilename", 1000, filenameCallback);
    ros::Subscriber scaleSub = nh.subscribe("imageScale", 1000, scaleCallback);

    cv::namedWindow("view", CV_WINDOW_NORMAL);
    cv::setWindowProperty("view", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    cv::startWindowThread();

    ros::spin();

    cv::destroyWindow("view");
}

