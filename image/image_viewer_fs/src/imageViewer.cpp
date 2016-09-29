#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_viewer_fs/Angle.h>
#include <image_viewer_fs/ImageFilename.h>
#include <map>

const uint WIDTH = 320;
const uint HEIGHT = 240;

float g_angle = 90.0f;
std::string g_filename = "";

std::map<std::string, cv::Mat> g_images;

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

	cv::Point2f center(tmp.cols/2.0f, tmp.rows/2.0f);
	
	cv::Mat R = cv::getRotationMatrix2D(center, angle, 1.0);
	cv::Rect bBox = cv::RotatedRect(center, tmp.size(), angle).boundingRect();
	R.at<double>(0,2) += bBox.width/2.0 - center.x;
	R.at<double>(1,2) += bBox.height/2.0 - center.y;

	cv::Mat dst;
	cv::warpAffine(tmp, dst, R, bBox.size());

    cv::Mat background = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    dst.copyTo(background(cv::Rect((WIDTH-bBox.width)/2, (HEIGHT-bBox.height)/2, dst.cols, dst.rows)));

	return background;
}

void angleCallback(const image_viewer_fs::Angle::ConstPtr& msg)
{
    float oldAngle = g_angle;
    g_angle = msg->angle;

    if (oldAngle != g_angle) {
        if (g_images.end() != g_images.find(g_filename)) {
            cv::Mat image = g_images.find(g_filename)->second;
            cv::imshow("view", rotate(image, g_angle));
            cv::waitKey(30);
        }
    }
}

void imageFilenameCallback(const image_viewer_fs::ImageFilename::ConstPtr& msg) {
    std::string oldFilename = g_filename;
    g_filename = msg->imageFilename;

    if (oldFilename != g_filename) {
        cv::Mat image = cv::imread(g_filename, CV_LOAD_IMAGE_COLOR);

        if (!image.data) {
            std::cout << "Image not found : " << g_filename << std::endl;
        } else {
            cv::imshow("view", rotate(image, g_angle));
            cv::waitKey(30);

            // image not in map
            if (g_images.end() == g_images.find(g_filename)) {
                std::cout << "Added image to map : " << g_filename << std::endl;
                g_images.insert(std::pair<std::string, cv::Mat>(g_filename, image));
            } else {
                std::cout << "Image already in map : " << g_filename << std::endl;
            }
        }
    }
}

int main(int argc, char **argv)
    {
    ros::init(argc, argv, "image_viewer_fs");
    ros::NodeHandle nh;

    ros::Subscriber angleSub = nh.subscribe("imageAngle", 1000, angleCallback);
    ros::Subscriber filenameSub = nh.subscribe("imageFilename", 1000, imageFilenameCallback);


    cv::namedWindow("view", CV_WINDOW_NORMAL);
    cv::setWindowProperty("view", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    cv::startWindowThread();

    ros::spin();

    cv::destroyWindow("view");
}

