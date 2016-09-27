#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

const uint WIDTH = 1920; // 320
const uint HEIGHT = 1080; // 240

cv::Mat rotate(cv::Mat src, double angle)
{
	cv::Point2f center(src.cols/2.0f, src.rows/2.0f);
	
	cv::Mat R = cv::getRotationMatrix2D(center, angle, 1.0);
	cv::Rect bBox = cv::RotatedRect(center, src.size(), angle).boundingRect();
	R.at<double>(0,2) += bBox.width/2.0 - center.x;
	R.at<double>(1,2) += bBox.height/2.0 - center.y;

	cv::Mat dst;
	cv::warpAffine(src, dst, R, bBox.size());

    cv::Mat background = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    dst.copyTo(background(cv::Rect((WIDTH-bBox.width)/2, (HEIGHT-bBox.height)/2, dst.cols, dst.rows)));

	return background;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::imshow("view", rotate(cv_ptr->image, 45));
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_viewer_fs");
  ros::NodeHandle nh;
  cv::namedWindow("view", CV_WINDOW_NORMAL);
  cv::setWindowProperty("view", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}

