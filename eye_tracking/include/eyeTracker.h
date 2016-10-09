#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "kalmanFilter.h"
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

class EyeTracker {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher position_pub_;
    ros::Publisher filtered_position_pub_;
	ros::Publisher offset_pub_;
	KalmanFilter filter;
public:
    EyeTracker();

    ~EyeTracker();

private:
    void frameCallback(const sensor_msgs::ImageConstPtr &msg);

};