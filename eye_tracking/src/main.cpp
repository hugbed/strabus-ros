

#include "eyeTracker.h"



/**
 * @function main
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eye_tracker");
  EyeTracker tracker;
  ros::spin();
  return 0;
}
