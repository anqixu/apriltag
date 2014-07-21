#include "apriltag_ros/pose_estimator.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_ros");
  ros::NodeHandle nh("~");

  apriltag_ros::PoseEstimator pose_estimator(nh);
  ros::spin();
}
