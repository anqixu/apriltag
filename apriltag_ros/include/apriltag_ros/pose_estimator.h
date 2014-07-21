#ifndef APRILTAG_ROS_POSE_ESTIMATOR_H_
#define APRILTAG_ROS_POSE_ESTIMATOR_H_

#include <memory>
#include <map>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

#include "opencv2/core/core.hpp"

namespace apriltag_ros {

class PoseEstimator {
 public:
  using Point2 = cv::Point2d;
  using Point3 = cv::Point3d;

  PoseEstimator(const ros::NodeHandle &nh);
  PoseEstimator(const PoseEstimator &) = delete;
  PoseEstimator &operator=(const PoseEstimator &) = delete;

  void CameraCallback(const sensor_msgs::ImageConstPtr &image,
                      const sensor_msgs::CameraInfoConstPtr &cinfo);

  struct CamInfo {
    cv::Mat K;
    cv::Mat D;
  };

  // P4 -- P3
  // |  P0 |
  // P1 -- P2
  struct Tag {
    Point2 p[4];
  };

  const cv::Scalar colors[4] = {
      cv::Scalar(255, 0, 0, 0), cv::Scalar(0, 255, 0, 0),
      cv::Scalar(0, 0, 255, 0), cv::Scalar(255, 0, 255, 0)};

 private:
  std::string label_{"\033[0;33m[APRIL]:\033[0m "};
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  geometry_msgs::PoseStamped pose_;
  ros::Publisher pose_pub_;
  CamInfo caminfo_;
  bool init_cam_{false};
  std::map<int, Tag> tag_w_;

  AprilTags::TagDetector tag_detector_{AprilTags::tagCodes36h11};

  bool GetCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo,
                     CamInfo &caminfo);
  void GetImage(const sensor_msgs::ImageConstPtr &image, cv::Mat &image_gray,
                cv::Mat &image_color);

  void GenerateMap(std::map<int,Tag> &tag_w);
  cv::Mat RodriguesToQuat(const cv::Mat &r) ;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_POSE_ESTIMATOR_
