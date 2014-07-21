#include "apriltag_ros/pose_estimator.h"

#include <stdexcept>
#include <sstream>
#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace apriltag_ros {

using std::cout;
using std::endl;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfoConstPtr;

PoseEstimator::PoseEstimator(const ros::NodeHandle &nh) : nh_{nh}, it_{nh} {
  it_.subscribeCamera("image_raw", 1, &PoseEstimator::CameraCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_cam", 1);
  GenerateMap(tag_w_);
  cv::namedWindow("image");
}

void PoseEstimator::CameraCallback(const ImageConstPtr &image,
                                   const CameraInfoConstPtr &cinfo) {
  // Store camera info once
  if (!init_cam_) {
    init_cam_ = GetCameraInfo(cinfo, caminfo_);
  }

  // Get image
  cv::Mat image_gray, image_color;
  GetImage(image, image_gray, image_color);

  // Detect Apriltags
  std::vector<AprilTags::TagDetection> detections =
      tag_detector_.extractTags(image_gray);
  if (!detections.empty()) {
    std::vector<Point2> pi;  // Points in image
    std::vector<Point3> pw;  // Points in world
    for (const auto &tag : detections) {
      const auto id = tag.id;
      const Point2 c2 = Point2(tag.cxy.first, tag.cxy.second);

      for (int j = 0; j < 4; ++j) {
        const Point2 p2 = Point2(tag.p[j].first, tag.p[j].second);
        pi.push_back(p2);
        const Point3 p3(tag_w_[id].p[j].x, tag_w_[id].p[j].y, 0.0);
        pw.push_back(p3);
        // Display tag corners
        cv::circle(image_color, p2, 4, colors[j], 2);
      }
      // Display tag id
      std::ostringstream ss;
      ss << id;
      cv::putText(image_color, ss.str(), Point2(c2.x - 5, c2.y + 5),
                  CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255), 2);
    }
    // Undistort points
    cv::Mat pi_undistort;
    cv::undistortPoints(cv::Mat(pi), pi_undistort, caminfo_.K, caminfo_.D);
    // Calculate pose
    static cv::Mat r = cv::Mat::zeros(cv::Size(1, 3), CV_64F);
    static cv::Mat cTw = cv::Mat::zeros(cv::Size(1, 3), CV_64F);
    cv::Mat wTc(cv::Size(3, 3), CV_64F);
    cv::Mat cRw(cv::Size(3, 3), CV_64F);
    cv::Mat wRc(cv::Size(3, 3), CV_64F);
    cv::solvePnP(pw, pi_undistort, caminfo_.K, caminfo_.D, r, cTw, true);
    cv::Rodrigues(r, cRw);
    wRc = cRw.inv();
    wTc = -wRc * cTw;
    cv::Mat q = RodriguesToQuat(r);

    // Publish
    pose_.header = image->header;
    double *pt = wTc.ptr<double>();
    pose_.pose.position.x = pt[0];
    pose_.pose.position.y = pt[1];
    pose_.pose.position.z = pt[2];
    double *pq = q.ptr<double>();
    pose_.pose.orientation.w = pq[0];
    pose_.pose.orientation.x = pq[1];
    pose_.pose.orientation.y = pq[2];
    pose_.pose.orientation.z = pq[3];
    pose_pub_.publish(pose_);
  }

  // Display image
  cv::imshow("image", image_color);
  cv::waitKey(5);
}

void PoseEstimator::GetImage(const ImageConstPtr &image, cv::Mat &image_gray,
                             cv::Mat &image_color) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
  if (cv_ptr->image.type() == CV_8UC1) {
    image_gray = cv_ptr->image;
    cv::cvtColor(image_gray, image_color, CV_GRAY2RGB);
  } else if (cv_ptr->image.type() == CV_8UC3) {
    image_color = cv_ptr->image;
    cv::cvtColor(image_color, image_gray, CV_BGR2GRAY);
  }
}

bool PoseEstimator::GetCameraInfo(const CameraInfoConstPtr &cinfo,
                                  CamInfo &caminfo) {
  if (cinfo->K[0] == 0.0) return false;
  auto d = cinfo->D.size();  // Distortion size
  cv::Mat K = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
  cv::Mat D = cv::Mat::zeros(cv::Size(1, d), CV_64F);

  // Assign K and D
  for (decltype(K.rows) i = 0; i < 3; ++i) {
    double *pk = K.ptr<double>(i);
    for (int j = 0; j < 3; ++j) {
      pk[j] = cinfo->K[3 * i + j];
    }
  }
  double *pd = D.ptr<double>();
  for (decltype(d) k = 0; k < d; k++) {
    pd[k] = cinfo->D[k];
  }
  caminfo.K = K;
  caminfo.D = D;
  cout << label_ << "Camera initialized" << endl;
  cout << "K: " << K << endl << "D: " << D << endl;
  return true;
}

void PoseEstimator::GenerateMap(std::map<int, Tag> &tag_w) {
  double tag_size = 3.8 / 100;
  double tag_center[4][2] = {{tag_size, tag_size},
                             {tag_size, tag_size * 2},
                             {tag_size * 2, tag_size},
                             {tag_size * 2, tag_size * 2}};
  for (int i = 0; i < 4; ++i) {
    double x = tag_center[i][0];
    double y = tag_center[i][1];
    tag_w[i].p[0] = Point2(x - tag_size / 2, y - tag_size / 2);
    tag_w[i].p[1] = Point2(x + tag_size / 2, y - tag_size / 2);
    tag_w[i].p[2] = Point2(x + tag_size / 2, y + tag_size / 2);
    tag_w[i].p[3] = Point2(x - tag_size / 2, y + tag_size / 2);
  }
}

cv::Mat PoseEstimator::RodriguesToQuat(const cv::Mat &r) {
  // theta = norm(r)
  // q = [cos(theta/2), sin(theta/2) * r / theta]
  cv::Mat q = cv::Mat::zeros(cv::Size(1, 4), CV_64F);
  double *pq = q.ptr<double>();
  const double *pr = r.ptr<double>();
  double x, y, z;
  x = pr[0], y = pr[1], z = pr[2];
  double theta = std::sqrt(x * x + y * y + z * z);
  if (theta < std::numeric_limits<double>::epsilon() * 10.0) {
    pq[0] = 1.0;
    return q;
  }

  double haver_sin = std::sin(0.5 * theta);
  double haver_cos = std::cos(0.5 * theta);
  pq[0] = haver_cos;
  pq[1] = haver_sin * x / theta;
  pq[2] = haver_sin * y / theta;
  pq[3] = haver_sin * z / theta;

  return q;
}

}  // namespace apriltag_ros
