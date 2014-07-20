#include "apriltag_ros/usb_camera.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera");
  ros::NodeHandle nh("~");

  apriltag_ros::UsbCamera usb_camera(nh);
  usb_camera.Run();
  ros::spin();
  usb_camera.End();
}
