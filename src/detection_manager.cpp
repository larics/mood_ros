#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <mood_ros/detector_interface.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_manager");
  auto plugin_loader =
    std::make_unique<pluginlib::ClassLoader<mood_base::detector_interface>>(
      "mood_ros", "mood_base::detector_interface");
  auto detector = plugin_loader->createUniqueInstance("BlobDetector");
  ros::spin();
  return 0;
}