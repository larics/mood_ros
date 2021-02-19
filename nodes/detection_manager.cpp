#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <mood_ros/detector_interface.hpp>
#include <uav_ros_lib/param_util.hpp>

using detector_loader_t = pluginlib::ClassLoader<mood_base::detector_interface>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_manager");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Load manager parameters
  std::string detector_plugin_name;
  param_util::getParamOrThrow(nh_private, "mood/detector", detector_plugin_name);

  // Load detector plugin
  auto plugin_loader =
    std::make_unique<detector_loader_t>("mood_ros", "mood_base::detector_interface");
  auto detector = plugin_loader->createUniqueInstance(detector_plugin_name);

  // Initialize the detector
  auto init_success = detector->initialize(nh, nh_private);
  if (!init_success) {
    ROS_FATAL("[DetectionManager] Detector initialization unsucessful!");
    return 1;
  }

  ros::spin();
  return 0;
}