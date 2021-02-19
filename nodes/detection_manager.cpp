#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mood_ros/detector_interface.hpp>
#include <mood_ros/msg_sync_interface.hpp>
#include <uav_ros_lib/param_util.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

using detector_loader_t = pluginlib::ClassLoader<mood_base::detector_interface>;
using sync_loader_t = pluginlib::ClassLoader<mood_base::msg_sync_interface>;

// Define synchronization message types
using ros_image_t = sensor_msgs::Image;
using ros_depth_t = sensor_msgs::Image;
using ros_pointcloud_t = sensor_msgs::PointCloud2;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_manager");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Load manager parameters
  std::string detector_plugin_name;
  param_util::getParamOrThrow(nh_private, "mood/detector", detector_plugin_name);

  // Load detector plugin
  auto detector_plugin_loader =
    std::make_unique<detector_loader_t>("mood_ros", "mood_base::detector_interface");
  auto detector = detector_plugin_loader->createUniqueInstance(detector_plugin_name);

  // Initialize the detector
  auto init_success = detector->initialize(nh, nh_private);
  if (!init_success) {
    ROS_FATAL("[DetectionManager] Detector initialization unsucessful!");
    return 1;
  }

  // Load synchronization plugin
  auto sync_plugin_loader =
    std::make_unique<sync_loader_t>("mood_ros", "mood_base::msg_sync_interface");
  auto synchronizer =
    sync_plugin_loader->createUniqueInstance("PointcloudSynchronization");
  synchronizer->register_callback(
    [&](const sensor_comm::sensor_info &info) { auto success = detector->update(info); });
  std::vector<std::string> topic_names{ "pointcloud" };
  auto sync_init = synchronizer->initialize(nh, topic_names);
  if (!sync_init) {
    ROS_FATAL("[DetectionManager] Synchronizer initialization unsucessful!");
    return 1;
  }
  
  ros::spin();
  return 0;
}