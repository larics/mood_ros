#include <ros/ros.h>
#include <mood_ros/BlobDetectorParamsConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <mood_ros/detector_interface.hpp>

namespace mood_plugin {

class BlobDetector : public mood_base::detector_interface
{
public:
  BlobDetector() { ROS_INFO("[BlobDetector] Constructor"); }

  sensor_comm::detection_response update(
    const sensor_comm::sensor_info &sensor_info) override
  {
    if (!m_is_initialized) {
      ROS_ERROR("[BlobDetector] Not initialized!");
      return { false, "[BlobDetector] Update unsuccessful, detector not initialized" };
    }

    ROS_INFO("[BlobDetector] Update");
    return { true, "[BlobDetector] Update successful." };
  }

  geometry_msgs::PoseArray get_object_poses() override
  {
    ROS_INFO("[BlobDetector] get_object_poses");
  }

  sensor_msgs::Image get_labeled_image() override
  {
    ROS_INFO("[BlobDetector] get_labeled_image");
  }

  bool initialize(ros::NodeHandle &nh, ros::NodeHandle &nh_private) override
  {
    ROS_INFO("[BlobDetector] initialize");
    m_is_initialized = true;
    return true;
  }

private:
  bool m_is_initialized = false;
  
};
}// namespace mood_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::BlobDetector, mood_base::detector_interface);