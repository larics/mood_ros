#include <ros/ros.h>
#include <mood_ros/BlobDetectorParamsConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <mood_ros/detector_interface.hpp>

namespace mood_plugin {

class BlobDetector : public mood_base::detector_interface
{
public:
  BlobDetector() : detector_interface() { ROS_INFO("[BlobDetector] Hello World"); }

  sensor_comm::detection_response update(
    const sensor_comm::sensor_info &sensor_info) override
  {
    ROS_INFO("[BlobDetector] Update");
  }

  geometry_msgs::PoseArray get_object_poses() override
  {
    ROS_INFO("[BlobDetector] get_object_poses");
  }

  sensor_msgs::Image get_labeled_image() override
  {
    ROS_INFO("[BlobDetector] get_labeled_image");
  }
};
}// namespace mood_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::BlobDetector, mood_base::detector_interface);