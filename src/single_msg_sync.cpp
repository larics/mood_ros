#include <message_filters/subscriber.h>
#include <mood_ros/msg_sync_interface.hpp>

namespace mood_plugin {

class PointcloudSync : public mood_base::msg_sync_interface
{
public:
  PointcloudSync() { ROS_INFO("[PointcloudSync] Constructor"); }
  bool initialize(ros::NodeHandle &nh) override
  {
    try {
      m_sub_ptr = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
        nh, "pointcloud", 1);
      m_sub_ptr->registerCallback([&](const sensor_msgs::PointCloud2ConstPtr &msg) {
        ROS_INFO_THROTTLE(5.0, "[PointcloudSync::m_sub_ptr::callback]");
        sensor_comm::sensor_info info;
        info.has_pointcloud = true;
        info.pointcloud = *msg;
        this->add_sensor_data(info);
      });
    } catch (std::runtime_error &e) {
      ROS_ERROR_STREAM("[PointcloudSync] initialize failed " << e.what());
      return false;
    }
    return true;
  }

private:
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_sub_ptr;
};
}// namespace mood_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::PointcloudSync, mood_base::msg_sync_interface);