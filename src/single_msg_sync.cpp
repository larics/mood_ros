#include <message_filters/subscriber.h>
#include <mood_ros/msg_sync_interface.hpp>

namespace mood_plugin {

class PointcloudSynchronization : public mood_base::msg_sync_interface
{
public:
  PointcloudSynchronization() { ROS_INFO("[PointcloudSynchronization] Constructor"); }
  bool initialize(ros::NodeHandle &nh, std::vector<std::string> &topic_names) override
  {
    try {
      m_sub_ptr = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
        nh, topic_names.front(), 1);
      m_sub_ptr->registerCallback([&](const sensor_msgs::PointCloud2ConstPtr& msg) {
        sensor_comm::sensor_info info;
        info.pointcloud_available = true;
        info.pointcloud = *msg;
        this->add_sensor_data(info);
      });
    } catch (std::runtime_error &e) {
      ROS_ERROR_STREAM("[PointcloudSynchronization] initialize failed " << e.what());
      return false;
    }
    return true;
  }

private:
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_sub_ptr;
};
}// namespace mood_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::PointcloudSynchronization, mood_base::msg_sync_interface);