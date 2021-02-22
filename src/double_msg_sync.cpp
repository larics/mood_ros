#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <mood_ros/msg_sync_interface.hpp>

namespace mood_plugin {
class PointcloudRGBSync : public mood_base::msg_sync_interface
{
public:
  using PointcloudRGBPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
      sensor_msgs::Image>;
  using Synchronizer = message_filters::Synchronizer<PointcloudRGBPolicy>;
  using PointcloudSub_t = message_filters::Subscriber<sensor_msgs::PointCloud2>;
  using ImageSub_t = image_transport::SubscriberFilter;

  PointcloudRGBSync() { ROS_INFO("[PointcloudRGBSync] Constructor"); }

  bool initialize(ros::NodeHandle &nh) override
  {
    try {

      m_it_ptr = std::make_unique<image_transport::ImageTransport>(nh);
      m_pointcloud_sub_ptr = std::make_unique<PointcloudSub_t>(nh, "pointcloud", 1);
      m_image_sub_ptr = std::make_unique<ImageSub_t>(*m_it_ptr, "image", 1);
      m_syncer_ptr = std::make_unique<Synchronizer>(
        PointcloudRGBPolicy(10), *m_pointcloud_sub_ptr, *m_image_sub_ptr);
      m_syncer_ptr->registerCallback(
        boost::bind(&PointcloudRGBSync::callback, this, _1, _2));
    } catch (std::runtime_error &e) {
      ROS_ERROR_STREAM("[PointcloudRGBSync] initialize failed with " << e.what());
      return false;
    }
    return true;
  }

private:
  void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
    const sensor_msgs::ImageConstPtr &img_msg)
  {
    ROS_INFO_THROTTLE(5.0, "[PointcloudRGBSync::callback]");
    sensor_comm::sensor_info sensor_info;
    sensor_info.has_pointcloud = true;
    sensor_info.pointcloud = *cloud_msg;
    sensor_info.has_rgb = true;
    sensor_info.rgb_image = *img_msg;
    this->add_sensor_data(sensor_info);
  }

  std::unique_ptr<image_transport::ImageTransport> m_it_ptr;
  std::unique_ptr<PointcloudSub_t> m_pointcloud_sub_ptr;
  std::unique_ptr<ImageSub_t> m_image_sub_ptr;
  std::unique_ptr<Synchronizer> m_syncer_ptr;
};
}// namespace mood_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::PointcloudRGBSync, mood_base::msg_sync_interface);