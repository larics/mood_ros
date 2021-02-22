#ifndef MSG_SYNC_INTERFACE_HPP
#define MSG_SYNC_INTERFACE_HPP

#include <ros/ros.h>
#include <mood_ros/sensor_comm.hpp>
#include <functional>

namespace mood_base {

/**
 * @brief An interface used for message synchronization used for object detection.
 *
 */
class msg_sync_interface
{
public:
  using cb_t = std::function<void(const sensor_comm::sensor_info &)>;

  /**
   * @brief Register a callback to process all synchronized sensor information.
   *
   * @param callback A callback function carrying the sensor information
   */
  void register_callback(cb_t callback) { m_callback = std::move(callback); }

  /**
   * @brief Call this method when you are ready to forward sensor data to the registered
   * callback.
   *
   * @param info
   */
  void add_sensor_data(const sensor_comm::sensor_info &info)
  {
    ROS_INFO_COND(info.has_pointcloud,
      "[msg_sync_interface::add_sensor_data] Forward pointcloud data.");
    ROS_INFO_COND(
      info.has_rgb, "[msg_sync_interface::add_sensor_data] Forward RGB data.");
    ROS_INFO_COND(
      info.has_depth, "[msg_sync_interface::add_sensor_data] Forward Depth data.");

    if (m_callback) {
      m_callback(info);
    } else {
      ROS_WARN("[msg_sync_interface] Callback not registered.");
    }
  }

  /**
   * @brief Initialize the synchronization interface.
   *
   * @param nh A public ROS Node Handle.
   * @param topic_names List of topic names used for subscription.
   */
  virtual bool initialize(ros::NodeHandle &nh, std::vector<std::string> &topic_names) = 0;

protected:
  msg_sync_interface() { ROS_INFO("[msg_sync_interface] Constructor"); }
  cb_t m_callback;
};
}// namespace mood_base

#endif /* MSG_SYNC_INTERFACE_HPP */