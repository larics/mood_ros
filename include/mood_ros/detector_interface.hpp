#ifndef DETECTOR_INTERFACE_HPP
#define DETECTOR_INTERFACE_HPP

#include <ros/ros.h>
#include <mood_ros/sensor_comm.hpp>
#include <geometry_msgs/PoseArray.h>

namespace mood_base {

/**
 * @brief Interface used for implementing object detection algorithms.
 *
 */
class detector_interface
{
public:
  /**
   * @brief This method is called each time when new sensor information is obtained.
   *
   * @param sensor_info Currently available sensor information
   * @return sensor_comm::detection_response  Response from the updated detection process.
   */
  virtual sensor_comm::detection_response update(
    const sensor_comm::sensor_info& sensor_info) = 0;

  /**
   * @brief Get the object poses found by the detector.
   *
   * @return geometry_msgs::PoseArray
   */
  virtual geometry_msgs::PoseArray get_object_poses() = 0;

  /**
   * @brief Get the labeled image object.
   *
   * @return sensor_msgs::Image
   */
  virtual sensor_msgs::Image get_labeled_image() = 0;

  /**
   * @brief Initialize the detector.
   *
   * @param nh A public ROS Node Handle.
   * @param nh_private A private ROS Node Handle.
   * @return true Detector initialization successful.
   * @return false Detector initialization successful.
   */
  virtual bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private) = 0;

protected:
  detector_interface() { ROS_INFO("[detector_interface] Constructor"); }
};

}// namespace mood_base

#endif /* DETECTOR_INTERFACE_HPP */