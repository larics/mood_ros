#ifndef SENSOR_COMM_HPP
#define SENSOR_COMM_HPP

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

namespace sensor_comm {

/**
 * @brief Synchronized sensor information sent to the current object detector.
 *
 */
struct sensor_info
{
  bool pointcloud_available;
  sensor_msgs::PointCloud2 pointcloud;

  bool rgb_available;
  sensor_msgs::Image rgb_image;

  bool depth_available;
  sensor_msgs::Image depth_image;
};

/**
 * @brief Response message from the detection update request.
 * 
 */
struct detection_response
{
  bool status;
  std::string response;
};

}// namespace mood_comm

#endif /* SENSOR_COMM_HPP */