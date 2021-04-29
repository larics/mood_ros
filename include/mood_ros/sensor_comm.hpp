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
  bool                             has_pointcloud;
  sensor_msgs::PointCloud2ConstPtr pointcloud;

  bool                       has_rgb;
  sensor_msgs::ImageConstPtr rgb_image;

  bool                       has_depth;
  sensor_msgs::ImageConstPtr depth_image;
};

/**
 * @brief Response message from the detection update request.
 *
 */
struct detection_response
{
  bool        status;
  std::string response;
};

}// namespace sensor_comm

#endif /* SENSOR_COMM_HPP */