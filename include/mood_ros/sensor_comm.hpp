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

  void add(const sensor_msgs::PointCloud2 &t) {
      pointcloud = t;
  }

  void add(const sensor_msgs::Image &t) {
    rgb_image = t;
  }
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

}// namespace sensor_comm

#endif /* SENSOR_COMM_HPP */