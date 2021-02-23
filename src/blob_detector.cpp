#include <ros/ros.h>
#include <mood_ros/BlobDetectorParamsConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <mood_ros/detector_interface.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

namespace mood_plugin {

class BlobDetector : public mood_base::detector_interface
{
public:
  BlobDetector() : m_blob_detector_ptr(cv::SimpleBlobDetector::create())
  {
    ROS_INFO("[BlobDetector] Constructor");
  }

  sensor_comm::detection_response update(
    const sensor_comm::sensor_info &sensor_info) override
  {
    if (!m_is_initialized) {
      ROS_ERROR("[BlobDetector] Not initialized!");
      return { false, "[BlobDetector] Update unsuccessful, detector not initialized" };
    }

    if (!sensor_info.has_rgb) {
      return { false,
        "[BlobDetector] Update unsuccessful, sensor_info missing RGB information." };
    }

    if (!sensor_info.has_pointcloud) {
      return { false,
        "[BlobDetector] Update unsuccessful, sensor_info missing pointcloud "
        "information. " };
    }

    ROS_INFO_THROTTLE(5.0, "[BlobDetector] Update");

    // Convert the image to OpencCV
    auto cv_image_ptr =
      cv_bridge::toCvCopy(sensor_info.rgb_image, sensor_msgs::image_encodings::BGR8);

    // Do the Blob Detection
    std::vector<cv::KeyPoint> keypoints;
    m_blob_detector_ptr->detect(cv_image_ptr->image, keypoints);

    return { true, "[BlobDetector] Update successful." };
  }

  geometry_msgs::PoseArray get_object_poses() override
  {
    return {};
  }

  sensor_msgs::Image get_labeled_image() override
  {
    return {};
  }

  bool initialize(ros::NodeHandle &nh, ros::NodeHandle &nh_private) override
  {
    ROS_INFO("[BlobDetector] initialize");
    m_is_initialized = true;
    return true;
  }

private:
  bool m_is_initialized = false;
  cv::Ptr<cv::SimpleBlobDetector> m_blob_detector_ptr;
};
}// namespace mood_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::BlobDetector, mood_base::detector_interface);