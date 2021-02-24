#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <mood_ros/BlobDetectorParamsConfig.h>
#include <mood_ros/detector_interface.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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

    do_blob_detection(cv_image_ptr);

    if (m_blob_keypoints.empty()) {
      return { false, "[BlobDetector] No keypoints found. " };
    }
    
    compute_keypoint_centroids(cv_image_ptr->image.rows, cv_image_ptr->image.cols);

    // cv_bridge::CvImage mask_img(header, "mono8", color_mask);
    return { true, "[BlobDetector] Update successful." };
  }

  geometry_msgs::PoseArray get_object_poses() override { return {}; }

  sensor_msgs::Image get_labeled_image() override
  {
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now();
    cv_image.image = m_labeled_image;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    return *(cv_image.toImageMsg());
  }

  bool initialize(ros::NodeHandle &nh, ros::NodeHandle &nh_private) override
  {
    ROS_INFO("[BlobDetector] initialize");
    m_is_initialized = true;

    m_it_ptr = std::make_unique<image_transport::ImageTransport>(nh);
    m_mask_debug_pub = m_it_ptr->advertise("mood/blob_detector/debug/mask", 1);
    return true;
  }

private:
  void do_blob_detection(const cv_bridge::CvImagePtr &cv_image_ptr)
  {
    m_blob_keypoints.clear();
    m_blob_detector_ptr->detect(cv_image_ptr->image, m_blob_keypoints);
    cv::drawKeypoints(cv_image_ptr->image,
      m_blob_keypoints,
      m_labeled_image,
      cv::Scalar(0, 0, 255),
      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  }

  void compute_keypoint_centroids(int image_rows, int image_cols)
  {
    // Go through all keypoints and find out their position
    cv::Mat debug_mask(image_rows, image_cols, CV_8UC1, cv::Scalar(0, 0, 0));
    for (const auto &keypoint : m_blob_keypoints) {
      cv::circle(debug_mask,
        cv::Point(keypoint.pt.x, keypoint.pt.y),
        keypoint.size / 2.0,// keypoint.size is a diameter, not the radius
        cv::Scalar(255, 255, 255),
        -1);
    }

    cv_bridge::CvImage debug_mask_cvimage;
    debug_mask_cvimage.header.stamp = ros::Time::now();
    debug_mask_cvimage.image = debug_mask;
    debug_mask_cvimage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    m_mask_debug_pub.publish(debug_mask_cvimage.toImageMsg());
  }

  bool m_is_initialized = false;
  cv::Ptr<cv::SimpleBlobDetector> m_blob_detector_ptr;

  /* Blob label information */
  std::vector<cv::KeyPoint> m_blob_keypoints;
  cv::Mat m_labeled_image;

  /* Image transport for debugging */
  std::unique_ptr<image_transport::ImageTransport> m_it_ptr;
  image_transport::Publisher m_mask_debug_pub;
};
}// namespace mood_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::BlobDetector, mood_base::detector_interface);