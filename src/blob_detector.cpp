#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mood_ros/detector_interface.hpp>

// OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// PCL Includes

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/intersections.h>
#include <pcl/point_types.h>

namespace mood_plugin {

struct Color
{
  static cv::Scalar RED;
  static cv::Scalar WHITE;
  static cv::Scalar BLACK;
};

cv::Scalar Color::RED(0, 0, 255);
cv::Scalar Color::WHITE(255, 255, 255);
cv::Scalar Color::BLACK(0, 0, 0);

using Pointcloud_t = pcl::PointCloud<pcl::PointXYZ>;

class BlobDetector : public mood_base::detector_interface
{
public:
  BlobDetector()
    : m_blob_detector_ptr(cv::SimpleBlobDetector::create()),
      m_cloud_ptr(boost::make_shared<Pointcloud_t>()),
      m_cvimage_ptr(boost::make_shared<cv_bridge::CvImage>())
  {
    ROS_INFO("[BlobDetector] Constructor");
    m_camera_to_base_link.setRPY(-1.57, 0, -1.57);
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
    m_cvimage_ptr =
      cv_bridge::toCvCopy(sensor_info.rgb_image, sensor_msgs::image_encodings::BGR8);
    pcl::fromROSMsg(sensor_info.pointcloud, *m_cloud_ptr);

    if (!m_cvimage_ptr) {
      return { false, "[BlobDetector] Image conversion to CVImage failed." };
    }

    if (!m_cloud_ptr) {
      return { false, "[BlobDetector] Pointcloud conversion to PCL failed." };
    }

    do_blob_detection(m_cvimage_ptr);

    if (m_blob_keypoints.empty()) {
      return { false, "[BlobDetector] No keypoints found. " };
    }

    compute_blob_centroids(m_cvimage_ptr->image.rows, m_cvimage_ptr->image.cols);


    // cv_bridge::CvImage mask_img(header, "mono8", color_mask);
    return { true, "[BlobDetector] Update successful." };
  }

  geometry_msgs::PoseArray get_object_poses() override
  {
    m_blob_poses.header.stamp = ros::Time::now();
    m_blob_poses.header.frame_id = "red/base_link";
    return m_blob_poses;
  }

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
      Color::RED,
      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  }

  std::tuple<geometry_msgs::Pose, bool> compute_blob_centroid(
    const cv::Mat &one_blob_mask)
  {
    tf2::Vector3 blob_pose{0, 0, 0};
    int count = 0;
    for (int i = 0; i < one_blob_mask.rows; i++) {
      for (int j = 0; j < one_blob_mask.cols; j++) {
        if (one_blob_mask.at<unsigned char>(i, j) == 0) { continue; }

        count++;
        auto point = m_cloud_ptr->at(j, i);
        blob_pose += tf2::Vector3{point.x, point.y, point.z};
      }
    }

    blob_pose /= count;

    if (!(std::isfinite(blob_pose.getX()) || std::isfinite(blob_pose.getY())
          || std::isfinite(blob_pose.getZ()))) {
      return { geometry_msgs::Pose{}, false };
    }

    blob_pose = m_camera_to_base_link * blob_pose;

    geometry_msgs::Pose new_blob_pose;
    new_blob_pose.position.x = blob_pose.x();
    new_blob_pose.position.y = blob_pose.y();
    new_blob_pose.position.z = blob_pose.z();
    new_blob_pose.orientation.x = 0;
    new_blob_pose.orientation.y = 0;
    new_blob_pose.orientation.z = 0;
    new_blob_pose.orientation.w = 1;
    return { new_blob_pose, true };
  }

  void compute_blob_centroids(int image_rows, int image_cols)
  {
    // Clear current blob poses
    m_blob_poses.poses.clear();

    // Go through all keypoints and find out their position
    cv::Mat debug_mask(image_rows, image_cols, CV_8UC1, Color::BLACK);
    for (const auto &keypoint : m_blob_keypoints) {
      cv::Mat one_blob_mask(image_rows, image_cols, CV_8UC1, Color::BLACK);

      // Draw a circle for the debug mask
      cv::circle(debug_mask,
        cv::Point(keypoint.pt.x, keypoint.pt.y),
        keypoint.size / 3.0,// keypoint.size is a diameter, not a radius. Duh...
        Color::WHITE,
        -1);

      // Draw a circle for the one blob mask
      cv::circle(one_blob_mask,
        cv::Point(keypoint.pt.x, keypoint.pt.y),
        keypoint.size / 3.0,
        Color::WHITE,
        -1);

      auto [blob_pose, success] = compute_blob_centroid(one_blob_mask);
      if (!success) { continue; }
      m_blob_poses.poses.emplace_back(blob_pose);
    }

    cv_bridge::CvImage debug_mask_cvimage;
    debug_mask_cvimage.header.stamp = ros::Time::now();
    debug_mask_cvimage.image = debug_mask;
    debug_mask_cvimage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    m_mask_debug_pub.publish(debug_mask_cvimage.toImageMsg());
  }

  bool m_is_initialized = false;
  tf2::Matrix3x3 m_camera_to_base_link;
  cv::Ptr<cv::SimpleBlobDetector> m_blob_detector_ptr;

  /* Blob label information */
  std::vector<cv::KeyPoint> m_blob_keypoints;
  cv::Mat m_labeled_image;
  geometry_msgs::PoseArray m_blob_poses;

  /* Image transport for debugging */
  std::unique_ptr<image_transport::ImageTransport> m_it_ptr;
  image_transport::Publisher m_mask_debug_pub;

  /* Sensor information ptrs */
  cv_bridge::CvImagePtr m_cvimage_ptr;
  Pointcloud_t::Ptr m_cloud_ptr;
};
}// namespace mood_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::BlobDetector, mood_base::detector_interface);