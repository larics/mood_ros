#include <memory>
#include <cmath>

// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ROS package includes
#include <uav_ros_lib/param_util.hpp>
#include <uav_ros_lib/reconfigure_handler.hpp>
#include <mood_ros/detector_interface.hpp>
#include <mood_ros/BlobDetectorParamsConfig.h>

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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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
using BlobDetectorParams_t = mood_ros::BlobDetectorParamsConfig;
using BlobDetectorReCfg_t = ros_util::ReconfigureHandler<BlobDetectorParams_t>;

class BlobDetector : public mood_base::detector_interface
{
public:
  BlobDetector()
    : m_blob_detector_ptr(cv::SimpleBlobDetector::create()),
      m_cloud_ptr(boost::make_shared<Pointcloud_t>()),
      m_cvimage_ptr(boost::make_shared<cv_bridge::CvImage>()), m_tf_listener(m_tf_buffer)
  {
    ROS_INFO("[BlobDetector] Constructor");
  }

  sensor_comm::detection_response update(
    const sensor_comm::sensor_info &sensor_info) override
  {
    // Clear current blob poses
    m_blob_poses.poses.clear();

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

    // Try to get a transform
    if (!m_is_transform_initialized) {
      auto [transform_success, message] = initialize_transform();
      if (!transform_success) { return { false, message }; }

      // Otherwise transform initialization was successful
      m_is_transform_initialized = true;
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
      m_labeled_image = m_cvimage_ptr->image;
      return { false, "[BlobDetector] No keypoints found. " };
    }

    compute_blob_centroids(m_cvimage_ptr->image.rows, m_cvimage_ptr->image.cols);


    // cv_bridge::CvImage mask_img(header, "mono8", color_mask);
    return { true, "[BlobDetector] Update successful." };
  }

  geometry_msgs::PoseArray get_object_poses() override
  {
    m_blob_poses.header.stamp = ros::Time::now();
    m_blob_poses.header.frame_id = m_base_link;
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

    std::string tf_prefix;

    BlobDetectorParams_t params;
    try {
      // Params loaded through yaml file
      param_util::getParamOrThrow(nh_private, "blob_detector/base_link", m_base_link);
      param_util::getParamOrThrow(nh_private, "blob_detector/camera_link", m_camera_link);
      param_util::getParamOrThrow(
        nh_private, "blob_detector/filter_by_area", params.filter_by_area);
      param_util::getParamOrThrow(nh_private, "blob_detector/min_area", params.min_area);
      param_util::getParamOrThrow(nh_private, "blob_detector/max_area", params.max_area);
      param_util::getParamOrThrow(
        nh_private, "blob_detector/filter_by_color", params.filter_by_color);
      param_util::getParamOrThrow(
        nh_private, "blob_detector/blob_color", params.blob_color);
      param_util::getParamOrThrow(
        nh_private, "blob_detector/filter_by_circularity", params.filter_by_circularity);
      param_util::getParamOrThrow(
        nh_private, "blob_detector/min_circularity", params.min_circularity);
      param_util::getParamOrThrow(
        nh_private, "blob_detector/max_circularity", params.max_circularity);

      // Params loaded directly through launch file
      param_util::getParamOrThrow(nh_private, "blob_detector/tf_prefix", tf_prefix);
    } catch (std::runtime_error &e) {
      ROS_ERROR_STREAM("[BlobDetector] Parameter initialization failed " << e.what());
      return false;
    }

    // Add namespace prefixes
    if (!tf_prefix.empty()) {
      m_base_link = tf_prefix + "/" + m_base_link;
      m_camera_link = tf_prefix + "/" + m_camera_link;
    }

    // Setup reconfigure handler
    m_blob_param_handler_ptr =
      std::make_unique<BlobDetectorReCfg_t>(params, "mood/blob_detector");

    ROS_INFO("[BlobDetector] Initialized Successfully");
    return true;
  }

private:
  std::tuple<bool, std::string> initialize_transform()
  {
    try {
      auto m_camera_to_base_link =
        m_tf_buffer.lookupTransform(m_base_link, m_camera_link, ros::Time(0));

      // Save the obtained transform
      m_camera_to_base_link_trans =
        Eigen::Vector3f(m_camera_to_base_link.transform.translation.x,
          m_camera_to_base_link.transform.translation.y,
          m_camera_to_base_link.transform.translation.z);
      m_camera_to_base_link_rot =
        Eigen::Quaternionf(m_camera_to_base_link.transform.rotation.w,
          m_camera_to_base_link.transform.rotation.x,
          m_camera_to_base_link.transform.rotation.y,
          m_camera_to_base_link.transform.rotation.z);
    } catch (std::runtime_error &e) {
      std::string message;
      message += "[BlobDetector] Transform initialization failed with message ";
      message += e.what();
      return { false, message };
    }

    return { true, "[BlobDetector] Transform initialization successful." };
  }

  void do_blob_detection(const cv_bridge::CvImagePtr &cv_image_ptr)
  {
    // Clear existing keypoints
    m_blob_keypoints.clear();

    // Get params from reconfigure handler
    auto reconf_params = m_blob_param_handler_ptr->getData();
    auto blob_params = cv::SimpleBlobDetector::Params();

    // Set corresponding parameters
    blob_params.filterByArea = reconf_params.filter_by_area;
    blob_params.minArea = reconf_params.min_area;
    blob_params.maxArea = reconf_params.max_area;
    blob_params.filterByCircularity = reconf_params.filter_by_circularity;
    blob_params.minCircularity = reconf_params.min_circularity;
    blob_params.maxCircularity = reconf_params.max_circularity;
    blob_params.filterByColor = reconf_params.filter_by_color;
    blob_params.blobColor = reconf_params.blob_color;
    m_blob_detector_ptr = cv::SimpleBlobDetector::create(blob_params);

    m_blob_detector_ptr->detect(cv_image_ptr->image, m_blob_keypoints);
    cv::drawKeypoints(cv_image_ptr->image,
      m_blob_keypoints,
      m_labeled_image,
      Color::RED,
      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  }

  Eigen::Quaternionf compute_blob_orientation(const Pointcloud_t::Ptr &blob_pointcloud)
  {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_PROSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(blob_pointcloud);

    auto inliers = boost::make_shared<pcl::PointIndices>();
    auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
    seg.segment(*inliers, *coefficients);

    Eigen::Vector3f model_normal{
      coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2)
    };
    model_normal = m_camera_to_base_link_rot * model_normal;
    model_normal.normalize();

    Eigen::Quaternionf orientation(Eigen::AngleAxisf(
      atan2(model_normal(1), model_normal(0)), Eigen::Vector3f::UnitZ()));
    return orientation;
  }

  std::tuple<geometry_msgs::Pose, bool> compute_blob_centroid(
    const cv::Mat &one_blob_mask)
  {
    Eigen::Vector3f blob_position{ 0, 0, 0 };
    auto blob_pointcloud = boost::make_shared<Pointcloud_t>();
    int count = 0;
    for (int i = 0; i < one_blob_mask.rows; i++) {
      for (int j = 0; j < one_blob_mask.cols; j++) {
        if (one_blob_mask.at<unsigned char>(i, j) == 0) { continue; }

        count++;
        auto &point = m_cloud_ptr->at(j, i);
        blob_pointcloud->push_back(point);
        blob_position += Eigen::Vector3f{ point.x, point.y, point.z };
      }
    }

    blob_position /= count;

    if (!(std::isfinite(blob_position(0)) || std::isfinite(blob_position(1))
          || std::isfinite(blob_position(2)))) {
      return { geometry_msgs::Pose{}, false };
    }

    blob_position = m_camera_to_base_link_rot * blob_position;
    blob_position += m_camera_to_base_link_trans;
    auto blob_orientation = compute_blob_orientation(blob_pointcloud);

    geometry_msgs::Pose new_blob_pose;
    new_blob_pose.position.x = blob_position(0);
    new_blob_pose.position.y = blob_position(1);
    new_blob_pose.position.z = blob_position(2);
    new_blob_pose.orientation.x = blob_orientation.x();
    new_blob_pose.orientation.y = blob_orientation.y();
    new_blob_pose.orientation.z = blob_orientation.z();
    new_blob_pose.orientation.w = blob_orientation.w();
    return { new_blob_pose, true };
  }

  void compute_blob_centroids(int image_rows, int image_cols)
  {
    // Go through all keypoints and find out their position
    cv::Mat debug_mask(image_rows, image_cols, CV_8UC1, Color::BLACK);
    for (const auto &keypoint : m_blob_keypoints) {
      cv::Mat one_blob_mask(image_rows, image_cols, CV_8UC1, Color::BLACK);

      // Draw a circle for the debug mask
      cv::circle(debug_mask,
        cv::Point(keypoint.pt.x, keypoint.pt.y),
        keypoint.size / 2.0,// keypoint.size is a diameter, not a radius. Duh...
        Color::WHITE,
        -1);

      // Draw a circle for the one blob mask
      cv::circle(one_blob_mask,
        cv::Point(keypoint.pt.x, keypoint.pt.y),
        keypoint.size / 2.0,
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

  /* General Blob Detector infoo */
  bool m_is_initialized = false;
  std::string m_camera_link;
  std::string m_base_link;
  std::unique_ptr<BlobDetectorReCfg_t> m_blob_param_handler_ptr;

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

  /* Camera to base link transform information */
  bool m_is_transform_initialized;
  Eigen::Quaternionf m_camera_to_base_link_rot;
  Eigen::Vector3f m_camera_to_base_link_trans;
  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
};// namespace mood_plugin
}// namespace mood_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_plugin::BlobDetector, mood_base::detector_interface);