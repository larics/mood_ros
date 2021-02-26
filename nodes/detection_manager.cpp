#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

#include <mood_ros/detector_interface.hpp>
#include <mood_ros/msg_sync_interface.hpp>
#include <mood_ros/detection_tracker.hpp>
#include <uav_ros_lib/param_util.hpp>

double pose_distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
  return sqrt((p1.position.x - p2.position.x) * (p1.position.x - p2.position.x)
              + (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y)
              + (p1.position.z - p2.position.z) * (p1.position.z - p2.position.z));
}

using detector_loader_t = pluginlib::ClassLoader<mood_base::detector_interface>;
using sync_loader_t = pluginlib::ClassLoader<mood_base::msg_sync_interface>;
using PoseTracker = mood_tracker::DetectionTracker<geometry_msgs::Pose>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_manager");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Load manager parameters
  std::string detector_plugin_name, synchronizer_plugin_name;
  param_util::getParamOrThrow(nh_private, "mood/detector", detector_plugin_name);
  param_util::getParamOrThrow(nh_private, "mood/synchronizer", synchronizer_plugin_name);

  // Load detector plugin
  auto detector_plugin_loader =
    std::make_unique<detector_loader_t>("mood_ros", "mood_base::detector_interface");
  auto detector = detector_plugin_loader->createUniqueInstance(detector_plugin_name);

  // Initialize the detector
  auto init_success = detector->initialize(nh, nh_private);
  if (!init_success) {
    ROS_FATAL("[DetectionManager] Detector initialization unsucessful!");
    return 1;
  }

  image_transport::ImageTransport it(nh);
  auto labeled_img_pub = it.advertise("mood/labeled_image", 1);
  auto detected_poses_pub = nh.advertise<geometry_msgs::PoseArray>("mood/poses", 1);

  // Load synchronization plugin
  auto sync_plugin_loader =
    std::make_unique<sync_loader_t>("mood_ros", "mood_base::msg_sync_interface");
  auto synchronizer = sync_plugin_loader->createUniqueInstance(synchronizer_plugin_name);
  synchronizer->register_callback([&](const sensor_comm::sensor_info &info) {
    auto resp = detector->update(info);
    if (!resp.status) {
      ROS_ERROR_STREAM(
        "[DetectionManager] Detector failed with message: " << resp.response);
    };

    // Detection is successful, publish topics
    labeled_img_pub.publish(detector->get_labeled_image());
    detected_poses_pub.publish(detector->get_object_poses());
  });
  auto sync_init = synchronizer->initialize(nh);
  if (!sync_init) {
    ROS_FATAL("[DetectionManager] Synchronizer initialization unsucessful!");
    return 1;
  }

  PoseTracker pose_tracker(50, pose_distance);
  ros::spin();
  return 0;
}