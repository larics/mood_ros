#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

#include <mood_ros/detector_interface.hpp>
#include <mood_ros/msg_sync_interface.hpp>
#include <mood_ros/detection_tracker.hpp>
#include <uav_ros_lib/param_util.hpp>

// Distance between poses needed by the DetectionTracker
double pose_distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
  return sqrt((p1.position.x - p2.position.x) * (p1.position.x - p2.position.x)
              + (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y)
              + (p1.position.z - p2.position.z) * (p1.position.z - p2.position.z));
}

// DetectionTracker<T> requires a T operator<< overload, in this case geometry_msgs::Pose
// NOTE: put this function in the same namesapce as the DetectionTracker i.e. mood_tracker
namespace mood_tracker {
std::ostream &operator<<(std::ostream &stream, const geometry_msgs::Pose &pose)
{
  stream << " [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
         << "] ";
  return stream;
}
}// namespace mood_tracker

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
  auto detected_poses_pub = nh.advertise<geometry_msgs::PoseArray>("mood/pose_array", 1);
  auto tracked_pose_pub =
    nh.advertise<geometry_msgs::PoseStamped>("mood/tracked_pose", 1);

  // Load synchronization plugin
  auto sync_plugin_loader =
    std::make_unique<sync_loader_t>("mood_ros", "mood_base::msg_sync_interface");
  auto synchronizer = sync_plugin_loader->createUniqueInstance(synchronizer_plugin_name);

  PoseTracker pose_tracker(50, pose_distance);
  synchronizer->register_callback([&](const sensor_comm::sensor_info &info) {
    // First, update the detector
    auto resp = detector->update(info);
    if (!resp.status) {
      ROS_ERROR_STREAM(
        "[DetectionManager] Detector failed with message: " << resp.response);
    };

    // Detection is successful, publish topics
    auto detected_pose_array = detector->get_object_poses();
    labeled_img_pub.publish(detector->get_labeled_image());
    detected_poses_pub.publish(detected_pose_array);

    // Update tracker & Publish tracked if possible
    auto [tracker_status, tracked_pose] =
      pose_tracker.updateAllCentroids(detected_pose_array.poses);

    if (!tracker_status) {
      ROS_FATAL_THROTTLE(1.0, "[DetectionManager] Tracking failed");
      // TODO(lmark): Maybe do something if tracking fails :-)
    }
    
    geometry_msgs::PoseStamped tracked_pose_stamped;
    tracked_pose_stamped.header = detected_pose_array.header;
    tracked_pose_stamped.pose = tracked_pose;
    tracked_pose_pub.publish(tracked_pose_stamped);
  });
  auto sync_init = synchronizer->initialize(nh);
  if (!sync_init) {
    ROS_FATAL("[DetectionManager] Synchronizer initialization unsucessful!");
    return 1;
  }

  ros::spin();
  return 0;
}