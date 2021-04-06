#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

#include <mood_ros/detector_interface.hpp>
#include <mood_ros/msg_sync_interface.hpp>
#include <mood_ros/detection_tracker.hpp>
#include <uav_ros_lib/param_util.hpp>
#include <uav_ros_lib/ros_convert.hpp>
#include <uav_ros_lib/estimation/constant_velocity_lkf.hpp>

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
  std::string detector_plugin_name, synchronizer_plugin_name, world_frame;
  param_util::getParamOrThrow(nh_private, "mood/detector", detector_plugin_name);
  param_util::getParamOrThrow(nh_private, "mood/synchronizer", synchronizer_plugin_name);
  param_util::getParamOrThrow(nh_private, "mood/world_frame", world_frame);

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
  auto filtered_pose_pub =
    nh.advertise<geometry_msgs::PoseStamped>("mood/filtered_pose", 1);

  // Load synchronization plugin
  auto sync_plugin_loader =
    std::make_unique<sync_loader_t>("mood_ros", "mood_base::msg_sync_interface");
  auto synchronizer = sync_plugin_loader->createUniqueInstance(synchronizer_plugin_name);

  // Initialize the pose tracker
  PoseTracker pose_tracker(50, pose_distance);

  // Initialize the switching service
  auto switching_service =
    nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      "mood/switch_tracking",
      [&](std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp) {
        if (pose_tracker.getMap().size() < 2) {
          resp.success = false;
          resp.message = "Tracking map has less than 2 entries";
          return true;
        }

        auto [success, message] = pose_tracker.nextTrackingID();
        resp.success = success;
        resp.message = message;
        return true;
      });

  bool new_pose_calculated = false;
  // Register the synchronizer callback, this is where the magic happens
  geometry_msgs::PoseStamped tracked_pose_stamped;
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

    // Update tracker
    auto [tracker_status, tracked_pose] =
      pose_tracker.updateAllCentroids(detected_pose_array.poses);

    if (!tracker_status) {
      ROS_FATAL_THROTTLE(1.0, "[DetectionManager] Tracking failed");
      // TODO(lmark): Maybe do something if tracking fails :-)
    }

    // Publish tracked pose
    tracked_pose_stamped.header = detected_pose_array.header;
    tracked_pose_stamped.pose = tracked_pose;
    tracked_pose_pub.publish(tracked_pose_stamped);
    new_pose_calculated = tracker_status;
  });

  // Initialize kalman filtering
  ConstantVelocityLKF lkf_x("lkf_x", nh_private);
  ConstantVelocityLKF lkf_y("lkf_y", nh_private);
  ConstantVelocityLKF lkf_z("lkf_z", nh_private);
  ConstantVelocityLKF lkf_heading("lkf_heading", nh_private);
  const auto kalman_dt = 0.02;
  geometry_msgs::PoseStamped filtered_pose_stamped;
  auto kalman_timer =
    nh.createTimer(ros::Duration(kalman_dt), [&](const ros::TimerEvent & /* unused */) {
      // Do the Kalman filtering
      lkf_x.estimateState(
        kalman_dt, { tracked_pose_stamped.pose.position.x }, new_pose_calculated);
      lkf_y.estimateState(
        kalman_dt, { tracked_pose_stamped.pose.position.y }, new_pose_calculated);
      lkf_z.estimateState(
        kalman_dt, { tracked_pose_stamped.pose.position.z }, new_pose_calculated);
      lkf_heading.estimateState(kalman_dt,
        { ros_convert::calculateYaw(tracked_pose_stamped.pose.orientation) },
        new_pose_calculated);

      // Publish filtered pose&
      filtered_pose_stamped.header = tracked_pose_stamped.header;
      filtered_pose_stamped.pose.position.x = lkf_x.getState()[0];
      filtered_pose_stamped.pose.position.y = lkf_y.getState()[0];
      filtered_pose_stamped.pose.position.z = lkf_z.getState()[0];
      filtered_pose_stamped.pose.orientation =
        ros_convert::calculate_quaternion(lkf_heading.getState()[0]);
      filtered_pose_pub.publish(filtered_pose_stamped);

      if (new_pose_calculated) { new_pose_calculated = false; }
    });


  auto world_tracked_pub =
    nh.advertise<geometry_msgs::PoseStamped>("mood/world_tracked_pose", 1);
  auto odom_sub = nh.subscribe<nav_msgs::Odometry>(
    "odometry", 1, [&](const nav_msgs::OdometryConstPtr &msg) {
      geometry_msgs::PoseStamped world_tracked_pose;

      tf2::Matrix3x3 vehicle_transformation;
      auto vehicle_q = tf2::Quaternion{ msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w };
      vehicle_transformation.setRotation(vehicle_q);

      // Get tracked world position
      auto world_tracked_position = vehicle_transformation
                                    * tf2::Vector3{ filtered_pose_stamped.pose.position.x,
                                        filtered_pose_stamped.pose.position.y,
                                        filtered_pose_stamped.pose.position.z };

      tf2::Matrix3x3 tracked_transformation;
      tracked_transformation.setRotation(
        tf2::Quaternion{ tracked_pose_stamped.pose.orientation.x,
          tracked_pose_stamped.pose.orientation.y,
          tracked_pose_stamped.pose.orientation.z,
          tracked_pose_stamped.pose.orientation.w });
      auto world_tracked_orientation = vehicle_transformation * tracked_transformation;

      // Get tracked world orientation
      tf2::Quaternion world_tracked_q;
      world_tracked_orientation.getRotation(world_tracked_q);

      // Get directional vectors
      double tracked_yaw, tracked_pitch, tracked_roll;
      double vehicle_yaw, vehicle_pitch, vehicle_roll;
      world_tracked_orientation.getEulerZYX(tracked_yaw, tracked_pitch, tracked_roll);
      vehicle_transformation.getEulerZYX(vehicle_yaw, vehicle_pitch, vehicle_roll);

      // Define directional vectors
      tf2::Vector3 tracked_direction{ cos(tracked_yaw), sin(tracked_yaw), 0 };
      tf2::Vector3 vehicle_direction{ cos(vehicle_yaw), sin(vehicle_yaw), 0 };

      // Get the tracked orientation whose dot product is positiove wrt. the vehicle axis
      if (tracked_direction.dot(vehicle_direction) < 0) {
        tf2::Matrix3x3 rotate;
        rotate.setEulerZYX(M_PI, 0, 0);
        (world_tracked_orientation * rotate).getRotation(world_tracked_q);
      }

      // Publish !
      world_tracked_pose.header.stamp = ros::Time::now();
      world_tracked_pose.header.frame_id = world_frame;
      world_tracked_pose.pose.position.x =
        world_tracked_position.x() + msg->pose.pose.position.x;
      world_tracked_pose.pose.position.y =
        world_tracked_position.y() + msg->pose.pose.position.y;
      world_tracked_pose.pose.position.z =
        world_tracked_position.z() + msg->pose.pose.position.z;
      world_tracked_pose.pose.orientation.x = world_tracked_q.x();
      world_tracked_pose.pose.orientation.y = world_tracked_q.y();
      world_tracked_pose.pose.orientation.z = world_tracked_q.z();
      world_tracked_pose.pose.orientation.w = world_tracked_q.w();
      world_tracked_pub.publish(world_tracked_pose);
    });

  // Initialize the synchronizer
  auto sync_init = synchronizer->initialize(nh);
  if (!sync_init) {
    ROS_FATAL("[DetectionManager] Synchronizer initialization unsucessful!");
    return 1;
  }

  ros::spin();
  return 0;
}