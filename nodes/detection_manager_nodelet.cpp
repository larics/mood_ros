#include <ros/ros.h>
#include <nodelet/nodelet.h>
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

#include <mutex>

// Distance between poses needed by the DetectionTracker
double pose_distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
  return sqrt((p1.position.x - p2.position.x) * (p1.position.x - p2.position.x)
              + (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y)
              + (p1.position.z - p2.position.z) * (p1.position.z - p2.position.z));
}

// DetectionTracker<T> requires a T operator<< overload, in this case geometry_msgs::Pose
// NOTE: put this function in the same namesapce as the DetectionTracker i.e. mood_tracker
namespace mood_tracker {
std::ostream& operator<<(std::ostream& stream, const geometry_msgs::Pose& pose)
{
  stream << " [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
         << "] ";
  return stream;
}
}// namespace mood_tracker

using detector_loader_t = pluginlib::ClassLoader<mood_base::detector_interface>;
using sync_loader_t     = pluginlib::ClassLoader<mood_base::msg_sync_interface>;
using PoseTracker       = mood_tracker::DetectionTracker<geometry_msgs::Pose>;

namespace mood_ros {

class DetectionManager : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  // Switching service
  ros::ServiceServer m_switching_srv;
  bool switching_srv_cb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

  // Odometry subscriber
  ros::Subscriber m_odom_sub;
  void            odom_cb(const nav_msgs::OdometryConstPtr& msg);

  // Detector plugin
  std::unique_ptr<detector_loader_t>               m_detector_loader_ptr;
  boost::shared_ptr<mood_base::detector_interface> m_detector_ptr;
  std::mutex                                       m_detector_mutex;

  // Synchronizer plugin
  std::unique_ptr<sync_loader_t>                   m_syncrhonizer_loader_ptr;
  boost::shared_ptr<mood_base::msg_sync_interface> m_synchronizer_ptr;
  void synchronizer_cb(const sensor_comm::sensor_info& info);

  // Pose Tracker
  std::mutex  m_tracker_mutex;
  PoseTracker pose_tracker{ 50, pose_distance };

  // Kalman Filters
  double                               m_kalman_dt = 0.02;
  std::unique_ptr<ConstantVelocityLKF> m_lkf_x;
  std::unique_ptr<ConstantVelocityLKF> m_lkf_y;
  std::unique_ptr<ConstantVelocityLKF> m_lkf_z;
  ros::Timer                           m_kalman_timer;
  void                                 kalman_event(const ros::TimerEvent& e);

  // Detection Manager Parameters
  std::string m_detector_name;
  std::string m_sync_name;
  std::string m_world_frame;
  bool        m_new_pose_calculated;
  bool        m_initialized = false;

  // ROS Publishers
  image_transport::Publisher m_labeled_img_pub;
  ros::Publisher             m_detected_poses_pub;

  // Tracked pose - Base link
  std::mutex                 m_tracked_pose_mutex;
  geometry_msgs::PoseStamped m_tracked_pose;
  ros::Publisher             m_tracked_pose_pub;

  // Tracked pose - World frame
  std::mutex                 m_world_tracked_pose_mutex;
  geometry_msgs::PoseStamped m_world_tracked_pose;
  ros::Publisher             m_world_tracked_pub;

  // Filtered tracked pose - World frame
  std::mutex                 m_world_filtered_pose_mutex;
  geometry_msgs::PoseStamped m_world_filtered_pose;
  ros::Publisher             m_world_filtered_pub;
};


}// namespace mood_ros

void mood_ros::DetectionManager::onInit()
{
  ROS_INFO("[DetectionManager] onInit()!");

  // Load Private parameters
  auto& nh_private = getMTPrivateNodeHandle();
  auto& nh         = getMTNodeHandle();

  ros::Time::waitForValid();

  // Load manager parameters
  param_util::getParamOrThrow(nh_private, "mood/detector", m_detector_name);
  param_util::getParamOrThrow(nh_private, "mood/synchronizer", m_sync_name);
  param_util::getParamOrThrow(nh_private, "mood/world_frame", m_world_frame);

  // Load detector plugin
  m_detector_loader_ptr =
    std::make_unique<detector_loader_t>("mood_ros", "mood_base::detector_interface");
  m_detector_ptr = m_detector_loader_ptr->createUniqueInstance(m_detector_name);

  // Initialize the detector
  auto init_success = m_detector_ptr->initialize(nh, nh_private);
  if (!init_success) {
    ROS_ERROR("[DetectionManager] Detector initialization unsucessful!");
    ros::shutdown();
  }

  // Load synchronization plugin
  m_syncrhonizer_loader_ptr =
    std::make_unique<sync_loader_t>("mood_ros", "mood_base::msg_sync_interface");
  m_synchronizer_ptr = m_syncrhonizer_loader_ptr->createUniqueInstance(m_sync_name);

  // Initialize the synchronizer plugin
  auto sync_init = m_synchronizer_ptr->initialize(nh);
  if (!sync_init) {
    ROS_ERROR("[DetectionManager] Synchronizer initialization unsucessful!");
    ros::shutdown();
  }
  m_new_pose_calculated = false;
  m_synchronizer_ptr->register_callback(
    std::bind(&DetectionManager::synchronizer_cb, this, std::placeholders::_1));

  image_transport::ImageTransport it(nh);
  m_labeled_img_pub    = it.advertise("mood/labeled_image", 1);
  m_odom_sub           = nh.subscribe("odometry", 1, &DetectionManager::odom_cb, this);
  m_detected_poses_pub = nh.advertise<geometry_msgs::PoseArray>("mood/pose_array", 1);
  m_tracked_pose_pub   = nh.advertise<geometry_msgs::PoseStamped>("mood/tracked_pose", 1);
  m_world_tracked_pub =
    nh.advertise<geometry_msgs::PoseStamped>("mood/world_tracked_pose", 1);
  m_world_filtered_pub =
    nh.advertise<geometry_msgs::PoseStamped>("mood/world_filtered_pose", 1);
  m_switching_srv = nh.advertiseService(
    "mood/switch_tracking", &DetectionManager::switching_srv_cb, this);

  // Initialize Kalman Filters
  m_lkf_x = std::make_unique<ConstantVelocityLKF>("lkf_x", nh_private);
  m_lkf_y = std::make_unique<ConstantVelocityLKF>("lkf_y", nh_private);
  m_lkf_z = std::make_unique<ConstantVelocityLKF>("lkf_z", nh_private);
  m_kalman_timer =
    nh.createTimer(ros::Duration(m_kalman_dt), &DetectionManager::kalman_event, this);

  // Set nodelet as initialized
  ROS_INFO("[DetectionManager] Initialized successfully");
  m_initialized = true;
}

bool mood_ros::DetectionManager::switching_srv_cb(std_srvs::TriggerRequest&  req,
                                                  std_srvs::TriggerResponse& resp)
{
  std::scoped_lock lock(m_tracker_mutex);
  if (pose_tracker.getMap().size() < 2) {
    resp.success = false;
    resp.message = "Tracking map has less than 2 entries";
    return true;
  }

  auto [success, message] = pose_tracker.nextTrackingID();
  resp.success            = success;
  resp.message            = message;
  return true;
}

void mood_ros::DetectionManager::synchronizer_cb(const sensor_comm::sensor_info& info)
{
  if (!m_initialized) { return; }

  geometry_msgs::PoseArray detected_pose_array;
  {
    std::scoped_lock lock(m_detector_mutex);
    // First, update the detector
    auto resp = m_detector_ptr->update(info);
    if (!resp.status) {
      ROS_ERROR_STREAM_THROTTLE(
        2.0, "[DetectionManager] Detector failed with message: " << resp.response);
    };

    // Detection is successful, publish topics
    detected_pose_array = m_detector_ptr->get_object_poses();
    m_labeled_img_pub.publish(m_detector_ptr->get_labeled_image());
    m_detected_poses_pub.publish(detected_pose_array);
  }

  bool                tracker_status;
  geometry_msgs::Pose tracked_pose;
  {
    std::scoped_lock lock(m_tracker_mutex);
    // Update tracker
    std::tie(tracker_status, tracked_pose) =
      pose_tracker.updateAllCentroids(detected_pose_array.poses);
  }

  if (!tracker_status) {
    ROS_FATAL_THROTTLE(1.0, "[DetectionManager] Tracking failed");
    // TODO(lmark): Maybe do something if tracking fails :-)
  }

  // Publish tracked pose
  {
    std::scoped_lock lock(m_tracked_pose_mutex);
    m_new_pose_calculated = tracker_status;
    m_tracked_pose.header = detected_pose_array.header;
    m_tracked_pose.pose   = tracked_pose;
    m_tracked_pose_pub.publish(m_tracked_pose);
  }
}

void mood_ros::DetectionManager::odom_cb(const nav_msgs::OdometryConstPtr& msg)
{
  if (!m_initialized) { return; }

  if (!m_new_pose_calculated) {
    std::scoped_lock lock(m_world_tracked_pose_mutex);
    m_world_tracked_pub.publish(m_world_tracked_pose);
    return;
  }


  auto           vehicle_q = tf2::Quaternion{ msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z,
                                    msg->pose.pose.orientation.w };
  tf2::Matrix3x3 vehicle_transformation;
  vehicle_transformation.setRotation(vehicle_q);

  // Get tracked pose
  geometry_msgs::PoseStamped tracked_pose;
  {
    std::scoped_lock lock(m_tracked_pose_mutex);
    tracked_pose = m_tracked_pose;
  }

  // Get tracked world position
  auto world_tracked_position = vehicle_transformation
                                * tf2::Vector3{ tracked_pose.pose.position.x,
                                                tracked_pose.pose.position.y,
                                                tracked_pose.pose.position.z };

  tf2::Matrix3x3 tracked_transformation;
  tracked_transformation.setRotation(tf2::Quaternion{ tracked_pose.pose.orientation.x,
                                                      tracked_pose.pose.orientation.y,
                                                      tracked_pose.pose.orientation.z,
                                                      tracked_pose.pose.orientation.w });
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
  {
    std::scoped_lock lock(m_world_tracked_pose_mutex);
    m_world_tracked_pose.header.stamp       = ros::Time::now();
    m_world_tracked_pose.header.frame_id    = m_world_frame;
    m_world_tracked_pose.pose.orientation.x = world_tracked_q.x();
    m_world_tracked_pose.pose.orientation.y = world_tracked_q.y();
    m_world_tracked_pose.pose.orientation.z = world_tracked_q.z();
    m_world_tracked_pose.pose.orientation.w = world_tracked_q.w();
    m_world_tracked_pose.pose.position.x =
      world_tracked_position.x() + msg->pose.pose.position.x;
    m_world_tracked_pose.pose.position.y =
      world_tracked_position.y() + msg->pose.pose.position.y;
    m_world_tracked_pose.pose.position.z =
      world_tracked_position.z() + msg->pose.pose.position.z;
    m_world_tracked_pub.publish(m_world_tracked_pose);
  }
}

void mood_ros::DetectionManager::kalman_event(const ros::TimerEvent& e)
{
  geometry_msgs::PoseStamped world_tracked_pose;
  {
    std::scoped_lock lock(m_world_tracked_pose_mutex);
    world_tracked_pose = m_world_tracked_pose;
  }

  // Do the Kalman filtering
  m_lkf_x->estimateState(
    m_kalman_dt, { world_tracked_pose.pose.position.x }, m_new_pose_calculated);
  m_lkf_y->estimateState(
    m_kalman_dt, { world_tracked_pose.pose.position.y }, m_new_pose_calculated);
  m_lkf_z->estimateState(
    m_kalman_dt, { world_tracked_pose.pose.position.z }, m_new_pose_calculated);
  // lkf_heading.estimateState(kalman_dt,
  //  { ros_convert::calculateYaw(world_tracked_pose.pose.orientation) },
  //  new_pose_calculated);

  // Publish filtered pose&
  {
    std::scoped_lock lock(m_world_filtered_pose_mutex);
    m_world_filtered_pose.header.frame_id  = world_tracked_pose.header.frame_id;
    m_world_filtered_pose.header.stamp     = ros::Time::now();
    m_world_filtered_pose.pose.position.x  = m_lkf_x->getState()[0];
    m_world_filtered_pose.pose.position.y  = m_lkf_y->getState()[0];
    m_world_filtered_pose.pose.position.z  = m_lkf_z->getState()[0];
    m_world_filtered_pose.pose.orientation = world_tracked_pose.pose.orientation;
    // ros_convert::calculate_quaternion(lkf_heading.getState()[0]);
    m_world_filtered_pub.publish(m_world_filtered_pose);
  }

  if (m_new_pose_calculated) { m_new_pose_calculated = false; }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_ros::DetectionManager, nodelet::Nodelet)