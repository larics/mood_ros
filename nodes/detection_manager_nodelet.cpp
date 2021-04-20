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
};
}// namespace mood_ros

void mood_ros::DetectionManager::onInit() {}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mood_ros::DetectionManager, nodelet::Nodelet)