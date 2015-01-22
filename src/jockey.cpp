#include <lj_laser_heading/jockey.h>

namespace lj_laser_heading {

// Maximum time interval between reception of LaserScan and heading (either as
// Pose or Odoometry), whichever comes first. The jockey will reject all data
// until one LaserScan message and one Pose/Odometry message come shortly one
// after another.
// TODO: write as a parameter.
const ros::Duration Jockey::max_data_time_delta_ = ros::Duration(0.050);

Jockey::Jockey(std::string name, const double frontier_width, const double max_frontier_angle) :
  lj_laser::Jockey(name, frontier_width, max_frontier_angle),
  heading_reception_time_(ros::Time(0))
{
}

/* Start the subscribers, wait for a LaserScan and a Pose, and exit upon reception.
 */
void Jockey::getData()
{
  data_received_ = false;
  laserHandler_ = nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);
  poseHandler_ = nh_.subscribe<geometry_msgs::Pose>("pose", 1, &Jockey::handlePose, this);
  odomHandler_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &Jockey::handleOdom, this);

  ros::Rate r(100);
  while (ros::ok())
  {
    if (data_received_)
    {
      // Stop the subscribers (may be superfluous).
      laserHandler_.shutdown();
      poseHandler_.shutdown();
      odomHandler_.shutdown();
      data_received_ = false;
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
  rotateScan();
}

/* Change angle_min and angle_max of scan_, so that they represent absolute angles.
 *
 * This as for consequence that angle_min or angle_max can be outside of ]-pi,pi].
 */
void Jockey::rotateScan()
{
  scan_.angle_min -= heading_;
  scan_.angle_max -= heading_;
}

/* Receive a LaserScan message and store it.
 */
void Jockey::handleLaser(const sensor_msgs::LaserScanConstPtr& msg)
{
  ROS_DEBUG("%s: laser arrived with %zu beams", jockey_name_.c_str(), msg->ranges.size());

  scan_ = *msg;
  scan_reception_time_ = msg->header.stamp;
  data_received_ = (scan_reception_time_ - heading_reception_time_) < max_data_time_delta_;
}

/* Return the angle from a quaternion representing a rotation around the z-axis
 *
 * The quaternion in ROS is q = (w, x, y, z), so that
 * q = (cos(a/2), ux * sin(a/2), uy * sin(a/2), uz * sin(a/2)),
 *   where a is the rotation angle and (ux, uy, uz) is the unit vector of the
 *   rotation axis.
 * For a rotation around z, we have q = (cos(a/2), 0, 0, sin(a/2)). Thus
 * a = 2 * atan2(z, w).
 */
double angleFromQuaternion(const geometry_msgs::Quaternion& q)
{
  if (std::fabs(q.x) > 1e-5 || std::fabs(q.y) > 1e-5)
  {
    ROS_WARN("Laser frame rotation is not around the z-axis, just pretending it is");
  }
  return 2 * std::atan2(q.z, q.w);
}

/* Receive a Pose message and store the heading information.
 */
void Jockey::handlePose(const geometry_msgs::PoseConstPtr& msg)
{
  ROS_DEBUG("%s: pose arrived", jockey_name_.c_str());

  heading_ = angleFromQuaternion(msg->orientation);
  heading_reception_time_ = ros::Time::now();
  data_received_ = (heading_reception_time_ - scan_reception_time_) < max_data_time_delta_;
}

/* Receive a Odometry message and store the heading information.
 */
void Jockey::handleOdom(const nav_msgs::OdometryConstPtr& msg)
{
  ROS_DEBUG("%s: odom arrived", jockey_name_.c_str());

  // TODO: make writing heading_ and heading_reception_time_ thread safe.
  heading_ = angleFromQuaternion(msg->pose.pose.orientation);
  heading_reception_time_ = msg->header.stamp;
  data_received_ = (heading_reception_time_ - scan_reception_time_) < max_data_time_delta_;
}

} // namespace lj_laser_heading
