/*
 * Localizing jockey based on LaserScan associated with
 * absolute heading.
 *
 * The role of this jockey is to get the dissimilarity of the LaserScan
 * descriptors of all vertices with the current LaserScan.
 * The action is done when the dissimilarities are computed.
 * Implemented actions:
 * - GET_VERTEX_DESCRIPTOR: return the LaserScan and the computed Crossing
 * - GET_DISSIMILARITY: return the dissimilarity based on LaserScan
 *
 * Interaction with the map (created by this jockey through lj_laser/jockey.h):
 * - Getter/Setter: VectorLaserScan, jockey_name + "_laser"
 * - Setter: Crossing, jockey_name + "_crossing"
 *
 * Interaction with the map (created by other jockeys):
 * - none
 *
 * Subscribers (other than map-related):
 * - sensor_msg/LaserScan, "~/base_scan", 360-deg laser-scan.
 * - geometry_msgs/Pose, "~/pose", heading is read from it.
 * - nav_msgs/Odometry, "~/odom", heading is read from it.
 *
 * Publishers (other than map-related):
 * - 
 * Services used (other than map-related):
 * - polygon_matcher::PolygonDisimilarity, "~/dissimilarity_server"
 */

#ifndef _LJ_LASER_HEADING_JOCKEY_H_
#define _LJ_LASER_HEADING_JOCKEY_H_

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <lj_laser/jockey.h>

namespace lj_laser_heading {

class Jockey : public lj_laser::Jockey
{
  public:

    Jockey(std::string name, const double frontier_width, const double max_frontier_angle=0.785);

  private:

    void initMapLaserInterface();
    void initMapCrossingInterface();
    void handlePose(const geometry_msgs::PoseConstPtr& msg);
    void handleOdom(const nav_msgs::OdometryConstPtr& msg);

    void getData();
    void rotateScan();
    void handleLaser(const sensor_msgs::LaserScanConstPtr& msg);

    // Reception and storage of Odometry and Crossing.
    ros::Subscriber poseHandler_;
    ros::Subscriber odomHandler_;
    ros::Time heading_reception_time_;
    double heading_;  //!> Heading information read from Pose or Odometry.

    // Hard-coded parameters.
    const static ros::Duration max_data_time_delta_;  //!> Max time interval between reception of scan_ and heading_.
};

} // namespace lj_laser_heading

#endif // _LJ_LASER_HEADING_JOCKEY_H_
