/* Action server for the memory-less navigating jockey based on LaserScan.
 *
 * The role of this jockey is to travel to the next crossing.
 * Implemented actions:
 * - TRAVERSE: will start navigating to the next crossing (place with at least
 *   three frontiers) and will succeed when the crossing center is reached.
 * - STOP: will stop
 * - INTERRUPT: same as CONTINUE
 * - CONTINUE: same as TRAVERSE
 *
 * Interaction with the map:
 * - Getter: none
 * - Setter: none.
 *
 * Subscribers (other than map-related):
 * - sensor_msg/LaserScan, "~/base_scan", 360-deg laser-scan.
 *
 * Publishers (other than map-related):
 * - visualization_msgs/Marker, "~crossing_marker", a sphere at the crossing center.
 * - visualization_msgs/Marker, "~exits_marker", lines from crossing center towards exits.
 * - geometry_msgs/Twist, "~cmd_vel", set velocity.
 * - sensor_msgs/PointCloud, "~place_profile_cloud", profile of the surroundings.
 * - lama_msgs/Crossing, "~crossing", computed Crossing
 *
 * Parameters:
 * - ~max_frontier_distance, Float, NO_DEFAULT, points farther than this are cut and
 *     frontier may exist.
 * - ~robot_radius, Float, frontier_width/2, robot radius (frontier_width is
 *     a constructor parameter)
*/

#ifndef _NJ_LASER_JOCKEY_H_
#define _NJ_LASER_JOCKEY_H_

#include <sensor_msgs/LaserScan.h>

#include <crossing_detector/laser_crossing_detector.h>
#include <goto_crossing/crossing_goer.h>
#include <lama_common/crossing_visualization.h>
#include <lama_jockeys/navigating_jockey.h>
#include <nj_oa_laser/twist_handler.h>

#include <nj_laser/visualization.h>

namespace nj_laser {

class Jockey : public lama_jockeys::NavigatingJockey
{
  public:

    Jockey(const std::string& name, const double frontier_width);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

    void handleLaser(const sensor_msgs::LaserScanConstPtr& msg);

  private:

    // Subscribers and publishers.
    ros::Subscriber laserHandler_;
    ros::Publisher pub_crossing_marker_;
    ros::Publisher pub_exits_marker_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_place_profile_;
    ros::Publisher pub_crossing_;

    // Parameters shown outside.
    double max_frontier_dist_;

    // Internals.
    bool has_crossing_;
    sensor_msgs::LaserScan scan_;  //!> Last received laser scan.
    lama_msgs::Crossing crossing_;  //!> Crossing descriptor from LaserScan.
    crossing_detector::LaserCrossingDetector crossing_detector_;
    goto_crossing::CrossingGoer crossing_goer_;
    nj_oa_laser::TwistHandler obstacle_avoider_;  //!> Twist computation for obstacle avoidance.
};

} // namespace nj_laser

#endif // _NJ_LASER_JOCKEY_H_

