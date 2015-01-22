/*
 * Obstacle avoidance with LaserScan.
 *
 * Drive the robot while avoiding obstacles:
 * - onTraverse and onContinue: go more or less forward depending
 *     on obstacle position. The action never stops by itself.
 *
 * Interaction with the map (created by this jockey):
 * - none.
 *
 * Interaction with the map (created by other jockeys):
 * - none.
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - sensor_msgs/LaserScan, "~base_scan", laser-scan at the front of the
 *   robot.
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 * - geometry_msgs/Twist, "~cmd_vel", set velocity.
 *
 * Services used (other than map-related):
 * - none.
 *
 * Parameters:
 * - ~robot_radius, Float, NO_DEFAULT, robot radius (m).
 * - ~min_distance, Float, 2 * robot_width, if an obstacle is closer than this,
 *     turn and don't go forward (m).
 * - ~long_distance, Float, 5 * robot_width, if no obstacle within this
 *     distance, go straight (m).
 * - ~turnrate_collide, Float, 0.4, turn rate when obstacle closer than
 *     min_distance_ (rad/s).
 * - ~max_vel, Float, 1.0, linear velocity without obstacle (m/s).
 * - ~vel_close_obstacle, Float, 0.5, linear velocity if obstacle between
 *     min_distance and long_distance (m/s).
 * - ~turnrate_factor, Float, 0.9, if obstacle closer than long_distance
 *     turnrate = -turnrate_factor_ * mean(lateral_position_of_obstacle)
 *     (rad.m^-1.s^-1).
 */

#ifndef NJ_OA_LASER_JOCKEY_H
#define NJ_OA_LASER_JOCKEY_H

#include <cmath>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <lama_jockeys/navigating_jockey.h>

#include <nj_oa_laser/twist_handler.h>

namespace nj_oa_laser {

class Jockey : public lama_jockeys::NavigatingJockey
{
  public :

    Jockey(const std::string& name, const double robot_radius);

    void initTwistHandlerParam(TwistHandler& twist_handler);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

  protected:

    // Subscribers and publishers.
    ros::Publisher pub_twist_;

  private :

    void handleLaser(const sensor_msgs::LaserScanConstPtr& msg);

    // Internals.
    TwistHandler twist_handler_;  //!> To compute the twist from a LaserScan.

};

} // namespace nj_oa_laser

#endif // NJ_OA_LASER_JOCKEY_H
