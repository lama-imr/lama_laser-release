#ifndef NJ_OA_LASER_TWIST_HANDLER_H
#define NJ_OA_LASER_TWIST_HANDLER_H

#include <cmath>
#include <string>

#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

namespace nj_oa_laser {

class TwistHandler
{
  public :
    
    TwistHandler(const double robot_radius);

    geometry_msgs::Twist getTwist(const sensor_msgs::LaserScan& scan);

    double robot_radius;  //!> (m), robot radius.
    double min_distance;  //!> (m), if an obstacle is closer (y-direction) than this, turn and don't go forward.
    double long_distance;  //!> (m), if no obstacle within this distance, go straight.
    double turnrate_collide;  //!> (rad/s), turn rate when obstacle closer than min_distance.
    double max_vel;  //!> (m/s), linear velocity without obstacle.
    double vel_close_obstacle;  //!> (m/s), linear velocity if obstacle between min_distance and long_distance.
    double turnrate_factor;  //!> (rad.m^-1.s^-1, > 0), if obstacle closer than long_distance,
                              //!> turnrate = -turnrate_factor * mean(lateral_position_of_obstacle).
};

} // namespace nj_oa_laser

#endif // NJ_OA_LASER_TWIST_HANDLER_H

