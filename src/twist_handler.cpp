#include <nj_oa_laser/twist_handler.h>

namespace nj_oa_laser {

TwistHandler::TwistHandler(const double robot_radius) :
  robot_radius(robot_radius),
  min_distance(1.5 * robot_radius),
  long_distance(5 * robot_radius),
  turnrate_collide(0.4),
  max_vel(1.0),
  vel_close_obstacle(0.5),
  turnrate_factor(0.9)
{
}

/* Return the twist to avoid obstacles
 *
 * The algorithm considers only beams in [-pi, pi]. The robot will turn in the direction
 * where the mean obstacle distance is larger.
 */
geometry_msgs::Twist TwistHandler::getTwist(const sensor_msgs::LaserScan& scan)
{
   bool collide = false;
   bool go_straight = true;
   double sum_y = 0;
   unsigned int count_y = 0;
   double sum_y_colliding = 0;
   const double long_radius = 1.5 * robot_radius;

   double x;
   double y;
   for (unsigned int i = 0; i < scan.ranges.size(); ++i)
   {
     const double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment);
     if ((angle < -M_PI_2) || (angle > M_PI_2))
     {
       // Do no consider a beam directed backward.
       continue;
     }

     x = scan.ranges[i] * std::cos(angle);
     y = scan.ranges[i] * std::sin(angle);

     if ((x < min_distance)  && (-robot_radius < y) && (y < robot_radius))
     {
       collide = true;
       sum_y_colliding += x;   
     } 

     if ((x < long_distance)  && (-long_radius < y) && (y < robot_radius))
     {
       go_straight = false;
     }
     sum_y += y;
     count_y++;  
   }

   geometry_msgs::Twist twist;
   if (collide)
   { 
     twist.linear.x = 0;
     if (sum_y_colliding < 0)
     { 
       twist.angular.z = -turnrate_collide;
     }
     else
     {
       twist.angular.z = turnrate_collide;
     }      
   }
   else if (go_straight)
   {
     twist.linear.x = max_vel;
     twist.angular.z = 0;
   }
   else
   {
     twist.linear.x = vel_close_obstacle;
     twist.angular.z = -turnrate_factor * sum_y / ((double) count_y);
   }

   return twist;
}

} // namespace nj_laser

