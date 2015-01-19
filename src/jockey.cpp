#include <nj_oa_laser/jockey.h>

namespace nj_oa_laser {

Jockey::Jockey(const std::string& name, const double robot_radius) :
  lama_jockeys::NavigatingJockey(name),
  twist_handler_(robot_radius)
{
  initTwistHandlerParam(twist_handler_);
}

void Jockey::initTwistHandlerParam(TwistHandler& twist_handler)
{
  double robot_radius_;
  if (private_nh_.getParam("robot_radius", robot_radius_))
  {
    twist_handler.robot_radius = robot_radius_;
  }

  double min_distance;
  if (private_nh_.getParam("min_distance", min_distance))
  {
    twist_handler.min_distance = min_distance;
  }

  double long_distance;
  if (private_nh_.getParam("long_distance", long_distance))
  {
    twist_handler.long_distance = long_distance;
  }
  
  double turnrate_collide;
  if (private_nh_.getParam("turnrate_collide", turnrate_collide))
  {
    twist_handler.turnrate_collide = turnrate_collide;
  }

  double max_vel;
  if (private_nh_.getParam("max_vel", max_vel))
  {
    twist_handler.max_vel = max_vel;
  }

  double vel_close_obstacle;
  if (private_nh_.getParam("vel_close_obstacle", vel_close_obstacle))
  {
    twist_handler.vel_close_obstacle = vel_close_obstacle;
  }

  double turnrate_factor;
  if (private_nh_.getParam("turnrate_factor", turnrate_factor))
  {
    twist_handler.turnrate_factor = turnrate_factor;
  }
}

void Jockey::onTraverse()
{
  ROS_DEBUG("%s: Received action TRAVERSE or CONTINUE", ros::this_node::getName().c_str());

  ros::Subscriber laser_handler = private_nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);
  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  ros::Rate r(100);
  while (ros::ok())
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    ros::spinOnce();
    r.sleep();
  }

  pub_twist_.shutdown();
}

void Jockey::onStop()
{
  ROS_DEBUG("%s: Received action STOP or INTERRUPT", ros::this_node::getName().c_str());
  result_.final_state = lama_jockeys::NavigateResult::DONE;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onInterrupt()
{
  ROS_DEBUG("%s: Received action INTERRUPT", ros::this_node::getName().c_str());
  onStop();
}

void Jockey::onContinue()
{
  ROS_DEBUG("%s: Received action CONTINUE", ros::this_node::getName().c_str());
  onTraverse();
}

/* Callback for the LaserScan topic
 */
void Jockey::handleLaser(const sensor_msgs::LaserScanConstPtr& msg)
{
  geometry_msgs::Twist twist = twist_handler_.getTwist(*msg);
  pub_twist_.publish(twist);
}

} // namespace nj_oa_laser
