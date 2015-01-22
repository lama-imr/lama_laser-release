/**
 * Large Map 
 * Laser based memoryless (reactive) navigating jockey
 *
 */

#include <string>

#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <nj_laser/jockey.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_jockey");
  ros::NodeHandle private_nh("~");
  
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  /* Minimal frontier width */
  if (!private_nh.hasParam("frontier_width"))
  {
    ROS_ERROR("Parameter frontier_width not set, exiting.");
    return 1;
  }
  double frontier_width;
  private_nh.param<double>("frontier_width", frontier_width, 0.0);

  std::string navigating_jockey_name;
  private_nh.param<std::string>("navigating_jockey_server_name",
      navigating_jockey_name, ros::this_node::getName() + "_server");

  nj_laser::Jockey jockey(navigating_jockey_name, frontier_width);

  ROS_INFO("%s started (with server %s)", ros::this_node::getName().c_str(), jockey.getName().c_str());
  ros::spin();
  return 0;
}

