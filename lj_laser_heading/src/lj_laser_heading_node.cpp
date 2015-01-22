#include <string>

#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <lj_laser_heading/jockey.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizing_jockey");
  ros::NodeHandle n("~");
  
  // Change log level.
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  std::string localizing_jockey_server;
  std::string default_server_name = ros::this_node::getName();
  default_server_name += "_server";
	n.param<std::string>("localizing_jockey_server_name", localizing_jockey_server, default_server_name);

  /* Minimal frontier width */
  if (!n.hasParam("frontier_width"))
  {
    ROS_ERROR("Parameter frontier_width not set, exiting.");
    return 1;
  }
  double frontier_width;
  n.param<double>("frontier_width", frontier_width, 0.0);

  lj_laser_heading::Jockey jockey(localizing_jockey_server, frontier_width);

  ROS_INFO("%s started (with action server %s)", ros::this_node::getName().c_str(), localizing_jockey_server.c_str());
  ros::spin();
  return 0;
}

