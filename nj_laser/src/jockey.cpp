#include <nj_laser/jockey.h>

namespace nj_laser {

Jockey::Jockey(const std::string& name, const double frontier_width) :
  lama_jockeys::NavigatingJockey(name),
  has_crossing_(false),
  crossing_detector_(frontier_width),
  obstacle_avoider_(frontier_width / 2)
{
  private_nh_.getParam("max_frontier_distance", max_frontier_dist_);
  double robot_radius;
  if (private_nh_.getParam("robot_radius", robot_radius))
  {
    obstacle_avoider_.robot_radius = robot_radius;
  }

  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_crossing_marker_ = private_nh_.advertise<visualization_msgs::Marker>("crossing_marker", 50, true);
  pub_exits_marker_ = private_nh_.advertise<visualization_msgs::Marker> ("exits_marker", 50, true);
  pub_place_profile_ = private_nh_.advertise<sensor_msgs::PointCloud>("place_profile_cloud", 50, true);
  pub_crossing_ = private_nh_.advertise<lama_msgs::Crossing>("crossing", 50, true);
}

void Jockey::onTraverse()
{
  ROS_DEBUG("%s: Received action TRAVERSE or CONTINUE", ros::this_node::getName().c_str());
  crossing_goer_.resetIntegrals();

  laserHandler_ = private_nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);
  
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

    if (has_crossing_)
    {
      geometry_msgs::Twist twist;
      if (crossing_.frontiers.size() < 3)
      {
        twist = obstacle_avoider_.getTwist(scan_);
        pub_twist_.publish(twist);
      }
      else
      {
        const bool goal_reached = crossing_goer_.goto_crossing(crossing_, twist);
        pub_twist_.publish(twist);

        if (goal_reached)
        {
          laserHandler_.shutdown();
          result_.final_state = result_.DONE;
          result_.completion_time = getCompletionDuration();
          server_.setSucceeded(result_);
          break;
        }
      }
      ROS_DEBUG("%s: twist (%.3f, %.3f)", ros::this_node::getName().c_str(), twist.linear.x, twist.angular.z);
      has_crossing_ = false;
    }
    ros::spinOnce();
    r.sleep();
  }
  ROS_DEBUG("Exiting onTraverse");
}

void Jockey::onStop()
{
  ROS_DEBUG("%s: Received action STOP or INTERRUPT", ros::this_node::getName().c_str());
  laserHandler_.shutdown();
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

void Jockey::handleLaser(const sensor_msgs::LaserScanConstPtr& msg)
{
  ROS_DEBUG("%s: laser arrived with %zu beams", ros::this_node::getName().c_str(), msg->ranges.size());

  scan_ = *msg;
  crossing_detector_.setMaxFrontierDistance(max_frontier_dist_);
  crossing_ = crossing_detector_.crossingDescriptor(scan_);
  ROS_DEBUG("%s: crossing (%.3f, %.3f, %.3f), number of exits: %zu", ros::this_node::getName().c_str(),
        crossing_.center.x, crossing_.center.y, crossing_.radius, crossing_.frontiers.size());

  has_crossing_ = true;

  // Visualization: a sphere at detected crossing center
  if (pub_crossing_marker_.getNumSubscribers())
  {
    visualization_msgs::Marker m = lama_common::getCrossingCenterMarker(msg->header.frame_id, crossing_);
    pub_crossing_marker_.publish(m);
  }

  // Visualization: a line at each detected road
  if (pub_exits_marker_.getNumSubscribers())
  {
    visualization_msgs::Marker m = lama_common::getFrontiersMarker(msg->header.frame_id, crossing_);
    pub_exits_marker_.publish(m);
  }

  // PlaceProfile visualization message.
  if (pub_place_profile_.getNumSubscribers())
  {
    sensor_msgs::PointCloud cloud = lama_common::placeProfileToPointCloud(crossing_detector_.getPlaceProfile());
    pub_place_profile_.publish(cloud);
  }

  pub_crossing_.publish(crossing_);
}

} // namespace nj_laser

