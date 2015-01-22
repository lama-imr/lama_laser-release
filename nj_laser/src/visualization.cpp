/* Functions for visualization of crossings in rviz */

#include <nj_laser/visualization.h>

namespace nj_laser {

/* Return the marker for the visualization of the crossing center
 */
visualization_msgs::Marker crossingMarker(const std::string& frame_id, const double x, const double y, const double radius)
{
	visualization_msgs::Marker m;
	m.header.frame_id = frame_id;
	m.ns = "crossing_center";
	m.type = visualization_msgs::Marker::SPHERE;
	m.pose.position.x = x;
	m.pose.position.y = y;
	m.pose.position.z = 0;
	m.pose.orientation.w = 1.0;
	m.scale.x = radius;
	m.scale.y = radius;
	m.scale.z = 1;
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.a = 0.5;
	return m;
}

/* Return the marker for the visualization of crossing exits
 */
visualization_msgs::Marker exitsMarker(const std::string& frame_id, const double x, const double y, const std::vector<double>& angles, const double length)
{
	visualization_msgs::Marker m;
	m.header.frame_id = frame_id;
	m.ns = "road_direction";
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.pose.orientation.w = 1.0;
	m.scale.x = 0.1;
	m.color.r = 0.0;
	m.color.g = 0.0;
	m.color.b = 1.0;
	m.color.a = 0.5;
  for(size_t i = 0; i < angles.size(); ++i)
	{
		geometry_msgs::Point p;
    p.x = x;
    p.y = y;
		m.points.push_back(p);
		p.x = x + length * cos(angles[i]);
		p.y = y + length * sin(angles[i]);
		m.points.push_back(p);
	}
	return m;
}

} // namespace nj_laser

