/*
 * Localizing Jockey based on LaserScan.
 *
 * The role of this jockey is to get the dissimilarity of the LaserScan
 * descriptors of all vertices with the current LaserScan.
 * The action is done when the dissimilarities are computed.
 * Implemented actions:
 * - GET_VERTEX_DESCRIPTOR: return the ids of the LaserScan descriptor and the
 *     computed Crossing descriptor
 * - GET_SIMILARITY: return the dissimilarity based on LaserScan
 *
 * Interaction with the map (created by this jockey):
 * - [Getter][/][Setter], message type, interface default name
 * - Getter/Setter: VectorLaserScan, jockey_name + "_laser"
 * - Setter: Crossing, jockey_name + "_crossing"
 *
 * Interaction with the map (created by other jockeys):
 * - [Getter][/][Setter], message type, interface default name
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - sensor_msgs::LaserScan, "~/base_scan", 360-deg laser-scan.
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 *
 * Services used (other than map-related):
 * - service type, server default name, description
 * - polygon_matcher::PolygonDissimilarity, "~/dissimilarity_server", used to
 *    compare all known places with the current place
 *
 * Parameters:
 * - name, type, default name, description
 * - laser_interface_name, String, jockey_name + "_laser", name of the map interface for 360-degree laser-scan.
 * - crossing_interface_name, String, jockey_name + "_crossing", name of the map interface for crossing.
 * - dissimilarity_server_name, String, "dissimilarity_server", name of the dissimilarity server.
 */

#ifndef _LJ_LASER_JOCKEY_H_
#define _LJ_LASER_JOCKEY_H_

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_common/polygon.h>  // for scanToPolygon
#include <lama_interfaces/ActOnMap.h>
#include <lama_interfaces/AddInterface.h>
#include <lama_interfaces/GetVectorLaserScan.h>
#include <lama_interfaces/SetVectorLaserScan.h>
#include <lama_msgs/DescriptorLink.h>
#include <lama_msgs/SetCrossing.h>
#include <lama_jockeys/localizing_jockey.h>
#include <polygon_matcher/PolygonDissimilarity.h>

#include <crossing_detector/laser_crossing_detector.h>

namespace lj_laser {

class Jockey : public lama_jockeys::LocalizingJockey
{
  public:

    Jockey(std::string name, const double frontier_width, const double max_frontier_angle=0.785);

    virtual void onGetVertexDescriptor();
    virtual void onGetEdgesDescriptors();
    virtual void onLocalizeInVertex();
    virtual void onLocalizeEdge();
    virtual void onGetDissimilarity();
    // virtual void onInterrupt();
    // virtual void onContinue();

    void setDissimilarityServerName(std::string name) {dissimilarity_server_name_ = name;}

  protected:

    bool data_received_;

    // Reception and storage of LaserScan.
    ros::Subscriber laserHandler_;
    ros::Time scan_reception_time_;
    sensor_msgs::LaserScan scan_;

    // Reception and Sending of LaserScan and Crossing descriptors.
    std::string laser_interface_name_;
    ros::ServiceClient laser_descriptor_getter_;
    ros::ServiceClient laser_descriptor_setter_;
    std::string crossing_interface_name_;
    ros::ServiceClient crossing_descriptor_setter_;

    // Dissimilarity server.
    std::string dissimilarity_server_name_;
    ros::ServiceClient dissimilarity_server_;

    crossing_detector::LaserCrossingDetector crossing_detector_;

  private:

    bool initMapLaserInterface();
    bool initMapCrossingInterface();
    void getData();
    void handleLaser(const sensor_msgs::LaserScanConstPtr& msg);

    lama_msgs::DescriptorLink laserDescriptorLink(const int32_t id);
    lama_msgs::DescriptorLink crossingDescriptorLink(const int32_t id);
};


} // namespace lj_laser

#endif // _LJ_LASER_JOCKEY_H_
