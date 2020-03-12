#ifndef MARKER_VIZ__MARKER_VIZ_HPP_
#define MARKER_VIZ__MARKER_VIZ_HPP_

#include <chrono>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "mocap4ros_msgs/msg/marker.hpp"
#include "mocap4ros_msgs/msg/markers.hpp"
#include "mocap4ros_msgs/msg/marker_with_id.hpp"
#include "mocap4ros_msgs/msg/markers_with_id.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "marker_viz_srvs/srv/set_marker_color.hpp"
#include "marker_viz_srvs/srv/reset_marker_color.hpp"

typedef visualization_msgs::msg::Marker VizMarker;
typedef mocap4ros_msgs::msg::Marker MocapMarker;
typedef mocap4ros_msgs::msg::Markers MocapMarkers;
typedef mocap4ros_msgs::msg::Markers::SharedPtr MocapMarkersSharedPtr;
typedef mocap4ros_msgs::msg::MarkerWithId MocapMarkerWithId;
typedef mocap4ros_msgs::msg::MarkersWithId MocapMarkersWithId;
typedef mocap4ros_msgs::msg::MarkersWithId::SharedPtr MocapMarkersWithIdSharedPtr;

typedef marker_viz_srvs::srv::SetMarkerColor SetMarkerColor;
typedef marker_viz_srvs::srv::ResetMarkerColor ResetMarkerColor;
typedef std::shared_ptr<marker_viz_srvs::srv::SetMarkerColor::Request> SetRequest;
typedef std::shared_ptr<marker_viz_srvs::srv::SetMarkerColor::Response> SetResponse;
typedef std::shared_ptr<marker_viz_srvs::srv::ResetMarkerColor::Request> ResetRequest;
typedef std::shared_ptr<marker_viz_srvs::srv::ResetMarkerColor::Response> ResetResponse;

class MarkerVisualizer : public rclcpp::Node
{
public:
  MarkerVisualizer();
private:
  void marker_callback(const MocapMarkersSharedPtr msg) const;
  void marker_with_id_callback(const MocapMarkersWithIdSharedPtr msg) const;
  void set_marker_color(const SetRequest request, 
                                          const SetResponse response);
  void reset_marker_color(const ResetRequest request, 
                                            const ResetResponse response);
  void process_marker(int index, const geometry_msgs::msg::Point& translation) const;
  rclcpp::Publisher<VizMarker>::SharedPtr publisher_;
  rclcpp::Subscription<MocapMarkers>::SharedPtr markers_subscription_;
  rclcpp::Subscription<MocapMarkersWithId>::SharedPtr markers_with_id_subscription_;
  rclcpp::Service<SetMarkerColor>::SharedPtr set_marker_color_;
  rclcpp::Service<ResetMarkerColor>::SharedPtr reset_marker_color_;
  geometry_msgs::msg::Vector3 marker_scale_;
  float marker_lifetime_;
  std::string marker_frame_;
  std::string namespace_;
  std_msgs::msg::ColorRGBA default_marker_color_;
  std::map<int, std_msgs::msg::ColorRGBA> marker_color_;
  bool use_markers_with_id_;
};

#endif
