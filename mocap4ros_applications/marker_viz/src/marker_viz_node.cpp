#include "marker_viz/marker_viz_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

MarkerVisualizer::MarkerVisualizer() : Node("marker_visualizer")
{
  publisher_ = this->create_publisher<VizMarker>("visualization_marker", 100);
  set_marker_color_ = this->create_service<SetMarkerColor>("set_marker_color", std::bind(&MarkerVisualizer::set_marker_color, this, _1, _2));
  reset_marker_color_ = this->create_service<ResetMarkerColor>("reset_marker_color", std::bind(&MarkerVisualizer::reset_marker_color, this, _1, _2));

  declare_parameter<float>("default_marker_color_r", 0.0f);
  declare_parameter<float>("default_marker_color_g", 1.0f);
  declare_parameter<float>("default_marker_color_b", 0.0f);
  declare_parameter<float>("default_marker_color_a", 1.0f);
  declare_parameter<double>("marker_scale_x", 0.014f);
  declare_parameter<double>("marker_scale_y", 0.014f);
  declare_parameter<double>("marker_scale_z", 0.014f);
  declare_parameter<float>("marker_lifetime", 0.1f);
  declare_parameter<std::string>("marker_frame", "mocap");
  declare_parameter<std::string>("namespace", "mocap_markers");
  declare_parameter<bool>("use_markers_with_id", true);

  get_parameter<float>("default_marker_color_r", default_marker_color_.r);
  get_parameter<float>("default_marker_color_g", default_marker_color_.g);
  get_parameter<float>("default_marker_color_b", default_marker_color_.b);
  get_parameter<float>("default_marker_color_a", default_marker_color_.a);
  get_parameter<double>("marker_scale_x", marker_scale_.x);
  get_parameter<double>("marker_scale_y", marker_scale_.y);
  get_parameter<double>("marker_scale_z", marker_scale_.z);
  get_parameter<float>("marker_lifetime", marker_lifetime_);
  get_parameter<std::string>("marker_frame", marker_frame_);
  get_parameter<std::string>("namespace", namespace_);
  get_parameter<bool>("use_markers_with_id", use_markers_with_id_);

  if (use_markers_with_id_)
  {
    markers_with_id_subscription_ = this->create_subscription<MocapMarkersWithId>("markers_with_id", 100, std::bind(&MarkerVisualizer::marker_with_id_callback, this, _1));
  }
  else
  {
    markers_subscription_ = this->create_subscription<MocapMarkers>("markers", 100, std::bind(&MarkerVisualizer::marker_callback, this, _1));
  }
}

void MarkerVisualizer::set_marker_color(const SetRequest request, 
                                        const SetResponse response)
{
  if (!use_markers_with_id_)
  {
    RCLCPP_ERROR(get_logger(), "Can only set marker color if markers are assigned an id");
    return;
  }
  auto marker_color_it = marker_color_.find(request->id.data);
  if (marker_color_it != marker_color_.end())
  {
    marker_color_it->second = request->color;
  }

  return;
}

void MarkerVisualizer::reset_marker_color(const ResetRequest request, 
                                          const ResetResponse response)
{
  if (!use_markers_with_id_)
  {
    RCLCPP_ERROR(get_logger(), "Can only reset marker color if markers are assigned an id");
    return;
  }
  auto marker_color_it = marker_color_.find(request->id.data);
  if (marker_color_it != marker_color_.end())
  {
    marker_color_it->second = default_marker_color_;
  }

  return;
}

void MarkerVisualizer::marker_callback(const MocapMarkersSharedPtr msg) const
{
  for (const MocapMarker& marker : msg->markers)
  {
    process_marker(-1, marker.translation);
  }
}

void MarkerVisualizer::marker_with_id_callback(const MocapMarkersWithIdSharedPtr msg) const
{
  for (const MocapMarkerWithId& marker : msg->markers)
  {
    process_marker(marker.index, marker.translation);
  }
}

void MarkerVisualizer::process_marker(int index, const geometry_msgs::msg::Point& translation) const
{
  VizMarker viz_marker;
  viz_marker.header.frame_id = marker_frame_;
  viz_marker.header.stamp = rclcpp::Clock().now();
  viz_marker.ns = namespace_;
  viz_marker.color = default_marker_color_;
  viz_marker.id = index;
  if (use_markers_with_id_)
  {
    auto marker_color_it = marker_color_.find(index);
    if (marker_color_it != marker_color_.end())
    {
      viz_marker.color = marker_color_it->second;
    }
  }
  viz_marker.type = visualization_msgs::msg::Marker::SPHERE;
  viz_marker.action = visualization_msgs::msg::Marker::ADD;
  viz_marker.pose.position.x = translation.x;
  viz_marker.pose.position.y = translation.y;
  viz_marker.pose.position.z = translation.z;
  viz_marker.pose.orientation.x = 0.0f;
  viz_marker.pose.orientation.y = 0.0f;
  viz_marker.pose.orientation.z = 0.0f;
  viz_marker.pose.orientation.w = 1.0f;
  viz_marker.scale = marker_scale_;
  viz_marker.lifetime = rclcpp::Duration(marker_lifetime_);
  publisher_->publish(viz_marker);
}
