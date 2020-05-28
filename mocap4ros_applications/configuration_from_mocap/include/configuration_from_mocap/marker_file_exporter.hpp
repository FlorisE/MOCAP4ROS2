// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Floris Erich <floris.erich@aist.go.jp>

#ifndef CONFIGURATION_FROM_MOCAP__MARKER_FILE_EXPORTER_HPP_
#define CONFIGURATION_FROM_MOCAP__MARKER_FILE_EXPORTER_HPP_

#include <string>
#include <vector>
#include <set>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "mocap4ros_msgs/msg/markers_with_id.hpp"
#include "marker_viz_srvs/srv/set_marker_color.hpp"
#include "marker_viz_srvs/srv/reset_marker_color.hpp"

using std::placeholders::_1;

typedef marker_viz_srvs::srv::SetMarkerColor SetMarkerColor;
typedef marker_viz_srvs::srv::ResetMarkerColor ResetMarkerColor;
typedef marker_viz_srvs::srv::SetMarkerColor::Request SetMarkerColorRequest;
typedef marker_viz_srvs::srv::ResetMarkerColor::Request ResetMarkerColorRequest;

struct Marker {
  int32_t id;
  float x;
  float y;
  float z;
  std::string label;
};

class MarkerFileExporter : public rclcpp::Node
{
public:
  MarkerFileExporter() : rclcpp::Node("marker_file_exporter")
  {
    declare_parameter<std::string>("output_path", std::string("markers.xml"));
    declare_parameter<std::string>("marker_origin_frame", std::string(""));
    declare_parameter<std::string>("robot_origin_frame", std::string(""));
    declare_parameter<double>("listen_time", 5.0);

    get_parameter<std::string>("output_path", output_path_);
    get_parameter<std::string>("marker_origin_frame", marker_origin_frame_);
    get_parameter<std::string>("robot_origin_frame", robot_origin_frame_);
    get_parameter<double>("listen_time", listen_time_);

    subscription_ = create_subscription<mocap4ros_msgs::msg::MarkersWithId>(
      "markers", 10, std::bind(&MarkerFileExporter::marker_callback, this, _1)
    );

    set_marker_color_ = create_client<SetMarkerColor>("set_marker_color");
    reset_marker_color_ = create_client<ResetMarkerColor>("reset_marker_color");

    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }
private:
  void marker_callback(const mocap4ros_msgs::msg::MarkersWithId::SharedPtr msg);
  void process_markers(const std::vector<mocap4ros_msgs::msg::MarkerWithId>& markers);
  void add_or_update_marker(int32_t id, double x, double y, double z);
  void add_marker(int32_t id, double x, double y, double z);
  void update_marker(Marker& marker, double x, double y, double z);
  void handle_markers_and_frames();
  bool enter_marker_frame(Marker&);
  void write_markers();
  bool shutdown_if_exit(const std::string&);
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<mocap4ros_msgs::msg::MarkersWithId>::SharedPtr subscription_;
  rclcpp::Client<SetMarkerColor>::SharedPtr set_marker_color_;
  rclcpp::Client<ResetMarkerColor>::SharedPtr reset_marker_color_;
  std::string output_path_;
  std::string marker_origin_frame_;
  std::string robot_origin_frame_;
  double listen_time_;
  bool markers_received_ = false;
  bool frames_received_ = false;
  bool results_written_ = false;
  std::vector<Marker> markers_;
  rclcpp::Time init_time_ = rclcpp::Clock().now();
  std::set<std::string> frames_;
  std::multimap<std::string, Marker> frame_markers_;
};

#endif
