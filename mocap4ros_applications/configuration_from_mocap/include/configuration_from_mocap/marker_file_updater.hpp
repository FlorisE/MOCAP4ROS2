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

#ifndef CONFIGURATION_FROM_MOCAP__MARKER_FILE_UPDATER_HPP_
#define CONFIGURATION_FROM_MOCAP__MARKER_FILE_UPDATER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "configuration_from_mocap/marker_lib.hpp"
#include "mocap4ros_msgs/msg/markers_with_id.hpp"

struct MarkerDistance
{
  MarkerDistance(marker_lib::Marker origin, marker_lib::Marker destination, double distance) : 
    origin(origin), destination(destination), distance(distance) {}
  marker_lib::Marker origin;
  marker_lib::Marker destination;
  double distance;
};

class MarkerFileUpdater : public rclcpp::Node
{
public:
  MarkerFileUpdater();
private:
  bool parse_markers();
  void transform_to_local_frame(
    const std::vector<std::shared_ptr<marker_lib::Marker>> source,
    std::vector<std::shared_ptr<marker_lib::Marker>> target
  );
  void marker_callback(const mocap4ros_msgs::msg::MarkersWithId::SharedPtr);
  void get_marker_distances(
    const std::vector<mocap4ros_msgs::msg::MarkerWithId>& to, 
    const std::vector<std::shared_ptr<marker_lib::Marker>>& from,
    std::vector<MarkerDistance>& res
  );
  void get_lowest_distance(
    const std::vector<MarkerDistance>& distances,
    std::vector<MarkerDistance>& res
  );
  void get_marker_map(
    const std::vector<MarkerDistance>& distance, std::map<int, int>& res
  );
  std::string input_path_;
  std::string output_path_;
  std::string source_frame_;
  bool ready_ = false;
  std::vector<std::shared_ptr<marker_lib::Marker>> markers_;
  rclcpp::Subscription<mocap4ros_msgs::msg::MarkersWithId>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

#endif
