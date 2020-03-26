// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
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

#ifndef CONFIGURATION_FROM_MOCAP__ESTIMATOR_HPP_
#define CONFIGURATION_FROM_MOCAP__ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include "configuration_from_mocap/estimator.hpp"

#include "configuration_from_mocap/marker_lib.hpp"
#include "mocap4ros_msgs/msg/markers_with_id.hpp"

namespace ConfigurationFromMocap
{

class Estimator : public rclcpp::Node
{
public:
  Estimator();
private:
  void markers_callback(
    const mocap4ros_msgs::msg::MarkersWithId::SharedPtr markers
  );
  std::string source_frame_;
  std::string global_frame_;
  urdf::Model robot_model_;
  std::multimap<std::string, std::shared_ptr<marker_lib::Marker>> stored_markers_;
  std::vector<std::shared_ptr<marker_lib::Marker>> marker_list_;
  rclcpp::Subscription<mocap4ros_msgs::msg::MarkersWithId>::SharedPtr markers_sub_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  geometry_msgs::msg::TransformStamped global_frame_to_source_frame_;
};

}

#endif
