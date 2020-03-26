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

#ifndef CONFIGURATION_FROM_MOCAP__MARKER_LIB_HPP_
#define CONFIGURATION_FROM_MOCAP__MARKER_LIB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tinyxml.h>
#include <string>
#include <map>
#include <memory>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

namespace marker_lib
{

enum JointType {
  FIXED,
  PRISMATIC,
  REVOLUTE
};

struct Marker {
  Marker(
    int id=-1, double x=0, double y=0, double z=0, std::string label="", std::string frame=""
  ) : id(id), x(x), y(y), z(z), label(label), frame(frame) {}

  int id;
  double x;
  double y;
  double z;
  std::string label;
  std::string frame;
  JointType joint_type;
  double norm()
  {
    return std::sqrt(x*x + y*y + z*z);
  }
};

bool operator==(const Marker& left, const Marker& right);

bool operator<(const Marker& left, const Marker& right);

std::shared_ptr<Marker> find_marker(int marker_id, const std::vector<std::shared_ptr<Marker>>& markers);

void transform_markers_to_frame(
  const std::string& frame,
  const std::multimap<std::string, Marker>& markers,
  tf2_ros::Buffer& buffer,
  std::multimap<std::string, Marker>& res
);
bool parse_markers(const std::string& input, std::multimap<std::string, std::shared_ptr<Marker>>& res);

bool parse_markers(const std::string& input, std::vector<std::shared_ptr<Marker>>& res);

bool write_markers(const std::string& output, const std::map<int, int>& markers);

void print_markers(const std::vector<Marker>& markers);

}
#endif
