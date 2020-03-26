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

#include <cmath>
#include <map>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "configuration_from_mocap/marker_file_updater.hpp"
#include "mocap4ros_msgs/msg/marker_with_id.hpp"

using std::placeholders::_1;

MarkerFileUpdater::MarkerFileUpdater() : rclcpp::Node("marker_file_updater")
{
  declare_parameter<std::string>("marker_path", std::string("markers.xml"));
  get_parameter<std::string>("marker_path", input_path_);

  declare_parameter<std::string>("output_path", input_path_);
  get_parameter<std::string>("output_path", output_path_);

  declare_parameter<std::string>("source_frame", std::string(""));
  get_parameter<std::string>("source_frame", source_frame_);

  ready_ = parse_markers();
  if (!ready_)
  {
    return;
  }

  subscription_ = create_subscription<mocap4ros_msgs::msg::MarkersWithId>(
    "markers", 10, std::bind(&MarkerFileUpdater::marker_callback, this, _1)
  );
}

bool MarkerFileUpdater::parse_markers()
{
  std::vector<std::shared_ptr<marker_lib::Marker>> loaded_markers;
  if (!marker_lib::parse_markers(input_path_, loaded_markers))
  {
    RCLCPP_ERROR(get_logger(), "Unable to parse marker definition");
    return false;
  }

  transform_to_local_frame(loaded_markers, markers_);
  return true;
}

void MarkerFileUpdater::transform_to_local_frame(
    const std::vector<std::shared_ptr<marker_lib::Marker>> source,
    std::vector<std::shared_ptr<marker_lib::Marker>> target
)
{
  for (auto marker_ptr : source)
  {
    auto t = buffer_->lookupTransform(
      marker_ptr->frame, source_frame_, rclcpp::Time(0, 0), rclcpp::Duration(1.0, 0)
    );
    marker_lib::Marker marker(
      marker_ptr->id,
      marker_ptr->x - t.transform.translation.x,
      marker_ptr->y - t.transform.translation.y,
      marker_ptr->z - t.transform.translation.z,
      marker_ptr->label,
      marker_ptr->frame
    );
    target.push_back( std::make_shared<marker_lib::Marker>( marker ) );
  }
}

void MarkerFileUpdater::get_marker_distances(
  const std::vector<mocap4ros_msgs::msg::MarkerWithId>& to, 
  const std::vector<std::shared_ptr<marker_lib::Marker>>& from,
  std::vector<MarkerDistance>& res
)
{
  for (const mocap4ros_msgs::msg::MarkerWithId& marker_msg : to)
  {
    marker_lib::Marker measured_marker(
      marker_msg.index, marker_msg.translation.x, marker_msg.translation.y, marker_msg.translation.z
    );
    for (const std::shared_ptr<marker_lib::Marker> stored_marker : from)
    {
      double distance = sqrt(
        pow(measured_marker.x - stored_marker->x, 2) +
        pow(measured_marker.y - stored_marker->y, 2) +
        pow(measured_marker.z - stored_marker->z, 2)
      );
      res.push_back(MarkerDistance(*stored_marker, measured_marker, distance));
    }
  }
}

void MarkerFileUpdater::get_lowest_distance(
  const std::vector<MarkerDistance>& distances,
  std::vector<MarkerDistance>& res
)
{
  for (const MarkerDistance& d : distances)
  {
    MarkerDistance* found = nullptr;
    for (MarkerDistance& temp : res)
    {
      if (temp.origin == d.origin)
      {
        found = &temp;
        break;
      }
    }
    if (found != nullptr && found->distance > d.distance)
    {
      found->destination = d.destination;
      found->distance = d.distance;
    }
    else
    {
      res.push_back(d);
    }
  }
}

void MarkerFileUpdater::get_marker_map(
  const std::vector<MarkerDistance>& distance, std::map<int, int>& res
)
{
  for (const MarkerDistance& d : distance)
  {
    res.insert(std::make_pair(d.origin.id, d.destination.id));
  }
}

void MarkerFileUpdater::marker_callback(const mocap4ros_msgs::msg::MarkersWithId::SharedPtr markers)
{
  // origin of MarkerDistance is the stored marker, destination is the measured marker
  std::vector<MarkerDistance> marker_distances;
  get_marker_distances(markers->markers, markers_, marker_distances);

  std::vector<MarkerDistance> lowest_distance;
  get_lowest_distance(marker_distances, lowest_distance);

  std::map<int, int> marker_map;
  get_marker_map(lowest_distance, marker_map);

  marker_lib::write_markers(output_path_, marker_map);

  rclcpp::shutdown();
}
