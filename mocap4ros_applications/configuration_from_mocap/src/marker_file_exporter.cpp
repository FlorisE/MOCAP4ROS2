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

#include "configuration_from_mocap/marker_file_exporter.hpp"
#include <fstream>

void MarkerFileExporter::marker_callback(const mocap4ros_msgs::msg::MarkersWithId::SharedPtr msg)
{
  process_markers(msg->markers);
  markers_received_ = true;
  handle_markers_and_frames();
}

void MarkerFileExporter::process_markers(
  const std::vector<mocap4ros_msgs::msg::MarkerWithId>& markers
)
{
  geometry_msgs::msg::TransformStamped transform = buffer_->lookupTransform(
    marker_origin_frame_, robot_origin_frame_, rclcpp::Time(0, 0), rclcpp::Duration(1.0, 0)
  );
  Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);

  for (const mocap4ros_msgs::msg::MarkerWithId& marker : markers)
  {
    Eigen::Vector3d v(marker.translation.x, marker.translation.y, marker.translation.z);
    Eigen::Vector3d res = eigen_transform.inverse() * v;
    add_or_update_marker(marker.index, res(0), res(1), res(2));
  }
}

void MarkerFileExporter::add_or_update_marker(int32_t id, double x, double y, double z) 
{
  bool marker_already_exists = false;
  for (Marker& existing_marker : markers_)
  {
    if (existing_marker.id == id)
    {
      marker_already_exists = true;
      update_marker(existing_marker, x, y, z);
      break; // do not add markers that already exist twice
    }
  }
  if (!marker_already_exists)
  {
    add_marker(id, x, y, z);
  }
}

void MarkerFileExporter::add_marker(int32_t id, double x, double y, double z)
{
  Marker simple_marker;
  simple_marker.id = id;
  update_marker(simple_marker, x, y, z);
  markers_.push_back(simple_marker);
}

void MarkerFileExporter::update_marker(Marker& marker, double x, double y, double z)
{
  marker.x = x;
  marker.y = y;
  marker.z = z;
}

void MarkerFileExporter::handle_markers_and_frames()
{
  // wait until enough samples are collected
  if ((rclcpp::Clock().now().seconds() - init_time_.seconds()) < listen_time_) 
    return;
  if (markers_received_ && frames_received_ && !results_written_)
  {
    std_msgs::msg::ColorRGBA highlight_color;
    highlight_color.r = 1.0f;
    highlight_color.g = 0.0f;
    highlight_color.b = 0.0f;
    highlight_color.a = 1.0f;

    std::cout << "Frames:\n";
    for (const std::string& frame : frames_)
    {
      std::cout << frame << '\n';
    }

    for (Marker& marker : markers_)
    {
      auto set_request = std::make_shared<SetMarkerColorRequest>();
      set_request->id.data = marker.id;
      set_request->color = highlight_color;
      auto set_result = set_marker_color_->async_send_request(set_request);
      rclcpp::spin_until_future_complete(
        shared_from_this(), set_result
      );
      bool marker_frame_entered = enter_marker_frame(marker);
      auto reset_request = std::make_shared<ResetMarkerColorRequest>();
      reset_request->id.data = marker.id;
      auto reset_result = reset_marker_color_->async_send_request(reset_request);
      rclcpp::spin_until_future_complete(
        shared_from_this(), reset_result
      );
      if (!marker_frame_entered)
        return;
    }

    std::cout << "Processed all markers received" << std::endl;
    write_markers();
    rclcpp::shutdown();
  }
}

bool MarkerFileExporter::shutdown_if_exit(const std::string& frame)
{
  if (frame == "exit")
  {
    rclcpp::shutdown();
    return true;
  }
  return false;
}

bool MarkerFileExporter::enter_marker_frame(Marker& marker)
{
  std::string frame;
  std::cout << "Marker id: " << marker.id << '\n'
            << "Enter frame id to assign to this marker:\n";
  std::getline(std::cin, frame);
  if (shutdown_if_exit(frame))
    return false;
  while (frames_.find(frame) == frames_.end())
  {
    std::cout << "The entered frame id does not exist.\n"
              << "Enter frame id to assign to this marker:\n";
    std::getline(std::cin, frame);
    if (shutdown_if_exit(frame))
      return false;
  }

  frame_markers_.insert(std::make_pair(frame, marker));
  return true;
}

void MarkerFileExporter::write_markers()
{
  std::ofstream myfile(output_path_, std::ios::out);
  myfile << "<markerFrames>\n";
  std::string latched("");
  for (std::pair<std::string, Marker> frame_marker : frame_markers_)
  {
    std::string frame(frame_marker.first);
    Marker marker = frame_marker.second;
    if (latched != "" && frame != latched)
    {
      myfile << "  </frame>\n";
    }
    if (frame != latched)
      myfile << "  <frame id=\"" << frame << "\" joint_type=\"revolute\">\n";
    myfile << "    <marker mocap_id=\"" << marker.id << "\">" << 
              marker.x << " " << marker.y << " " << marker.z << 
              "</marker>\n";
    latched = frame;
  }
  if (latched != "")
    myfile << "  </frame>\n";
  myfile << "</markerFrames>\n";
  myfile.close();
  results_written_ = true;
}
