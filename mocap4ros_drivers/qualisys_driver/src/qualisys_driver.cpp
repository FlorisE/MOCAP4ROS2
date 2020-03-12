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
// Author: Floris Erich <floris.erich@aist.go.jp>,
//         David Vargas Frutos <david.vargas@urjc.es>
// 
// Also includes code fragments from Kumar Robotics ROS 1 Qualisys driver

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include "qualisys_driver/qualisys_driver.hpp"
#include <lifecycle_msgs/msg/state.hpp>

void QualisysDriver::set_settings_qualisys()
{
}

void QualisysDriver::start_qualisys()
{
  set_settings_qualisys();
  auto period = std::chrono::milliseconds(100);
  rclcpp::Rate d(period);
  while (rclcpp::ok())
  {
    CRTPacket* prt_packet = port_protocol_.GetRTPacket();
    CRTPacket::EPacketType e_type;
    port_protocol_.GetCurrentFrame(CRTProtocol::cComponent3dNoLabels);
    if (port_protocol_.ReceiveRTPacket(e_type, true))
    {
      switch (e_type)
      {
        case CRTPacket::PacketError:
          RCLCPP_ERROR(get_logger(), std::string("Error when streaming frames: ") +
              port_protocol_.GetRTPacket()->GetErrorString());
          break;
        case CRTPacket::PacketNoMoreData:
          RCLCPP_WARN(get_logger(), "No more data");
          break;
        case CRTPacket::PacketData:
          process_packet(prt_packet);
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Unknown CRTPacket");
      }
    }
    d.sleep();
  }

  return;
}

void QualisysDriver::process_packet(CRTPacket* const packet)
{
  unsigned int marker_count = packet->Get3DNoLabelsMarkerCount();
  int frame_number = packet->GetFrameNumber();

  int frame_diff = 0;
  if (last_frame_number_ != 0)
  {
    frame_diff = frame_number - last_frame_number_;
    frame_count_ += frame_diff;
    if (frame_diff > 1)
    {
      dropped_frame_count_ += frame_diff;
      double dropped_frame_pct = static_cast<double>(dropped_frame_count_ / frame_count_ * 100);

      RCLCPP_DEBUG(get_logger(), 
        "%d more (total %d / %d, %f %%) frame(s) dropped. Consider adjusting rates",
        frame_diff, dropped_frame_count_, frame_count_, dropped_frame_pct
      );
    }
  }
  last_frame_number_ = frame_number;

  if (use_markers_with_id_)
  {
    mocap4ros_msgs::msg::MarkersWithId markers_msg;
    markers_msg.header.stamp = rclcpp::Clock().now();
    markers_msg.frame_number = frame_number;

    for (unsigned int i = 0; i < marker_count; ++i)
    {
      float x, y, z;
      unsigned int id;
      packet->Get3DNoLabelsMarker(i, x, y, z, id);
      mocap4ros_msgs::msg::MarkerWithId this_marker;
      this_marker.translation.x = x/1000;
      this_marker.translation.y = y/1000;
      this_marker.translation.z = z/1000;
      markers_msg.markers.push_back( this_marker );
    }

    marker_with_id_pub_->publish( markers_msg );
  }
  else
  {
    mocap4ros_msgs::msg::Markers markers_msg;
    markers_msg.header.stamp = rclcpp::Clock().now();
    markers_msg.frame_number = frame_number;

    for (unsigned int i = 0; i < marker_count; ++i)
    {
      float x, y, z;
      unsigned int id;
      packet->Get3DNoLabelsMarker(i, x, y, z, id);
      mocap4ros_msgs::msg::Marker this_marker;
      this_marker.translation.x = x/1000;
      this_marker.translation.y = y/1000;
      this_marker.translation.z = z/1000;
      markers_msg.markers.push_back( this_marker );
    }

    marker_pub_->publish( markers_msg );
  }

  return;
}

bool QualisysDriver::stop_qualisys()
{
  RCLCPP_INFO(get_logger(), "Stopping the Qualisys motion capture");
  port_protocol_.StreamFramesStop();
  port_protocol_.Disconnect();
  return true;
}

QualisysDriver::QualisysDriver(const rclcpp::NodeOptions node_options)
  : rclcpp_lifecycle::LifecycleNode("qualisys_driver_node", node_options)
{
  initParameters();
}
    
using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT QualisysDriver::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  auto rmw_qos_history_policy = name_to_history_policy_map.find(qos_history_policy_);
  auto rmw_qos_reliability_policy = name_to_reliability_policy_map.find(qos_reliability_policy_);
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      // The history policy determines how messages are saved until taken by
      // the reader.
      // KEEP_ALL saves all messages until they are taken.
      // KEEP_LAST enforces a limit on the number of messages that are saved,
      // specified by the "depth" parameter.
      rmw_qos_history_policy->second,
      // Depth represents how many messages to store in history when the
      // history policy is KEEP_LAST.
      qos_depth_
  ));
  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  qos.reliability(rmw_qos_reliability_policy->second);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/qualisys_driver/change_state");

  marker_pub_ = create_publisher<mocap4ros_msgs::msg::Markers>(
    "/markers", 100);

  marker_with_id_pub_ = create_publisher<mocap4ros_msgs::msg::MarkersWithId>(
    "/markers_with_id", 100);

  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/qualisys_driver/update_notify", qos);

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_activate();
  marker_pub_->on_activate();
  bool success = connect_qualisys();

  if (success)
  {
    RCLCPP_INFO(get_logger(), "Activated!\n");

    return CallbackReturnT::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Unable to activate!\n");

    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT QualisysDriver::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  /* Clean up stuff */
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  /* Shut down stuff */
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  return CallbackReturnT::SUCCESS;
}

bool QualisysDriver::connect_qualisys()
{
  RCLCPP_WARN(get_logger(), "Trying to connect to Qualisys host at %s:%d", host_name_.c_str(), port_);

  if(!port_protocol_.Connect((char *)host_name_.data(), port_, 0, 1, 7)) {
    RCLCPP_FATAL(get_logger(), "Connection error");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Connected");

  bool settings_read;
  port_protocol_.Read6DOFSettings(settings_read);

  start_qualisys();

  return settings_read;
}

void QualisysDriver::initParameters()
{
  declare_parameter<std::string>("host_name", "mocap");
  declare_parameter<int>("port", 22222);
  declare_parameter<int>("last_frame_number", 0);
  declare_parameter<int>("frame_count", 0);
  declare_parameter<int>("dropped_frame_count", 0);
  declare_parameter<std::string>("qos_history_policy", "keep_all");
  declare_parameter<std::string>("qos_reliability_policy", "best_effort");
  declare_parameter<int>("qos_depth", 10);
  declare_parameter<bool>("use_markers_with_id", true);

  get_parameter<std::string>("host_name", host_name_);
  get_parameter<int>("port", port_);
  get_parameter<int>("last_frame_number", last_frame_number_);
  get_parameter<int>("frame_count", frame_count_);
  get_parameter<int>("dropped_frame_count", dropped_frame_count_);
  get_parameter<std::string>("qos_history_policy", qos_history_policy_);
  get_parameter<std::string>("qos_reliability_policy", qos_reliability_policy_);
  get_parameter<int>("qos_depth", qos_depth_);
  get_parameter<bool>("use_markers_with_id", use_markers_with_id_);

  RCLCPP_WARN(get_logger(),
    "Param host_name: %s", host_name_.c_str());
  RCLCPP_WARN(get_logger(),
    "Param port: %s", port_);
  RCLCPP_WARN(get_logger(),
    "Param last_frame_number: %s", last_frame_number_);
  RCLCPP_WARN(get_logger(),
    "Param frame_count: %d", frame_count_);
  RCLCPP_WARN(get_logger(),
    "Param dropped_frame_count: %d", dropped_frame_count_);
  RCLCPP_WARN(get_logger(),
    "Param qos_history_policy: %s", qos_history_policy_.c_str());
  RCLCPP_WARN(get_logger(),
    "Param qos_reliability_policy: %s", qos_reliability_policy_.c_str());
  RCLCPP_WARN(get_logger(),
    "Param qos_depth: %d", qos_depth_);
  RCLCPP_WARN(get_logger(),
    "Param use_markers_with_id: %s", use_markers_with_id_ ? "true" : "false");
}
