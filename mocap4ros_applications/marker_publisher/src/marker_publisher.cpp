#include <chrono>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "mocap4ros_msgs/msg/marker.hpp"
#include "mocap4ros_msgs/msg/markers.hpp"
#include "mocap4ros_msgs/msg/marker_with_id.hpp"
#include "mocap4ros_msgs/msg/markers_with_id.hpp"

typedef mocap4ros_msgs::msg::MarkerWithId MocapMarkerWithId;
typedef mocap4ros_msgs::msg::MarkersWithId MocapMarkers;

using namespace std::chrono_literals;

class MarkerPublisher : public rclcpp::Node
{
public:
  MarkerPublisher() : Node("marker_publisher")
  {
    publisher_ = this->create_publisher<MocapMarkers>("markers", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&MarkerPublisher::timer_callback, this));
  }

  void timer_callback()
  {
    MocapMarkers markers;
    for (int i = 1; i <= 10; ++i)
    {
      MocapMarkerWithId marker;
      marker.index = i;
      marker.translation.x = 0;
      marker.translation.y = 0;
      marker.translation.z = 0.1 * i;
      markers.markers.push_back(marker);
    }
    publisher_->publish(markers);

    return;
  }
private:
  rclcpp::Publisher<MocapMarkers>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
