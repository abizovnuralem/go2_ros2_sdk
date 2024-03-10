// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * @author Enrique Fernandez
 * @author Jeremie Deray
 * @author Brighten Lee
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

class TwistMarker
{
public:
  TwistMarker(std::string & frame_id, double scale, double z)
  : frame_id_(frame_id), scale_(scale), z_(z)
  {
    // ID and type:
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::ARROW;

    // Frame ID:
    marker_.header.frame_id = frame_id_;

    // Pre-allocate points for setting the arrow with the twist:
    marker_.points.resize(2);

    // Vertical position:
    marker_.pose.position.z = z_;

    // Scale:
    marker_.scale.x = 0.05 * scale_;
    marker_.scale.y = 2 * marker_.scale.x;

    // Color:
    marker_.color.a = 1.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;

    // Error when all points are zero:
    marker_.points[1].z = 0.01;
  }

  void update(const geometry_msgs::msg::Twist & twist)
  {
    using std::abs;

    marker_.points[1].x = twist.linear.x;

    if (abs(twist.linear.y) > abs(twist.angular.z)) {
      marker_.points[1].y = twist.linear.y;
    } else {
      marker_.points[1].y = twist.angular.z;
    }
  }

  const visualization_msgs::msg::Marker & getMarker()
  {
    return marker_;
  }

private:
  visualization_msgs::msg::Marker marker_;

  std::string frame_id_;
  double scale_;
  double z_;
};

class TwistMarkerPublisher : public rclcpp::Node
{
public:
  TwistMarkerPublisher()
  : Node("twist_marker")
  {
    std::string frame_id;
    double scale;
    double z;

    this->declare_parameter("frame_id", "base_footprint");
    this->declare_parameter("scale", 1.0);
    this->declare_parameter("vertical_position", 2.0);

    this->get_parameter<std::string>("frame_id", frame_id);
    this->get_parameter<double>("scale", scale);
    this->get_parameter<double>("vertical_position", z);

    marker_ = std::make_shared<TwistMarker>(frame_id, scale, z);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "twist", rclcpp::SystemDefaultsQoS(),
      std::bind(&TwistMarkerPublisher::callback, this, std::placeholders::_1));

    pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
      "marker",
      rclcpp::QoS(rclcpp::KeepLast(1)));
  }

  void callback(const geometry_msgs::msg::Twist::ConstSharedPtr twist)
  {
    marker_->update(*twist);

    pub_->publish(marker_->getMarker());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;

  std::shared_ptr<TwistMarker> marker_ = nullptr;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto twist_mux_node = std::make_shared<TwistMarkerPublisher>();

  rclcpp::spin(twist_mux_node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
