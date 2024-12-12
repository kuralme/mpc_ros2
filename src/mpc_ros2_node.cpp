/*
 * MIT License
 * 
 * Copyright (c) 2024 Mustafa Ege Kural
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "std_msgs/msg/string.hpp"

#include "mpc_ros2/mpc.hpp"

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mpc_ros2");
namespace MpcRos
{
class MPCRosNode : public rclcpp::Node
{
  public:
    MPCRosNode(const rclcpp::NodeOptions& options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
    void odomCallback(const nav_msgs::msg::Odometry::ConstPtr& odomMsg);
    void calculateControl(void);

  private:
    int param_a_;
    bool param_b_;
    std::string param_c_;
    size_t param_d_;

    // MPC solver
    std::map<std::string, double> _mpc_params;
    MPC _mpc;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

MPCRosNode::MPCRosNode(const rclcpp::NodeOptions& options)
  : Node("mpc_ros2_node", options)
{
  // Controller parameters
  param_a_ = node_->get_parameter("param_a").as_int();
  param_b_ = node_->get_parameter("param_b").as_bool();
  param_c_ = node_->get_parameter("param_c").as_int();
  
  // MPC solver parameters
  _mpc_params["mpcparam_a"] = node_->get_parameter("mpcparam_a").as_int();
  _mpc_params["mpcparam_b"] = node_->get_parameter("mpcparam_b").as_bool();
  _mpc_params["mpcparam_c"] = node_->get_parameter("mpcparam_c").as_int();
  auto _mpc= std::make_shared<MPC>(_mpc_params);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers and Subscribers
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "topic", 10, std::bind(&MPCRosNode::odomCallback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<std_msgs::msg::String>("controller_topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&MPCRosNode::calculateControl, this));
  
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MPCRosNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MPCRosNode::odomCallback(const nav_msgs::msg::Odometry::ConstPtr& odomMsg)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  
  std::string fromFrameRel = "map";
  // std::string fromFrameRel = target_frame_.c_str();
  std::string toFrameRel = "odom";
  try {
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time when = now - rclcpp::Duration(5, 0);
    transformStamped = tf_buffer_->lookupTransform(
      toFrameRel,
      now,
      fromFrameRel,
      when,
      "world",
      50ms);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s",
      toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }
  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

void MPCRosNode::calculateControl(void)
{

}

} // namespace MpcRos

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mpc_ros2_node = std::make_shared<MpcRos::MPCRosNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  // Extra thread to handle ROS2 events
  auto spin_thread = std::make_unique<std::thread>([&executor, &mpc_ros2_node]() {
    executor.add_node(mpc_ros2_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mpc_ros2_node->getNodeBaseInterface());
  });

  mpc_ros2_node->calculateControl();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}