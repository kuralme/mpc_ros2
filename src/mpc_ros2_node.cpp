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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>

#include "mpc_ros2/mpc.hpp"

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mpc_ros2");

namespace MpcRos
{
/**
* @brief MPC Controller ROS2 class
*        ...
*/
class MPCRosNode : public rclcpp::Node
{
  public:
    MPCRosNode(const std::string& nodeName, const rclcpp::NodeOptions& options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
    void odomCallback(const nav_msgs::msg::Odometry::ConstPtr& odomMsg);
    void calculateControl(void);

  private:   
    // ROS2 node parameters
    double dt_, w_, throttle_, speed_, maxspeed_, pathLength_, goalRadius_, waypointsDist_;
    int controlRate_, downSampling_, threadNumbers_;
    bool goalReceived_, goalReached_, pathComputed_, pubTwistFlag_, debugInfo_, delayMode_;
    std::string globalPathTopic_, goalTopic_;
    std::string mapFrame_, odomFrame_, baseFrame_, robotFrame_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{nullptr};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_{nullptr};

    // MPC solver
    MPC _mpc;
    std::map<std::string, double> _mpc_params;
};

/**
* @brief MPC Node class constructor
*        ...
*/
MPCRosNode::MPCRosNode(const std::string& nodeName, const rclcpp::NodeOptions& options)
  : Node(nodeName, options)
{
  // Get ROS2 parameters from yaml
  this->get_parameter_or<std::string>("map_frame", mapFrame_, "map");
  this->get_parameter_or<std::string>("odom_frame", odomFrame_, "odom");
  this->get_parameter_or<std::string>("base_frame", baseFrame_, "base_link");
  this->get_parameter_or<int>("control_rate", controlRate_ , 10);
  this->get_parameter_or<bool>("delay_mode", delayMode_, "false");
  this->get_parameter_or<bool>("debug_info", debugInfo_, "false");

  std::cout << "======= Loading ROS2 parameters =======" << std::endl;
  std::cout << "- Map frame name : " << mapFrame_   << std::endl;
  std::cout << "- Odom frame name: " << odomFrame_  << std::endl;
  std::cout << "- Base frame name: " << baseFrame_  << std::endl;
  std::cout << "- Control rate   : " << controlRate_<< std::endl;
  std::cout << "- Delay mode     : " << delayMode_  << std::endl;
  std::cout << "- Debug info     : " << debugInfo_  << std::endl;

  // Get MPC solver parameters from yaml
  double _mpcSteps, _mpcRefCte, _mpcRefVel, _refEtheta, _wCte, _wEtheta, _wVel,
      _wAngvel, _wAccel, _wAngveld, _wAcceld, _maxAngvel, _maxThrottle, _boundValue;
  
  this->get_parameter_or<double>("mpc_steps", _mpcSteps, 0.0);
  this->get_parameter_or<double>("mpc_ref_cte", _mpcRefCte, 0.0);
  this->get_parameter_or<double>("mpc_ref_vel", _mpcRefVel, 0.0);

  std::cout << "======= Loading MPC parameters =======" << std::endl;
  std::cout << "mpc_steps     : " << _mpcSteps << std::endl;
  std::cout << "mpc_ref_vel   : " << _mpcRefCte << std::endl;
  std::cout << "mpc_w_cte     : " << _mpcRefVel << std::endl;
  // std::cout << "mpc_wEtheta  : " << delayMode_ << std::endl;
  // std::cout << "mpc_max_angvel: " << debugInfo_ << std::endl;

  // Fill MPC solver parameter map
  _mpc_params["STEPS"]      = _mpcSteps;
  _mpc_params["REF_CTE"]    = _mpcRefCte;
  _mpc_params["REF_V"]      = _mpcRefVel;
  
  // Construct MPC Solver object
  auto _mpc= std::make_shared<MPC>(_mpc_params);

  // Create ROS2 Publishers and Subscribers
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "topic", 10, std::bind(&MPCRosNode::odomCallback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<std_msgs::msg::String>("controller_topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&MPCRosNode::calculateControl, this));
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/**
* @brief Node interfacing
*        Neccessary for multithreaded execution
*/
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MPCRosNode::getNodeBaseInterface()
{
  return this->get_node_base_interface();
}

/**
* @brief Odometry ROS2 subscription
*        ...
*/
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

/**
* @brief MPC Controller loop
*        ...
*/
void MPCRosNode::calculateControl(void)
{

}

} // namespace MpcRos

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::MultiThreadedExecutor executor;

  options.automatically_declare_parameters_from_overrides(true);
  auto mpc_ros2_node = std::make_shared<MpcRos::MPCRosNode>("mpc_ros2_node", options);
  
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