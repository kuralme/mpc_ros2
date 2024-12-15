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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
    void glPathCallback(const nav_msgs::msg::Path::ConstPtr& glbPathMsg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::ConstPtr& goalMsg);
    void localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& locMsg);
    void calculateControl(void);

  private:
    // ROS2 node parameters
    size_t downSampleRate_ = 10;
    double dt_, maxSpeed_, controlRate_, pathLength_, goalRadius_, waypointsDist_;
    bool goalReceived_, goalReached_, pathComputed_, debugInfo_, delayMode_;
    std::string odomPath_Otn_, mpcPath_Otn_, glbPath_Itn_, loc_Itn_, goal_Itn_;
    std::string mapFrame_, odomFrame_, robotFrame_;
    geometry_msgs::msg::Point goalPoint_;
    nav_msgs::msg::Path globalPath_, mpcTraj_;
    rclcpp::TimerBase::SharedPtr controlTimer_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMpcPath_{nullptr}, pubOdomPath_{nullptr};
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr glbPathSub_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr locSub_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_{nullptr};

    // MPC solver parameters
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
  this->get_parameter_or<std::string>("loc_topic", loc_Itn_, "/amcl");
  this->get_parameter_or<std::string>("goal_topic", goal_Itn_, "/goal");
  this->get_parameter_or<std::string>("global_path_topic", glbPath_Itn_, "/global_plan");
  this->get_parameter_or<std::string>("odom_path_topic", odomPath_Otn_, "odom_path");
  this->get_parameter_or<std::string>("mpc_path_topic", mpcPath_Otn_, "mpc_path");
  this->get_parameter_or<std::string>("robot_frame", robotFrame_, "base_link");
  this->get_parameter_or<std::string>("odom_frame", odomFrame_, "odom");
  this->get_parameter_or<std::string>("map_frame", mapFrame_, "map");
  this->get_parameter_or<double>("max_speed", maxSpeed_ , .5);
  this->get_parameter_or<double>("path_length", pathLength_ , 8.);
  this->get_parameter_or<double>("goal_radius", goalRadius_ , .5);
  this->get_parameter_or<double>("control_rate", controlRate_, 10.);
  this->get_parameter_or<bool>("delay_mode", delayMode_, "false");
  this->get_parameter_or<bool>("debug_info", debugInfo_, "false");

  goalReceived_ = false;
  goalReached_  = false;
  pathComputed_ = false;
  dt_           = 1. / controlRate_;
  std::chrono::milliseconds msdt(int(dt_* 1000));
    
  std::cout << "======= Loading ROS2 parameters =======" << std::endl;
  std::cout << "- Odom frame name : " << odomFrame_  << std::endl;
  std::cout << "- Robot frame name: " << robotFrame_ << std::endl;
  std::cout << "- Map frame name  : " << mapFrame_   << std::endl;
  std::cout << "- Control freq    : " << controlRate_<< std::endl;
  std::cout << "- Delay mode      : " << delayMode_  << std::endl;
  std::cout << "- Debug info      : " << debugInfo_  << std::endl;

  // Create ROS2 Publishers and Subscribers
  tfBuffer_     = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_   = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  pubOdomPath_  = this->create_publisher<nav_msgs::msg::Path>(odomPath_Otn_, 10);
  pubMpcPath_   = this->create_publisher<nav_msgs::msg::Path>(mpcPath_Otn_, 10);
  controlTimer_ = this->create_wall_timer(msdt, std::bind(&MPCRosNode::calculateControl, this));
  glbPathSub_   = this->create_subscription<nav_msgs::msg::Path>(
                    glbPath_Itn_, 10, std::bind(&MPCRosNode::glPathCallback, this, std::placeholders::_1));
  locSub_       = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    loc_Itn_, 10, std::bind(&MPCRosNode::localizationCallback, this, std::placeholders::_1));
  goalSub_      = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    goal_Itn_, 10, std::bind(&MPCRosNode::goalCallback, this, std::placeholders::_1));
  
  // Get MPC solver parameters from yaml
  double _mpcSteps, _refCte, _refVel, _refEtheta, _wCte, _wEtheta, _wVel,
    _wAngvel, _wAccel, _wAngveld, _wAcceld, _maxAngvel, _maxThrottle, _boundValue;
  
  this->get_parameter_or<double>("mpc_steps"       , _mpcSteps   , 20.0  );
  this->get_parameter_or<double>("mpc_ref_cte"     , _refCte     , 0.0   );
  this->get_parameter_or<double>("mpc_ref_vel"     , _refVel     , 1.0   );
  this->get_parameter_or<double>("mpc_ref_etheta"  , _refEtheta  , 0.0   );
  this->get_parameter_or<double>("mpc_w_cte"       , _wCte       , 5000.0);
  this->get_parameter_or<double>("mpc_w_etheta"    , _wEtheta    , 5000.0);
  this->get_parameter_or<double>("mpc_w_vel"       , _wVel       , 1.0   );
  this->get_parameter_or<double>("mpc_w_angvel"    , _wAngvel    , 100.0 );
  this->get_parameter_or<double>("mpc_w_angvel_d"  , _wAngveld   , 10.0  );
  this->get_parameter_or<double>("mpc_w_accel"     , _wAccel     , 50.0  );
  this->get_parameter_or<double>("mpc_w_accel_d"   , _wAcceld    , 10.0  );
  this->get_parameter_or<double>("mpc_max_angvel"  , _maxAngvel  , 3.0   );
  this->get_parameter_or<double>("mpc_max_throttle", _maxThrottle, 1.0   );
  this->get_parameter_or<double>("mpc_bound_value" , _boundValue , 1.0e3 );

  std::cout << "======= Loading MPC parameters =======" << std::endl;
  std::cout << "mpc_steps       : " << _mpcSteps    << std::endl;
  std::cout << "mpc_ref_vel     : " << _refCte      << std::endl;
  std::cout << "mpc_w_cte       : " << _refVel      << std::endl;
  std::cout << "mpc_ref_etheta  : " << _refEtheta   << std::endl;
  std::cout << "mpc_w_cte       : " << _wCte        << std::endl;
  std::cout << "mpc_w_etheta    : " << _wEtheta     << std::endl;
  std::cout << "mpc_w_vel       : " << _wVel        << std::endl;
  std::cout << "mpc_w_angvel    : " << _wAngvel     << std::endl;
  std::cout << "mpc_w_angvel_d  : " << _wAngveld    << std::endl;
  std::cout << "mpc_w_accel     : " << _wAccel      << std::endl;
  std::cout << "mpc_w_accel_d   : " << _wAcceld     << std::endl;
  std::cout << "mpc_max_angvel  : " << _maxAngvel   << std::endl;
  std::cout << "mpc_max_throttle: " << _maxThrottle << std::endl;
  std::cout << "mpc_bound_value : " << _boundValue  << std::endl;

  // Fill MPC solver parameter map and construct object
  _mpc_params["DT"]         = dt_;
  _mpc_params["STEPS"]      = _mpcSteps;
  _mpc_params["REF_CTE"]    = _refCte;
  _mpc_params["REF_ETHETA"] = _refVel;
  _mpc_params["REF_V"]      = _refEtheta;
  _mpc_params["W_CTE"]      = _wCte;
  _mpc_params["W_EPSI"]     = _wEtheta;
  _mpc_params["W_V"]        = _wVel;
  _mpc_params["W_ANGVEL"]   = _wAngvel;
  _mpc_params["W_A"]        = _wAngveld;
  _mpc_params["W_DANGVEL"]  = _wAccel;
  _mpc_params["W_DA"]       = _wAcceld;
  _mpc_params["ANGVEL"]     = _maxAngvel;
  _mpc_params["MAXTHR"]     = _maxThrottle;
  _mpc_params["BOUND"]      = _boundValue;
  auto _mpc = std::make_shared<MPC>(_mpc_params);
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
* @brief Goal topic ROS2 subscription
*        
*/
void MPCRosNode::goalCallback(const geometry_msgs::msg::PoseStamped::ConstPtr& goalMsg)
{
  const auto& gp = goalMsg->pose.position;
  if(goalPoint_ != gp)
  {
    goalPoint_    = gp;
    goalReceived_ = true;
    std::cout << "New Goal Received! " << std::endl;
  }
}

/**
* @brief Localization ROS2 subscription
*        Checks distance to the goal point
*/
void MPCRosNode::localizationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& locMsg)
{
    if(goalReceived_)
    {
        double goalDist = std::hypot(
          goalPoint_.x - locMsg->pose.pose.position.x,
          goalPoint_.y - locMsg->pose.pose.position.y);
        if(goalDist < goalRadius_)
        {
          goalReceived_  = false;
          goalReached_   = true;
          pathComputed_  = false;
          std::cout << "Goal Reached! " << std::endl;
        }
    }
}

/**
* @brief Global path ROS2 subscription
*        Downsamples the global path and converts the frames
*/
void MPCRosNode::glPathCallback(const nav_msgs::msg::Path::ConstPtr& glbPathMsg)
{
  if(goalReceived_ && !goalReached_)
  {
    std::cout << "--PathCB--" << std::endl;
    nav_msgs::msg::Path odomPath;
    geometry_msgs::msg::TransformStamped tf_path2odom;
    const auto pathPoses = glbPathMsg->poses;
    const auto pathFrame = glbPathMsg->header.frame_id;
    const auto pathSize  = pathPoses.size();
    double totalLength  = 0.0;
    double waypointDist = 0.0;
    size_t sample = 0;

    try {
      // Check the latest available transform
      tf_path2odom = tfBuffer_->lookupTransform(odomFrame_, pathFrame, rclcpp::Time(0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(
      this->get_logger(), "Could not transform %s to %s: %s",
        odomFrame_.c_str(), pathFrame.c_str(), ex.what());
      return;
    }

    // Average distance between consecutive waypoints
    double pathLength = 0.0;
    for (size_t i = 0; i < pathSize-1; i++) {
      double dx = pathPoses[i+1].pose.position.x - pathPoses[i].pose.position.x;
      double dy = pathPoses[i+1].pose.position.y - pathPoses[i].pose.position.y;
      pathLength += std::hypot(dx, dy);
    }
    waypointDist = pathLength / (pathSize-1);

    // Calculate downsampling size
    const size_t downSampleSize = static_cast<int>
      (pathLength / (waypointDist * downSampleRate_));
    
    // Transform downsampled path's frame to odom frame
    for (size_t i = 0; i < pathSize; i++)
    {
        if (totalLength > pathLength)
          break;

        if (sample % downSampleSize == 0)
        {
          geometry_msgs::msg::PoseStamped waypPose;
          tf2::doTransform(pathPoses[i], waypPose, tf_path2odom);
          odomPath.poses.push_back(waypPose);
        }
        totalLength += waypointDist;
        sample++;
    }

    // Publish the downsampled path if long enough
    if (odomPath.poses.size() >= 6)
    {
      odomPath.header.frame_id = odomFrame_;
      odomPath.header.stamp = this->get_clock()->now(); // will this match with each pose?
      pubOdomPath_->publish(odomPath);
    }
    else
    {
      std::cout << "Failed to downsample the path" << std::endl;
    }
    
    goalReceived_ = false;
  }
}

/**
* @brief MPC Controller loop
*        ...
*/
void MPCRosNode::calculateControl(void)
{
  geometry_msgs::msg::TransformStamped tfStamped;
  double throttle_ = 0.0;
  double linV_     = 0.0;
  double angularW_ = 0.0;

  // std::string fromFrameRel = "map";
  // std::string fromFrameRel = target_frame_.c_str();
  // std::string toFrameRel = "odom";
  // try {
  //   rclcpp::Time now = this->get_clock()->now();
  //   rclcpp::Time when = now - rclcpp::Duration(5, 0);
  //   tfStamped = tfBuffer_->lookupTransform(
  //     toFrameRel,
  //     now,
  //     fromFrameRel,
  //     when,
  //     "world",
  //     50ms);
  // } catch (const tf2::TransformException & ex) {
  //   RCLCPP_INFO(
  //     this->get_logger(), "Could not transform %s to %s: %s",
  //     toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
  //   return;
  // }

  //   // publish general cmd_vel 
  //   if(_pub_twist_flag)
  //   {
  //     geometry_msgs::msg::Twist twistMsg;
  //     twistMsg.linear.x  = _speed; 
  //     twistMsg.angular.z = _w;
  //     _pub_twist.publish(twistMsg);
  //   }

  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
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