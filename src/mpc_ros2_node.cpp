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
#include <tf2/LinearMath/Quaternion.h>
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
    void glPathCallback(const std::shared_ptr<const nav_msgs::msg::Path>& glbPathMsg);
    void goalCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& goalMsg);
    void localizationCallback(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& locMsg);
    void calculateControl(void);
    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xMat, Eigen::VectorXd yMat, int order);

  private:
    // ROS2 node parameters
    size_t downSampleRate_;
    double dt_, maxSpeed_, controlRate_, pathLength_, goalRadius_, waypointsDist_;
    bool goalReceived_, goalReached_, pathComputed_, debugInfo_, delayMode_, twist_en_;
    std::string odomPath_Otn_, mpcPath_Otn_, twist_Otn_, glbPath_Itn_, loc_Itn_, goal_Itn_;
    std::string mapFrame_, odomFrame_, robotFrame_;
    geometry_msgs::msg::Pose odom_;
    geometry_msgs::msg::Point goalPoint_;
    nav_msgs::msg::Path globalPath_;
    rclcpp::TimerBase::SharedPtr controlTimer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubTwist_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMpcPath_{nullptr}, pubOdomPath_{nullptr};
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr glbPathSub_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr locSub_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_{nullptr};

    // MPC solver parameters
    MPC _mpc;
};

/**
* @brief MPC Node class constructor
*        ...
*/
MPCRosNode::MPCRosNode(const std::string& nodeName, const rclcpp::NodeOptions& options)
  : Node(nodeName, options)
{
  // Load ROS2 parameters from yaml
  this->get_parameter_or<std::string>("loc_topic", loc_Itn_, "/amcl");
  this->get_parameter_or<std::string>("goal_topic", goal_Itn_, "/goal");
  this->get_parameter_or<std::string>("global_path_topic", glbPath_Itn_, "/global_plan");
  this->get_parameter_or<std::string>("odom_path_topic", odomPath_Otn_, "odom_path");
  this->get_parameter_or<std::string>("mpc_path_topic", mpcPath_Otn_, "mpc_path");
  this->get_parameter_or<std::string>("twist_topic", twist_Otn_, "cmd_vel");
  this->get_parameter_or<std::string>("robot_frame", robotFrame_, "base_link");
  this->get_parameter_or<std::string>("odom_frame", odomFrame_, "odom");
  this->get_parameter_or<std::string>("map_frame", mapFrame_, "map");
  this->get_parameter_or<double>("max_speed", maxSpeed_ , .5);
  this->get_parameter_or<double>("path_length", pathLength_ , 8.);
  this->get_parameter_or<double>("goal_radius", goalRadius_ , .5);
  this->get_parameter_or<double>("control_rate", controlRate_, 10.);
  this->get_parameter_or<size_t>("downsample_rate", downSampleRate_, 10);
  this->get_parameter_or<bool>("delay_mode", delayMode_, "true");
  this->get_parameter_or<bool>("debug_info", debugInfo_, "true");
  this->get_parameter_or<bool>("publish_twist_flag", twist_en_, "true");

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
  if(twist_en_)
    pubTwist_  = this->create_publisher<geometry_msgs::msg::Twist>(twist_Otn_, 10);

  controlTimer_ = this->create_wall_timer(msdt, std::bind(&MPCRosNode::calculateControl, this));
  glbPathSub_   = this->create_subscription<nav_msgs::msg::Path>(
                    glbPath_Itn_, 10, std::bind(&MPCRosNode::glPathCallback, this, std::placeholders::_1));
  locSub_       = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    loc_Itn_, 10, std::bind(&MPCRosNode::localizationCallback, this, std::placeholders::_1));
  goalSub_      = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    goal_Itn_, 10, std::bind(&MPCRosNode::goalCallback, this, std::placeholders::_1));
  
  // Load MPC solver parameters from yaml
  std::map<std::string, double> mpc_params;
  double mpcSteps, refCte, refVel, refEtheta, wCte, wEtheta, wVel,
    wAngvel, wAccel, wAngveld, wAcceld, maxAngvel, maxAccel, boundValue;
  
  this->get_parameter_or<double>("mpc_steps"       , mpcSteps   , 20.0  );
  this->get_parameter_or<double>("mpc_ref_cte"     , refCte     , 0.0   );
  this->get_parameter_or<double>("mpc_ref_vel"     , refVel     , 1.0   );
  this->get_parameter_or<double>("mpc_ref_etheta"  , refEtheta  , 0.0   );
  this->get_parameter_or<double>("mpc_w_cte"       , wCte       , 5000.0);
  this->get_parameter_or<double>("mpc_w_etheta"    , wEtheta    , 5000.0);
  this->get_parameter_or<double>("mpc_w_vel"       , wVel       , 1.0   );
  this->get_parameter_or<double>("mpc_w_angvel"    , wAngvel    , 100.0 );
  this->get_parameter_or<double>("mpc_w_angvel_d"  , wAngveld   , 10.0  );
  this->get_parameter_or<double>("mpc_w_accel"     , wAccel     , 50.0  );
  this->get_parameter_or<double>("mpc_w_accel_d"   , wAcceld    , 10.0  );
  this->get_parameter_or<double>("mpc_max_angvel"  , maxAngvel  , 3.0   );
  this->get_parameter_or<double>("mpc_max_throttle", maxAccel   , 1.0   );
  this->get_parameter_or<double>("mpc_bound_value" , boundValue , 1.0e3 );

  std::cout << "======= Loading MPC parameters =======" << std::endl;
  std::cout << "mpc_steps      : " << mpcSteps   << std::endl;
  std::cout << "mpc_ref_vel    : " << refCte     << std::endl;
  std::cout << "mpc_w_cte      : " << refVel     << std::endl;
  std::cout << "mpc_ref_etheta : " << refEtheta  << std::endl;
  std::cout << "mpc_w_cte      : " << wCte       << std::endl;
  std::cout << "mpc_w_etheta   : " << wEtheta    << std::endl;
  std::cout << "mpc_w_vel      : " << wVel       << std::endl;
  std::cout << "mpc_w_angvel   : " << wAngvel    << std::endl;
  std::cout << "mpc_w_angvel_d : " << wAngveld   << std::endl;
  std::cout << "mpc_w_accel    : " << wAccel     << std::endl;
  std::cout << "mpc_w_accel_d  : " << wAcceld    << std::endl;
  std::cout << "mpc_max_angvel : " << maxAngvel  << std::endl;
  std::cout << "mpc_max_accel  : " << maxAccel   << std::endl;
  std::cout << "mpc_bound_value: " << boundValue << std::endl;

  // Fill MPC solver parameter map and construct the object
  mpc_params["DT"]         = dt_;
  mpc_params["STEPS"]      = mpcSteps;
  mpc_params["REF_CTE"]    = refCte;
  mpc_params["REF_ETHETA"] = refVel;
  mpc_params["REF_V"]      = refEtheta;
  mpc_params["W_CTE"]      = wCte;
  mpc_params["W_EPSI"]     = wEtheta;
  mpc_params["W_V"]        = wVel;
  mpc_params["W_ANGVEL"]   = wAngvel;
  mpc_params["W_ACC"]      = wAccel;
  mpc_params["W_ANGVELD"]  = wAngveld;
  mpc_params["W_ACCD"]     = wAcceld;
  mpc_params["ANGVEL"]     = maxAngvel;
  mpc_params["MAXACC"]     = maxAccel;
  mpc_params["BOUNDV"]     = boundValue;
  auto _mpc = std::make_shared<MPC>(mpc_params);
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
*        Updates when new goal received
*/
void MPCRosNode::goalCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& goalMsg)
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
*        Updates odometry
*        Checks distance to the goal point
*/
void MPCRosNode::localizationCallback(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& locMsg)
{
  odom_ = locMsg->pose.pose;
  odomFrame_ = locMsg->header.frame_id;

  if(goalReceived_)
  {
    double goalDist = std::hypot(
      goalPoint_.x - locMsg->pose.pose.position.x,
      goalPoint_.y - locMsg->pose.pose.position.y);
    if(goalDist < goalRadius_)
    {
      goalReached_   = true;
      goalReceived_  = false;
      pathComputed_  = false;
      std::cout << "Goal Reached! " << std::endl;
    }
  }
}

/**
* @brief Global path ROS2 subscription
*        Downsamples the global path and converts the frames
*/
void MPCRosNode::glPathCallback(const std::shared_ptr<const nav_msgs::msg::Path>& glbPathMsg)
{
  if(goalReceived_ && !goalReached_)
  {
    std::cout << "--PathCB--" << std::endl;
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
          globalPath_.poses.push_back(waypPose);
        }
        totalLength += waypointDist;
        sample++;
    }

    // Publish the downsampled path if long enough
    if (globalPath_.poses.size() >= 6)
    {
      globalPath_.header.stamp = this->get_clock()->now(); // will this match with each pose?
      pubOdomPath_->publish(globalPath_);
    }
    else
    {
      std::cout << "Failed to downsample the path" << std::endl;
    }
    
    goalReceived_ = false;
  }
}

/**
* @brief Controller - local planner
*        Closed loop nonlinear MPC
*        Publishes predicted MPC trajectory and control signals
*/
void MPCRosNode::calculateControl(void)
{
  double throttle = 0.0;
  double linV     = 0.0;
  double angularW = 0.0;

  if (goalReceived_ && !goalReached_ && pathComputed_)
  {
    tf2::Transform tfPose;
    tf2::Quaternion quat;
    tf2::fromMsg(odom_, tfPose);
    tf2::fromMsg(odom_.orientation, quat);

    double roll, pitch, yaw;
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);

    const double cosTheta = std::cos(yaw);
    const double sinTheta = std::sin(yaw);

    const auto N = globalPath_.poses.size();
    Eigen::VectorXd x_Robot(N);
    Eigen::VectorXd y_Robot(N);

    for(size_t i = 0; i < N; i++)
    {
      const double dx = globalPath_.poses[i].pose.position.x - odom_.position.x;
      const double dy = globalPath_.poses[i].pose.position.y - odom_.position.y;
      x_Robot[i] = dx * cosTheta + dy * sinTheta;
      y_Robot[i] = dy * cosTheta - dx * sinTheta;
    }
    // Fit waypoints and evaluate the polynom
    const auto coeffs = polyfit(x_Robot, y_Robot, 3);
    const double cte  = polyeval(coeffs, 0.0);
    const double eTheta = std::atan(coeffs[1]);

    Eigen::VectorXd state(6);
    if(delayMode_)
    {
      // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
      const double px_act = linV * dt_;
      const double py_act = 0;
      const double theta_act = angularW * dt_; //(steering) theta_act = v * steering * dt / Lf;
      const double v_act = linV + throttle * dt_; //v = v + a * dt
      
      const double cte_act = cte + linV * std::sin(eTheta) * dt_;
      const double etheta_act = eTheta - theta_act;  
      
      state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
    }
    else
    {
      state << 0, 0, 0, linV, cte, eTheta;
    }

    /*=============================================================
    *                 Solve MPC problem
    * =============================================================
    */
    std::vector<double> speedsVec(2); // Output speed vector 
    _mpc.solve(state, speedsVec, coeffs);
    angularW = speedsVec[0];  // Angular velocity(rad/s)
    throttle = speedsVec[1];  // Acceleration
    linV += throttle * dt_;   // Linear velocity
    if (linV >= maxSpeed_)
      linV = maxSpeed_;
    if(linV <= 0.0)
      linV = 0.0;

    // Visualize output if neccessary
    if(debugInfo_)
    {
      std::cout << "\n\n===== DEBUG ===== "<< std::endl;
      // std::cout << "theta: "      << theta << std::endl;
      // std::cout << "V: "          << v << std::endl;
      std::cout << "coeffs: \n"   << coeffs << std::endl;
      std::cout << "AngW: \n"     << angularW << std::endl;
      std::cout << "Throttle: \n" << throttle << std::endl;
      std::cout << "LinV: \n"     << linV << std::endl;
    }
    nav_msgs::msg::Path mpcTraj;
    mpcTraj.header.frame_id = robotFrame_;
    mpcTraj.header.stamp    = this->get_clock()->now();
    for(size_t i=0; i<_mpc._mpcPx.size(); i++)
    {
      geometry_msgs::msg::PoseStamped mpcPose;
      mpcPose.header = mpcTraj.header;
      mpcPose.pose.position.x = _mpc._mpcPx[i];
      mpcPose.pose.position.y = _mpc._mpcPy[i];
      mpcPose.pose.orientation.w = 1.0;
      mpcTraj.poses.push_back(mpcPose);
    }
    pubMpcPath_->publish(mpcTraj);
  }
  else{
    throttle = .0;
    angularW = .0;
    linV     = .0;
  }
  
  if(twist_en_)
  {
    geometry_msgs::msg::Twist twistMsg;
    twistMsg.linear.x  = linV; 
    twistMsg.angular.z = angularW;
    pubTwist_->publish(twistMsg);
  }
}

/**
* @brief Evaluate a polynomial
*
*/
double MPCRosNode::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

/**
* @brief Fit a polynomial
*
*/
Eigen::VectorXd MPCRosNode::polyfit(Eigen::VectorXd xMat, Eigen::VectorXd yMat, int order) 
{
    assert(xMat.size() == yMat.size());
    assert(order >= 1 && order <= xMat.size() - 1);
    Eigen::MatrixXd A(xMat.size(), order + 1);

    for (int i = 0; i < xMat.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xMat.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xMat(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yMat);
    return result;
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