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
#include <std_msgs/msg/string.hpp>
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
#include <nav_msgs/srv/get_plan.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
    void localizationCallback(const std::shared_ptr
      <const geometry_msgs::msg::PoseWithCovarianceStamped>& locMsg);
    void requestGlobalPath(void);
    nav_msgs::msg::Path downsamplePath(const nav_msgs::msg::Path& globPathResponse);
    void calculateControl(void);
    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xMat, Eigen::VectorXd yMat, int order);

    // void sendLocalTrajCallback(
    //   const std::shared_ptr<mpc_ros2::srv::MpcTrajectory::Request> request,
    //   std::shared_ptr<mpc_ros2::srv::MpcTrajectory::Response> response);
    // void sendControlCmdCallback(
    //   const std::shared_ptr<mpc_ros2::srv::MpcControlCommands::Request> request,
    //   std::shared_ptr<mpc_ros2::srv::MpcControlCommands::Response> response);

  private:
    // ROS2 node parameters
    size_t downSampleRate_;
    double dt_, maxSpeed_, controlRate_, pathLength_, goalRadius_, waypointsDist_;
    bool isGoalReached_, debugInfo_, delayMode_;
    std::string mpcPath_Otn_, mpcCmd_Otn_, loc_Itn_, glbPath_Clin_;
    std::string mapFrame_, odomFrame_, robotFrame_;
    geometry_msgs::msg::Pose odom_;
    geometry_msgs::msg::Twist mpcCommands_;
    nav_msgs::msg::Path odomGlobalPath_, mpcTraj_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr locSub_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubTwist_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMpcPath_{nullptr};
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr globalPathClient_;
    rclcpp::TimerBase::SharedPtr controlTimer_{nullptr};
    // rclcpp::Client<nav_msgs::srv::GetRobotPose>::SharedPtr robotPoseClient_;
    // rclcpp::Service<mpc_ros2::srv::MpcTrajectory>::SharedPtr localTrajectoryServer_;
    // rclcpp::Service<mpc_ros2::srv::MpcControlCommands>::SharedPtr controlCommandsServer_;
    
    // MPC object ptr
    std::unique_ptr<MPC> _mpc;
};

/**
* @brief MPC Node class constructor
*        Initializes the ROS2 node and the MPC solver object
*/
MPCRosNode::MPCRosNode(const std::string& nodeName, const rclcpp::NodeOptions& options)
  : Node(nodeName, options), isGoalReached_(false)
{
  // Load ROS2 parameters from yaml
  this->get_parameter_or<std::string>("localization_topic", loc_Itn_, "/amcl_pose");
  this->get_parameter_or<std::string>("global_path_client", glbPath_Clin_, "/planner_server/get_plan");
  this->get_parameter_or<std::string>("mpc_path_topic", mpcPath_Otn_, "/mpc_path");
  this->get_parameter_or<std::string>("twist_topic", mpcCmd_Otn_, "/cmd_vel");
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

  std::cout << "======= Loading ROS2 parameters =======" << std::endl;
  std::cout << "- Odom frame name : " << odomFrame_  << std::endl;
  std::cout << "- Robot frame name: " << robotFrame_ << std::endl;
  std::cout << "- Map frame name  : " << mapFrame_   << std::endl;
  std::cout << "- Control freq    : " << controlRate_<< std::endl;
  std::cout << "- Delay mode      : " << delayMode_  << std::endl;
  std::cout << "- Debug info      : " << debugInfo_  << std::endl;

  // Create ROS2 services and clients
  tfBuffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  pubMpcPath_ = this->create_publisher<nav_msgs::msg::Path>(mpcPath_Otn_, 10);
  pubTwist_   = this->create_publisher<geometry_msgs::msg::Twist>(mpcCmd_Otn_, 10);
  locSub_     = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                loc_Itn_, 10, std::bind(&MPCRosNode::localizationCallback, this, std::placeholders::_1));
  globalPathClient_ = this->create_client<nav_msgs::srv::GetPlan>(glbPath_Clin_);
  // localTrajectoryServer_ = this->create_service<mpc_ros2::srv::MpcTrajectory>(localTraj_Srvn_, 
  //   std::bind(&MPCRosNode::sendLocalTrajCallback, this, std::placeholders::_1, std::placeholders::_2));
  // controlCommandsServer_ = this->create_service<mpc_ros2::srv::MpcControlCommands>(controller_Srvn_, 
  //   std::bind(&MPCRosNode::sendControlCmdCallback, this, std::placeholders::_1, std::placeholders::_2));

  dt_ = 1. / controlRate_;
  std::chrono::milliseconds msdt(int(dt_* 1000));
  controlTimer_ = this->create_wall_timer(msdt, std::bind(&MPCRosNode::calculateControl, this));

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
  _mpc = std::make_unique<MPC>(mpc_params);
}

/**
* @brief Node management interface
*        Essential for multithreaded execution
*/
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MPCRosNode::getNodeBaseInterface()
{
  return this->get_node_base_interface();
}


/**
* @brief MPC trajectory service callback
*        Publishes the MPC trajectory
*/
// void MPCRosNode::void MPCRosNode::sendLocalTrajCallback(
//   const std::shared_ptr<mpc_ros2::srv::MpcTrajectory::Request>  request,
//         std::shared_ptr<mpc_ros2::srv::MpcTrajectory::Response> response)
// {
//   local_trajectory_request->success = true;
// }

/**
* @brief MPC control commands service callback
*        Publishes the control commands
*/
// void MPCRosNode::sendControlCmdCallback(
//   const std::shared_ptr<mpc_ros2::srv::MpcControlCommands::Request>  request,
//         std::shared_ptr<mpc_ros2::srv::MpcControlCommands::Response> response)
// {
//   response->success = true;
// }

/**
* @brief Localization service request
*        Requests the robot pose from the localization node
*/
// void MPCRosNode::requestRobotPose()
// {
//   auto request = std::make_shared<nav2_msgs::srv::GetRobotPose::Request>();
//   auto future = robotPoseClient_->async_send_request(request);

//   if (rclcpp::spin_until_future_complete(
//     this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) 
//   {
//     auto response = future.get();
//     odom_ = response->pose.pose;
//     odomFrame_ = response->pose.header.frame_id; //maybe?

//     // TBD
//     double pathEnd = odomGlobalPath_.poses.end();
//     double goalDist = std::hypot(
//       pathEnd.x - response->pose.pose.position.x,
//       pathEnd.y - response->pose.pose.position.y);
//     if(goalDist < goalRadius_)
//     {
//       isGoalReached_  = true;
//       std::cout << "Goal Reached! " << std::endl;
//     }
//   }
// }

/**
* @brief Localization ROS2 subscription
*        Updates odometry
*        Checks distance to the goal point
*/
void MPCRosNode::localizationCallback(
  const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& locMsg)
{
  odom_      = locMsg->pose.pose;
  odomFrame_ = locMsg->header.frame_id;

  const auto pathEnd = odomGlobalPath_.poses.end();
  double goalDist = std::hypot(
    pathEnd->pose.position.x - locMsg->pose.pose.position.x,
    pathEnd->pose.position.y - locMsg->pose.pose.position.y);
  if(goalDist < goalRadius_)
  {
    isGoalReached_ = true;
    std::cout << "Goal Reached! " << std::endl;
  }
}

/**
* @brief Global path service request
*        Requests the global path from the global planner node
*/
void MPCRosNode::requestGlobalPath(void)
{
  if (!globalPathClient_->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(this->get_logger(), "Global planner service not available.");
    return;
  }
  auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
  auto future = globalPathClient_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) 
  {
    auto response = future.get();
    std::cout << "--PathReq--" << std::endl;
    odomGlobalPath_ = downsamplePath(response->plan);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to receive global path from the global planner.");
  }
}

/**
* @brief MPC gloabl path downsampling
*        Downsamples the global path to reduce the computational load
*        Transforms the global path to the odometry frame
*        Parameters: downSampleRate_, odomFrame_
*/
nav_msgs::msg::Path MPCRosNode::downsamplePath(const nav_msgs::msg::Path& globPathResponse)
{
  const auto pathPoses = globPathResponse.poses;
  const auto pathFrame = globPathResponse.header.frame_id;
  const auto pathSize  = pathPoses.size();
  nav_msgs::msg::Path odomPath;

  try {
    // Check the latest available transform
    geometry_msgs::msg::TransformStamped tf_path2odom;
    tf_path2odom = tfBuffer_->lookupTransform(odomFrame_, pathFrame, rclcpp::Time(0));

    double totalLength  = 0.0;
    double waypointDist = 0.0;
    double pathLength   = 0.0;
    size_t sample = 0;

    // Average distance between consecutive waypoints
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

    // Check the downsampled path if it has enough waypoints
    if (!odomPath.poses.size())
      std::cout << "Failed to downsample the path" << std::endl;
  }
  catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(
    this->get_logger(), "Could not transform %s to %s: %s",
      odomFrame_.c_str(), pathFrame.c_str(), ex.what());
  }

  odomPath.header.stamp = this->get_clock()->now();
  return odomPath;
}

/**
* @brief Nonlinear MPC Controller
*        Publishes predicted MPC trajectory and control commands
*/
void MPCRosNode::calculateControl(void)
{
  if (!isGoalReached_)
  {
    if (!odomGlobalPath_.poses.size())
    {
      if(debugInfo_)
      {
        std::cout << "No global path received!" << std::endl;
      }
      return;
    }
    else if (odomGlobalPath_.poses.size() < 5)
    {
      if(debugInfo_)
      {
        std::cout << "Downsampled path is too short!" << std::endl;
      }
      return;
    }

    if (!odom_.position.x)
    {
      if(debugInfo_)
      {
        std::cout << "No robot pose received!" << std::endl;
      }
      return;
    }

    tf2::Transform tfPose;
    tf2::Quaternion quat;
    tf2::fromMsg(odom_, tfPose);
    tf2::fromMsg(odom_.orientation, quat);

    double roll, pitch, yaw;
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);

    const double cosTheta = std::cos(yaw);
    const double sinTheta = std::sin(yaw);

    const auto N = odomGlobalPath_.poses.size();
    Eigen::VectorXd x_Robot(N);
    Eigen::VectorXd y_Robot(N);

    for(size_t i = 0; i < N; i++)
    {
      const double dx = odomGlobalPath_.poses[i].pose.position.x - odom_.position.x;
      const double dy = odomGlobalPath_.poses[i].pose.position.y - odom_.position.y;
      x_Robot[i] = dx * cosTheta + dy * sinTheta;
      y_Robot[i] = dy * cosTheta - dx * sinTheta;
    }
    // Fit waypoints and evaluate the polynom
    const auto coeffs = polyfit(x_Robot, y_Robot, 3);
    const double cte  = polyeval(coeffs, 0.0);
    const double eTheta = std::atan(coeffs[1]);

    double throttle = 0.0;
    double linVel   = 0.0;
    double angVel   = 0.0;
    Eigen::VectorXd state(6);
    if(delayMode_)
    {
      // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
      const double px_act = linVel * dt_;
      const double py_act = 0;
      const double theta_act = angVel * dt_; //(steering) theta_act = v * steering * dt / Lf;
      const double v_act = linVel + throttle * dt_; //v = v + a * dt
      
      const double cte_act = cte + linVel * std::sin(eTheta) * dt_;
      const double etheta_act = eTheta - theta_act;  
      
      state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
    }
    else
    {
      state << 0, 0, 0, linVel, cte, eTheta;
    }

    /*=============================================================
    *                 Solve MPC problem
    * =============================================================
    */
    auto [mpcTraj, mpcCommands] = _mpc->solve(state, coeffs);

    if (mpcTraj.empty() || mpcCommands.empty()) {
      return;
    }
    else{
      angVel   = mpcCommands[0]; // Angular velocity(rad/s)
      throttle = mpcCommands[1]; // Acceleration
      linVel  += throttle * dt_; // Linear velocity
      if (linVel >= maxSpeed_)
        linVel = maxSpeed_;
      if(linVel <= 0.0)
        linVel = 0.0;
      
      mpcCommands_.linear.x  = linVel;
      mpcCommands_.angular.z = angVel;

      mpcTraj_.header.frame_id = robotFrame_;
      mpcTraj_.header.stamp    = this->get_clock()->now();
      for(const auto& [x, y] : mpcTraj)
      {
        geometry_msgs::msg::PoseStamped mpcPose;
        mpcPose.header = mpcTraj_.header;
        mpcPose.pose.position.x = x;
        mpcPose.pose.position.y = y;
        mpcPose.pose.orientation.w = 1.0;
        mpcTraj_.poses.push_back(mpcPose);
      }

      if(debugInfo_)
      {
        std::cout << "\n\n===== DEBUG ===== "   << std::endl;
        std::cout << "coeffs: \n"   << coeffs   << std::endl;
        std::cout << "AngW: \n"     << angVel   << std::endl;
        std::cout << "Throttle: \n" << throttle << std::endl;
        std::cout << "LinVel: \n"   << linVel     << std::endl;
      }
    }
  }
  else{
    mpcCommands_.linear.x  = .0;
    mpcCommands_.angular.z = .0;
  }
  pubMpcPath_->publish(mpcTraj_);
  pubTwist_->publish(mpcCommands_);

  // Prepare service responses
  // auto local_trajectory_request = std::make_shared<mpc_ros2::srv::MpcTrajectory::Request>();
  // local_trajectory_request->trajectory = mpcTraj_;
  // localTrajectoryServer_->async_send_request(local_trajectory_request);
  // auto control_commands_request = std::make_shared<mpc_ros2::srv::MpcControlCommands::Request>();
  // control_commands_request->commands = mpcCommands_;
  // controlCommandsServer_->async_send_request(control_commands_request);
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

    mpc_ros2_node->requestGlobalPath();
    // mpc_ros2_node->requestRobotPose();

    executor.spin();
    executor.remove_node(mpc_ros2_node->getNodeBaseInterface());
  });

  mpc_ros2_node->calculateControl();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}