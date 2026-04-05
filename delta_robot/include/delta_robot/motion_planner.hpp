#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_
#include "rclcpp/rclcpp.hpp"
#include "deltarobot_interfaces/msg/delta_joints.hpp"
#include "deltarobot_interfaces/srv/play_demo_trajectory.hpp"
#include "deltarobot_interfaces/srv/delta_ik.hpp"
#include "deltarobot_interfaces/srv/delta_fk.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_trajectory.hpp"
#include "deltarobot_interfaces/srv/convert_to_joint_vel_trajectory.hpp"
#include "deltarobot_interfaces/srv/move_to_point.hpp"
#include "deltarobot_interfaces/srv/move_to_configuration.hpp"
#include "deltarobot_interfaces/srv/motion_demo.hpp"
#include "deltarobot_interfaces/srv/play_custom_trajectory.hpp"
#include "deltarobot_interfaces/srv/set_motion_mode.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
using DeltaIK = deltarobot_interfaces::srv::DeltaIK;
using DeltaFK = deltarobot_interfaces::srv::DeltaFK;
using PlayDemoTraj = deltarobot_interfaces::srv::PlayDemoTrajectory;
using ConvertToJointTrajectory = deltarobot_interfaces::srv::ConvertToJointTrajectory;
using ConvertToJointVelTrajectory = deltarobot_interfaces::srv::ConvertToJointVelTrajectory;
using Point = geometry_msgs::msg::Point;
using DeltaJoints = deltarobot_interfaces::msg::DeltaJoints;
using DeltaJointVels = deltarobot_interfaces::msg::DeltaJointVels;
using MoveToPoint = deltarobot_interfaces::srv::MoveToPoint;
using MoveToConfiguration = deltarobot_interfaces::srv::MoveToConfiguration;
using MotionDemo = deltarobot_interfaces::srv::MotionDemo;
using PlayCustomTrajectory = deltarobot_interfaces::srv::PlayCustomTrajectory;
using SetMotionMode = deltarobot_interfaces::srv::SetMotionMode;

class DeltaMotionPlanner : public rclcpp::Node {
public:
  DeltaMotionPlanner();
  ~DeltaMotionPlanner();

  // Operating modes for the motion planner
  enum OperatingMode : uint8_t {
    TASK_MODE = 0,          // Service-based discrete commands
    LIVE_TEACH_MODE = 1     // Topic-based real-time control
  };

private:
  bool playDemo = false;
  bool initialized = false;
  std::size_t demo_sequence_index = 0;
  std::atomic<bool> cancel_current_traj{false};
  std::unique_ptr<std::thread> traj_thread;
  std::atomic<bool> motion_active{false};

  // Motion mode state
  OperatingMode current_mode = TASK_MODE;
  std::mutex mode_mutex;
  
  // Live target buffer (overwrite semantics)
  Point live_target_buffer;
  std::atomic<bool> live_target_updated{false};
  std::mutex live_target_mutex;

  // Parameters
  int param_traj_step_ms = 20;  // Default to 20ms (50Hz)
  int param_live_controller_ms = 20;  // Default to 20ms (50Hz)
  std::vector<std::string> controller_joint_names;
  std::string live_target_topic;

  // Timers
  rclcpp::TimerBase::SharedPtr init_timer;  // ← only once
  rclcpp::TimerBase::SharedPtr demo_timer;
  rclcpp::TimerBase::SharedPtr live_controller_timer;  // Fixed-rate live motion controller

  // Publishers
  rclcpp::Publisher<DeltaJoints>::SharedPtr joint_pub;
  rclcpp::Publisher<DeltaJointVels>::SharedPtr joint_vel_pub;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub;
  rclcpp::Subscription<Point>::SharedPtr live_target_sub;

  // Servers
  rclcpp::Service<PlayDemoTraj>::SharedPtr demo_traj_server;
  rclcpp::Service<MoveToPoint>::SharedPtr move_to_point_server;
  rclcpp::Service<MoveToConfiguration>::SharedPtr move_to_configuration_server;
  rclcpp::Service<MotionDemo>::SharedPtr motion_demo_server;
  rclcpp::Service<PlayCustomTrajectory>::SharedPtr play_custom_trajectory_server;
  rclcpp::Service<SetMotionMode>::SharedPtr set_motion_mode_server;

  // Clients
  rclcpp::Client<ConvertToJointTrajectory>::SharedPtr convert_to_joint_trajectory_client;
  rclcpp::Client<ConvertToJointVelTrajectory>::SharedPtr convert_to_joint_vel_trajectory_client;
  rclcpp::Client<DeltaIK>::SharedPtr delta_ik_client;
  rclcpp::Client<DeltaFK>::SharedPtr delta_fk_client;

  void publishMotorCommands(const std::vector<DeltaJoints>& joint_traj, const unsigned int delay_ms = 50);
  void publishMotorVelocityCommands(const std::vector<DeltaJointVels>& joint_vel_traj, const unsigned int delay_ms = 50);

  bool tryAcquireMotionSlot(const char* action_name);
  void releaseMotionSlot();

  bool moveToPoint(const Point& point, bool require_task_mode = true);
  bool moveToConfiguration(const DeltaJoints& joints, bool require_task_mode = true);
  void moveThroughPoints(const std::vector<Point>& points);
  void liveTargetCallback(const Point::SharedPtr msg);
  void liveMotionController();  // Fixed-rate controller for live teach mode
  
  void setMotionMode(
    const std::shared_ptr<SetMotionMode::Request> request,
    std::shared_ptr<SetMotionMode::Response> response);

  bool playTrajectory(const std::vector<Point>& trajectory);
  void playCustomTrajectory(
    const std::shared_ptr<PlayCustomTrajectory::Request> request,
    std::shared_ptr<PlayCustomTrajectory::Response> response);
  void playDemoTrajectory(const std::shared_ptr<PlayDemoTraj::Request> request, std::shared_ptr<PlayDemoTraj::Response> response);

  std::vector<Point> readCSV(const std::string& fileName);

  std::vector<Point> straightUpDownTrajectory();
  std::vector<Point> pringleTrajectory();
  std::vector<Point> axesTrajectory();
  std::vector<Point> circleTrajectory();
  std::vector<Point> scanTrajectory();
  std::vector<Point> randomSampleTrajectory(const int numPoints);
};

#endif  // MOTION_PLANNER_HPP_