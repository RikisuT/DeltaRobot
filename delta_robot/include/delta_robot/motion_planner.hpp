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
#include "geometry_msgs/msg/point.hpp"

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

class DeltaMotionPlanner : public rclcpp::Node {
public:
  DeltaMotionPlanner();
  ~DeltaMotionPlanner() = default;

private:
  bool playDemo = false;

  rclcpp::Publisher<DeltaJoints>::SharedPtr joint_pub;
  rclcpp::Publisher<DeltaJointVels>::SharedPtr joint_vel_pub;
  rclcpp::Service<PlayDemoTraj>::SharedPtr demo_traj_server;
  rclcpp::Service<MoveToPoint>::SharedPtr move_to_point_server;
  rclcpp::Service<MoveToConfiguration>::SharedPtr move_to_configuration_server;
  rclcpp::Service<MotionDemo>::SharedPtr motion_demo_server;
  rclcpp::Client<ConvertToJointTrajectory>::SharedPtr convert_to_joint_trajectory_client;
  rclcpp::Client<ConvertToJointVelTrajectory>::SharedPtr convert_to_joint_vel_trajectory_client;
  rclcpp::Client<DeltaIK>::SharedPtr delta_ik_client;
  rclcpp::Client<DeltaFK>::SharedPtr delta_fk_client;
  rclcpp::TimerBase::SharedPtr demo_timer;

  void publishMotorCommands(const std::vector<DeltaJoints>& joint_traj, const unsigned int delay_ms = 50);
  void publishMotorVelocityCommands(const std::vector<DeltaJointVels>& joint_vel_traj, const unsigned int delay_ms = 50);

  void moveToPoint(const Point& point);
  void moveToConfiguration(const DeltaJoints& joints);
  void moveThroughPoints(const std::vector<Point>& points);

  void playTrajectory(const std::vector<Point> trajectory);
  void playDemoTrajectory(const std::shared_ptr<PlayDemoTraj::Request> request, std::shared_ptr<PlayDemoTraj::Response> response);

  std::vector<Point> readCSV(const std::string& fileName);

  std::vector<Point> straightUpDownTrajectory();
  std::vector<Point> pringleTrajectory();
  std::vector<Point> axesTrajectory();
  std::vector<Point> circleTrajectory();
  std::vector<Point> scanTrajectory();
  std::vector<Point> randomSampleTrajectory(const int numPoints);
};

#endif // !MOTION_PLANNER_HPP_