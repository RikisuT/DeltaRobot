/// @file delta_math.hpp
/// @brief Pure C++ math library for delta robot kinematics and trajectory generation.
///
/// This header-only library contains all kinematic math (FK, IK, Jacobian) and
/// trajectory generators with ZERO ROS dependency.  It is used by the kinematics
/// and motion_planner nodes so that the nodes stay thin service/action wrappers.
///
/// Design notes:
///   • All lengths are in millimeters, angles in radians.
///   • The struct DeltaRobotParams holds the geometric constants that define a
///     particular robot instance.  Pass it into the free functions.
///   • Trajectory generators return std::vector<Point3D>.

#ifndef DELTA_MATH_HPP_
#define DELTA_MATH_HPP_

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace delta_math {

// ─── Trigonometric constants ────────────────────────────────────────────────

inline const double kSqrt3  = std::sqrt(3.0);
inline const double kSin120 = kSqrt3 / 2.0;
inline constexpr double kCos120 = -0.5;
inline const double kTan60  = kSqrt3;
inline constexpr double kSin30  = 0.5;
inline const double kTan30  = 1.0 / kSqrt3;

// ─── Lightweight POD types (no ROS dependency) ──────────────────────────────

struct Point3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct JointAngles {
  double theta1 = 0.0;
  double theta2 = 0.0;
  double theta3 = 0.0;
};

struct JointVelocities {
  double theta1_vel = 0.0;
  double theta2_vel = 0.0;
  double theta3_vel = 0.0;
};

struct EEVelocity {
  double x_vel = 0.0;
  double y_vel = 0.0;
  double z_vel = 0.0;
};

/// Geometric parameters that define a specific delta robot instance.
struct DeltaRobotParams {
  double base_side      = 200.0;   ///< SB — base triangle side length [mm]
  double ee_side        = 100.0;   ///< SP — end-effector triangle side [mm]
  double active_link    = 100.0;   ///< AL — active link length [mm]
  double passive_link   = 100.0;   ///< PL — passive link length [mm]
  double passive_width  =  30.0;   ///< PW — passive link width [mm]
  double joint_min      =   0.0;   ///< JMin [rad]
  double joint_max      = M_PI_2;  ///< JMax [rad]
  double max_joint_vel  =   1.0;   ///< [rad/s]

  /// Phi angles for the three kinematic chains [rad]
  double phi[3] = {-M_PI_2, M_PI / 6.0, (5.0 * M_PI) / 6.0};
};

// ─── IK helper (single-chain YZ-plane solution) ────────────────────────────

/// Compute the active-link angle for one kinematic chain projected onto the
/// YZ-plane.  Returns 0 on success, -1 if the point is unreachable.
inline int ik_angle_yz(const DeltaRobotParams& p,
                       double x0, double y0, double z0,
                       double& theta) {
  double y1 = -0.5 * kTan30 * p.base_side;
  y0 -= 0.5 * kTan30 * p.ee_side;

  double a = (x0 * x0 + y0 * y0 + z0 * z0
              + p.active_link * p.active_link
              - p.passive_link * p.passive_link
              - y1 * y1) / (2.0 * z0);
  double b = (y1 - y0) / z0;

  double d = -(a + b * y1) * (a + b * y1)
             + p.active_link * (b * b * p.active_link + p.active_link);
  if (d < 0.0) return -1;

  double yj = (y1 - a * b - std::sqrt(d)) / (b * b + 1.0);
  double zj = a + b * yj;
  theta = std::atan(-zj / (y1 - yj)) + ((yj > y1) ? M_PI : 0.0);
  return 0;
}

// ─── Forward Kinematics ─────────────────────────────────────────────────────

/// Compute the end-effector position from three active-link angles.
/// Returns a fail-safe position (0, 0, -250) if the configuration is invalid.
inline Point3D forward_kinematics(const DeltaRobotParams& p,
                                  double theta1, double theta2, double theta3) {
  double t = (p.base_side - p.ee_side) * kTan30 / 2.0;

  double y1 = -(t + p.active_link * std::cos(theta1));
  double z1 = -p.active_link * std::sin(theta1);

  double y2 = (t + p.active_link * std::cos(theta2)) * kSin30;
  double x2 = y2 * kTan60;
  double z2 = -p.active_link * std::sin(theta2);

  double y3 = (t + p.active_link * std::cos(theta3)) * kSin30;
  double x3 = -y3 * kTan60;
  double z3 = -p.active_link * std::sin(theta3);

  double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  double w1 = y1 * y1 + z1 * z1;
  double w2 = x2 * x2 + y2 * y2 + z2 * z2;
  double w3 = x3 * x3 + y3 * y3 + z3 * z3;

  double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

  double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  double a = a1 * a1 + a2 * a2 + dnm * dnm;
  double b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1
             + dnm * dnm * (z1 * z1 - p.passive_link * p.passive_link);

  double disc = b * b - 4.0 * a * c;
  Point3D result;
  if (disc < 0.0) {
    // Fail-safe default
    result.x = 0.0;
    result.y = 0.0;
    result.z = -250.0;
  } else {
    result.z = -0.5 * (b + std::sqrt(disc)) / a;
    result.x = (a1 * result.z + b1) / dnm;
    result.y = (a2 * result.z + b2) / dnm;
  }
  return result;
}

// ─── Inverse Kinematics ─────────────────────────────────────────────────────

/// Compute joint angles from a Cartesian position.
/// Returns NaN angles if the point is unreachable (check with std::isfinite).
inline JointAngles inverse_kinematics(const DeltaRobotParams& p,
                                      double x, double y, double z) {
  JointAngles j;
  double t1 = 0.0, t2 = 0.0, t3 = 0.0;

  int s = ik_angle_yz(p, x, y, z, t1);
  if (s == 0) {
    s = ik_angle_yz(p,
                    x * kCos120 + y * kSin120,
                    y * kCos120 - x * kSin120,
                    z, t2);
  }
  if (s == 0) {
    s = ik_angle_yz(p,
                    x * kCos120 - y * kSin120,
                    y * kCos120 + x * kSin120,
                    z, t3);
  }

  if (s != 0) {
    j.theta1 = std::numeric_limits<double>::quiet_NaN();
    j.theta2 = std::numeric_limits<double>::quiet_NaN();
    j.theta3 = std::numeric_limits<double>::quiet_NaN();
  } else {
    j.theta1 = t1;
    j.theta2 = t2;
    j.theta3 = t3;
  }
  return j;
}

// ─── Jacobian and velocity mapping ──────────────────────────────────────────

/// Compute the auxiliary (passive-link) angles for all three chains.
/// Returns {theta2_vec, theta3_vec} where each has 3 elements.
inline std::pair<std::vector<double>, std::vector<double>>
calc_aux_angles(const DeltaRobotParams& p,
                double theta1, double theta2, double theta3) {
  Point3D pos = forward_kinematics(p, theta1, theta2, theta3);

  const double UP = (kSqrt3 / 3.0) * p.ee_side;
  const Eigen::Vector3d P = {pos.x, pos.y, pos.z};
  const Eigen::Vector3d D = {UP - p.active_link, 0.0, 0.0};

  Eigen::Matrix3d C;
  for (int i = 0; i < 3; ++i) {
    double phi_i = p.phi[i];
    Eigen::Matrix3d R;
    R << std::cos(phi_i), std::sin(phi_i), 0.0,
        -std::sin(phi_i), std::cos(phi_i), 0.0,
         0.0,             0.0,             1.0;
    C.col(i) = R * P + D;
  }

  double C_x2 = C(0, 1), C_y2 = C(1, 1), C_z2 = C(2, 1);
  double C_x3 = C(0, 2), C_y3 = C(1, 2), C_z3 = C(2, 2);

  double C_sqrd_2 = C_x2 * C_x2 + C_y2 * C_y2 + C_z2 * C_z2;
  double C_sqrd_3 = C_x3 * C_x3 + C_y3 * C_y3 + C_z3 * C_z3;

  double t32 = std::acos(C_y2 / p.passive_link);
  double t33 = std::acos(C_y3 / p.passive_link);

  double t22_num = C_sqrd_2 - p.active_link * p.active_link - p.passive_link * p.passive_link;
  double t22_den = 2.0 * p.active_link * p.passive_link * std::sin(t32);
  double t23_num = C_sqrd_3 - p.active_link * p.active_link - p.passive_link * p.passive_link;
  double t23_den = 2.0 * p.active_link * p.passive_link * std::sin(t33);

  double t22 = std::acos(t22_num / t22_den);
  double t23 = std::acos(t23_num / t23_den);

  return std::make_pair(
    std::vector<double>{theta2, t22, t23},
    std::vector<double>{theta3, t32, t33});
}

/// Compute the full Jacobian matrix J such that theta_dot = J * p_dot.
inline Eigen::Matrix3d calc_jacobian(const DeltaRobotParams& p,
                                     double theta1, double theta2, double theta3) {
  auto aux = calc_aux_angles(p, theta1, theta2, theta3);
  const std::vector<double> t1 = {theta1, theta2, theta3};
  const std::vector<double>& t2 = aux.first;
  const std::vector<double>& t3 = aux.second;

  auto J_ix = [&](int i) -> double {
    return std::sin(t3[i]) * std::cos(t2[i] + t1[i]) * std::cos(p.phi[i])
           + std::cos(t3[i]) * std::sin(p.phi[i]);
  };
  auto J_iy = [&](int i) -> double {
    return -std::sin(t3[i]) * std::cos(t2[i] + t1[i]) * std::sin(p.phi[i])
           + std::cos(t3[i]) * std::cos(p.phi[i]);
  };
  auto J_iz = [&](int i) -> double {
    return std::sin(t3[i]) * std::sin(t2[i] + t1[i]);
  };

  Eigen::Matrix3d Jp;
  for (int i = 0; i < 3; ++i) {
    Jp(i, 0) = J_ix(i);
    Jp(i, 1) = J_iy(i);
    Jp(i, 2) = J_iz(i);
  }

  Eigen::Matrix3d JTheta = Eigen::Matrix3d::Zero();
  JTheta(0, 0) = p.active_link * std::sin(t2[0]) * std::sin(t3[0]);
  JTheta(1, 1) = p.active_link * std::sin(t2[1]) * std::sin(t3[1]);
  JTheta(2, 2) = p.active_link * std::sin(t2[2]) * std::sin(t3[2]);

  return JTheta.inverse() * Jp;
}

/// Map a desired end-effector velocity to joint velocities.
inline JointVelocities calc_theta_dot(const DeltaRobotParams& p,
                                      double theta1, double theta2, double theta3,
                                      double x_dot, double y_dot, double z_dot) {
  Eigen::Matrix3d J = calc_jacobian(p, theta1, theta2, theta3);
  Eigen::Vector3d p_dot(x_dot, y_dot, z_dot);
  Eigen::Vector3d td = J * p_dot;
  return {td(0), td(1), td(2)};
}

/// Compute end-effector velocities via finite differences over a position trajectory.
inline std::vector<EEVelocity> compute_gradient(const std::vector<Point3D>& positions,
                                                double dt) {
  const std::size_t n = positions.size();
  std::vector<EEVelocity> velocities(n, {0.0, 0.0, 0.0});
  if (n <= 1) return velocities;

  // Forward difference for first point
  velocities[0].x_vel = (positions[1].x - positions[0].x) / dt;
  velocities[0].y_vel = (positions[1].y - positions[0].y) / dt;
  velocities[0].z_vel = (positions[1].z - positions[0].z) / dt;

  // Central difference for interior points
  for (std::size_t i = 1; i < n - 1; ++i) {
    velocities[i].x_vel = (positions[i + 1].x - positions[i - 1].x) / (2.0 * dt);
    velocities[i].y_vel = (positions[i + 1].y - positions[i - 1].y) / (2.0 * dt);
    velocities[i].z_vel = (positions[i + 1].z - positions[i - 1].z) / (2.0 * dt);
  }

  // Backward difference for last point
  velocities[n - 1].x_vel = (positions[n - 1].x - positions[n - 2].x) / dt;
  velocities[n - 1].y_vel = (positions[n - 1].y - positions[n - 2].y) / dt;
  velocities[n - 1].z_vel = (positions[n - 1].z - positions[n - 2].z) / dt;

  return velocities;
}

/// Check if a Cartesian point is reachable (IK has a valid solution
/// AND all joint angles are within [joint_min, joint_max]).
inline bool is_reachable(const DeltaRobotParams& params, double x, double y, double z) {
  JointAngles joints = inverse_kinematics(params, x, y, z);
  if (!std::isfinite(joints.theta1) || !std::isfinite(joints.theta2) || !std::isfinite(joints.theta3)) {
    return false;
  }
  
  auto check_bounds = [](double theta, double min_val, double max_val) {
    return (theta >= min_val) && (theta <= max_val);
  };
  
  if (!check_bounds(joints.theta1, params.joint_min, params.joint_max) ||
      !check_bounds(joints.theta2, params.joint_min, params.joint_max) ||
      !check_bounds(joints.theta3, params.joint_min, params.joint_max)) {
    return false;
  }
  return true;
}

// ─── Trajectory generators ──────────────────────────────────────────────────

/// Straight up-and-down oscillation along the Z axis.
inline std::vector<Point3D> straight_up_down_trajectory(
    double center_z = -380.0, double amplitude = 75.0,
    int cycles = 5, int num_points = 300) {
  std::vector<Point3D> traj(num_points);
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    traj[i].x = 0.0;
    traj[i].y = 0.0;
    traj[i].z = center_z + amplitude * std::sin(2.0 * M_PI * cycles * t);
  }
  return traj;
}

/// Pringle-shaped trajectory (circle in XY with sinusoidal Z).
inline std::vector<Point3D> pringle_trajectory(
    double center_z = -380.0, double amplitude = 25.0, int num_points = 200) {
  std::vector<Point3D> traj(num_points);
  double step = (2.0 * M_PI) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    double t = i * step;
    traj[i].x = (2.0 * amplitude) * std::cos(t);
    traj[i].y = (2.0 * amplitude) * std::sin(t);
    traj[i].z = center_z + amplitude * std::sin(2.0 * t);
  }
  return traj;
}

/// Translation along each axis sequentially.
inline std::vector<Point3D> axes_trajectory(
    double x_end = 60.0, double y_end = 60.0,
    double z_start = -380.0, double z_end = -320.0, int num_points_per_seg = 25) {
  std::vector<Point3D> traj;

  auto lerp_seg = [&](double x0, double y0, double z0,
                       double x1, double y1, double z1) {
    for (int i = 0; i < num_points_per_seg; ++i) {
      double t = static_cast<double>(i) / (num_points_per_seg - 1);
      traj.push_back({x0 + t * (x1 - x0),
                       y0 + t * (y1 - y0),
                       z0 + t * (z1 - z0)});
    }
  };

  // X axis
  lerp_seg(0, 0, z_start, x_end, 0, z_start);
  lerp_seg(x_end, 0, z_start, 0, 0, z_start);
  // Y axis
  lerp_seg(0, 0, z_start, 0, y_end, z_start);
  lerp_seg(0, y_end, z_start, 0, 0, z_start);
  // Z axis
  lerp_seg(0, 0, z_start, 0, 0, z_end);
  lerp_seg(0, 0, z_end, 0, 0, z_start);

  return traj;
}

/// Circular trajectory in the XY plane at constant Z.
inline std::vector<Point3D> circle_trajectory(
    double center_z = -380.0, double radius = 40.0, int num_points = 200) {
  std::vector<Point3D> traj(num_points);
  double step = (2.0 * M_PI) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    double t = i * step;
    traj[i].x = radius * std::cos(t);
    traj[i].y = radius * std::sin(t);
    traj[i].z = center_z;
  }
  return traj;
}

/// Read a CSV file with columns X, Y, Z (header row skipped).
inline std::vector<Point3D> read_csv(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) return {};

  std::vector<Point3D> trajectory;
  std::string line;
  bool first_line = true;
  while (std::getline(file, line)) {
    if (first_line) { first_line = false; continue; }
    std::istringstream iss(line);
    std::string val;
    Point3D pt;
    if (!std::getline(iss, val, ',')) continue;
    pt.x = std::stod(val);
    if (!std::getline(iss, val, ',')) continue;
    pt.y = std::stod(val);
    if (!std::getline(iss, val, ',')) continue;
    pt.z = std::stod(val);
    trajectory.push_back(pt);
  }
  return trajectory;
}

/// Randomly sample numPoints from a loaded CSV trajectory.
inline std::vector<Point3D> random_sample_trajectory(
    const std::vector<Point3D>& all_points, int num_points) {
  if (all_points.empty() || num_points <= 0) return {};
  std::vector<Point3D> sampled;
  sampled.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    int idx = std::rand() % static_cast<int>(all_points.size());
    sampled.push_back(all_points[idx]);
  }
  return sampled;
}

}  // namespace delta_math

#endif  // DELTA_MATH_HPP_
