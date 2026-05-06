#include <gtest/gtest.h>
#include <cmath>
#include "delta_robot/delta_math.hpp"

using namespace delta_math;

class DeltaMathTest : public ::testing::Test {
protected:
  DeltaRobotParams params;
  
  void SetUp() override {
    // Default params matches typical configuration
    params.base_side = 346.41;
    params.ee_side = 112.58;
    params.active_link = 100.0;
    params.passive_link = 263.0;
    params.joint_min = -0.1745;
    params.joint_max = 1.5708;
  }
};

TEST_F(DeltaMathTest, FK_IK_RoundTrip) {
  // Test a reachable configuration
  double t1 = 0.5;
  double t2 = 0.5;
  double t3 = 0.5;
  
  Point3D p = forward_kinematics(params, t1, t2, t3);
  
  // Basic sanity check: Z should be negative
  EXPECT_LT(p.z, 0.0);
  
  JointAngles joints = inverse_kinematics(params, p.x, p.y, p.z);
  
  EXPECT_NEAR(joints.theta1, t1, 1e-4);
  EXPECT_NEAR(joints.theta2, t2, 1e-4);
  EXPECT_NEAR(joints.theta3, t3, 1e-4);
}

TEST_F(DeltaMathTest, IsReachable_Valid) {
  // Find a valid point using FK
  Point3D p = forward_kinematics(params, 0.2, 0.2, 0.2);
  
  EXPECT_TRUE(is_reachable(params, p.x, p.y, p.z));
}

TEST_F(DeltaMathTest, IsReachable_InvalidBounds) {
  // A point where IK returns valid angles but they are outside [joint_min, joint_max]
  // Provide a point very high up (Z close to 0) which forces negative joint angles
  EXPECT_FALSE(is_reachable(params, 0.0, 0.0, -50.0));
}

TEST_F(DeltaMathTest, IsReachable_Unreachable) {
  // Way out of workspace
  EXPECT_FALSE(is_reachable(params, 0.0, 0.0, -1000.0));
  EXPECT_FALSE(is_reachable(params, 1000.0, 1000.0, -300.0));
}

TEST_F(DeltaMathTest, TrajectoryGenerators) {
  auto traj = circle_trajectory(-300.0, 40.0, 100);
  EXPECT_EQ(traj.size(), 100u);
  
  // All points should have z = -300
  for (const auto& pt : traj) {
    EXPECT_NEAR(pt.z, -300.0, 1e-6);
  }
  
  // First point should be (40, 0)
  EXPECT_NEAR(traj[0].x, 40.0, 1e-6);
  EXPECT_NEAR(traj[0].y, 0.0, 1e-6);
}

TEST_F(DeltaMathTest, Jacobian_Invertible) {
  Eigen::Matrix3d J = calc_jacobian(params, 0.2, 0.2, 0.2);
  
  // Determinant should be non-zero for a valid pose away from singularities
  double det = J.determinant();
  EXPECT_NE(det, 0.0);
  EXPECT_TRUE(std::isfinite(det));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
