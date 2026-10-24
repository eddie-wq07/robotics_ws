#include <gtest/gtest.h>
#include <limits>
#include "flexiv_hardware/joint_commands.hpp"
using namespace flexiv_hardware;

TEST(JointCommands, PreservesVelocityAndJointMapping) {
  std::vector<double> q(2),v(2);
  ASSERT_TRUE(map_position_command({1.,2.},{.1,-.2},{1,0},true,q,v));
  EXPECT_EQ(q,(std::vector<double>{2.,1.}));
  EXPECT_EQ(v,(std::vector<double>{-.2,.1}));
}
TEST(JointCommands, LegacyPositionDoesNotReplayOldVelocity) {
  std::vector<double> q(2),v{5.,6.};
  ASSERT_TRUE(map_position_command({1.,2.},{},{0,1},false,q,v));
  EXPECT_EQ(v,(std::vector<double>{0.,0.}));
}
TEST(JointCommands, RejectsBadVelocityWithoutPartialUpdate) {
  std::vector<double> q{3.,4.},v{.3,.4};
  EXPECT_FALSE(map_position_command({1.,2.},{.1,std::numeric_limits<double>::quiet_NaN()},
      {0,1},true,q,v));
  EXPECT_EQ(q,(std::vector<double>{3.,4.}));
  EXPECT_EQ(v,(std::vector<double>{.3,.4}));
  EXPECT_FALSE(map_position_command({1.},{.1},{0,1},true,q,v));
}
TEST(JointCommands, RequiresCompleteExclusiveClaims) {
  std::set<std::string> p{"a/position","b/position"},v{"a/velocity","b/velocity"},
      torque{"a/effort","b/effort"},cart=p;
  cart.insert("tcp/x");
  auto both=p;both.insert(v.begin(),v.end());
  EXPECT_EQ(select_arm_mode(both,p,v,torque,cart,true),ArmCommandMode::PositionVelocity);
  EXPECT_EQ(select_arm_mode(p,p,v,torque,cart,true),ArmCommandMode::Position);
  EXPECT_EQ(select_arm_mode(cart,p,v,torque,cart,true),ArmCommandMode::Cartesian);
  both.erase("b/velocity");
  EXPECT_FALSE(select_arm_mode(both,p,v,torque,cart,true));
  cart.insert(v.begin(),v.end());
  EXPECT_FALSE(select_arm_mode(cart,p,v,torque,{"a/position","b/position","tcp/x"},true));
}
