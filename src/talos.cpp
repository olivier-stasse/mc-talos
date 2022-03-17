#include "talos.h"
#include "config.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/constants.h>
#include <RBDyn/parsers/urdf.h>
#include <sch/S_Object/S_Sphere.h>
#include <boost/algorithm/string.hpp>

namespace mc_robots
{

TalosCommonRobotModule::TalosCommonRobotModule() : RobotModule(TALOS_DESCRIPTION_PATH, "talos")
{
  rsdf_dir = path + "/rsdf";
  calib_dir = path + "/calib";
}

void TalosCommonRobotModule::setupCommon()
{
  _grippers = {{"l_gripper", {"gripper_left_joint"}, false}, {"r_gripper", {"gripper_right_joint"}, false}};

  _stance = {
      {"torso_1_joint", {                      0.0}},
      {"torso_2_joint", {                      0.006761}},
      {"head_1_joint", {                       0.0}},
      {"head_2_joint", {                       0.0}},
      {"arm_left_1_joint", {                   0.25847}},
      {"arm_left_2_joint", {                   0.173046}},
      {"arm_left_3_joint", {                   0.0002}},
      {"arm_left_4_joint", {                  -0.525366}},
      {"arm_left_5_joint", {                   0.0}},
      {"arm_left_6_joint", {                   0.0}},
      {"arm_left_7_joint", {                   0.1}},
      {"arm_right_1_joint", {                 -0.25847}},
      {"arm_right_2_joint", {                 -0.173046}},
      {"arm_right_3_joint", {                 -0.0002}},
      {"arm_right_4_joint", {                 -0.525366}},
      {"arm_right_5_joint", {                  0.0}},
      {"arm_right_6_joint", {                  0.0}},
      {"arm_right_7_joint", {                  0.1}},
      {"gripper_left_joint", {                 0.0}},
      {"gripper_left_inner_double_joint", {    0.0}},
      {"gripper_left_fingertip_1_joint", {     0.0}},
      {"gripper_left_fingertip_2_joint", {     0.0}},
      {"gripper_left_motor_single_joint", {    0.0}},
      {"gripper_left_inner_single_joint", {    0.0}},
      {"gripper_left_fingertip_3_joint", {     0.0}},
      {"gripper_right_joint", {                0.0}},
      {"gripper_right_inner_double_joint", {   0.0}},
      {"gripper_right_fingertip_1_joint", {    0.0}},
      {"gripper_right_fingertip_2_joint", {    0.0}},
      {"gripper_right_motor_single_joint", {   0.0}},
      {"gripper_right_inner_single_joint", {   0.0}},
      {"gripper_right_fingertip_3_joint", {    0.0}},
      {"leg_left_1_joint", {                   0.0}},
      {"leg_left_2_joint", {                   0.0}},
      {"leg_left_3_joint", {                  -0.411354}},
      {"leg_left_4_joint", {                   0.859395}},
      {"leg_left_5_joint", {                  -0.448041}},
      {"leg_left_6_joint", {                  -0.001708}},
      {"leg_right_1_joint", {                  0.0}},
      {"leg_right_2_joint", {                  0.0}},
      {"leg_right_3_joint", {                 -0.411354}},
      {"leg_right_4_joint", {                  0.859395}},
      {"leg_right_5_joint", {                 -0.448041}},
      {"leg_right_6_joint", {                 -0.001708}}
  };

  _default_attitude = {1, 0, 0, 0, 0, 0, 1.0};

  // XXX dummy parameters
  _forceSensors.emplace_back("RightFootForceSensor", "leg_right_6_link", sva::PTransformd(Eigen::Vector3d(0, 0, -0.098)));
  _forceSensors.emplace_back("LeftFootForceSensor", "leg_left_6_link", sva::PTransformd(Eigen::Vector3d(0, 0, -0.098)));

  _bodySensors.clear();
  _bodySensors.emplace_back("Accelerometer", "base_link", sva::PTransformd(Eigen::Vector3d(-0.0325, 0, 0.1095)));
  _bodySensors.emplace_back("FloatingBase", "base_link", sva::PTransformd::Identity());

  // Configure the stabilizer.
  _lipmStabilizerConfig.leftFootSurface = "LeftFootCenter";
  _lipmStabilizerConfig.rightFootSurface = "RightFootCenter";
  _lipmStabilizerConfig.torsoBodyName = "torso_2_link";
  _lipmStabilizerConfig.comHeight = 0.879;
  _lipmStabilizerConfig.torsoWeight = 100;
  _lipmStabilizerConfig.comActiveJoints =
  {
    "Root",
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",
    "leg_left_6_joint",
    "leg_right_1_joint",
    "leg_right_2_joint",
    "leg_right_3_joint",
    "leg_right_4_joint",
    "leg_right_5_joint",
    "leg_right_6_joint"
  };
  _lipmStabilizerConfig.torsoPitch = 0;
  _lipmStabilizerConfig.copAdmittance = Eigen::Vector2d{0.03, 0.04};
  _lipmStabilizerConfig.dcmPropGain = 4.0;
  _lipmStabilizerConfig.dcmIntegralGain = 10;
  _lipmStabilizerConfig.dcmDerivGain = 0.2;
  _lipmStabilizerConfig.dcmDerivatorTimeConstant = 5;
  _lipmStabilizerConfig.dcmIntegratorTimeConstant = 10;
}

TalosRobotModule::TalosRobotModule(bool fixed) : TalosCommonRobotModule()
{
  urdf_path = path + "/urdf/talos_full_v2.urdf";
  _real_urdf = urdf_path;
  init(rbd::parsers::from_urdf_file(urdf_path, fixed, {}, true, "base_link"));
  setupCommon();
}

} // namespace mc_robots
