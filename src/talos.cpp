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
  //rsdf_dir = path + "/rsdf";
	urdf_path = TALOS_URDF_PATH;
	_real_urdf = urdf_path;
	rsdf_dir = TALOS_RSDF_DIR;
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

  auto fileByBodyName = stdCollisionsFiles(mb);
  initConvexHull(fileByBodyName);
  
  // Force Sensor attachments
  _forceSensors.emplace_back("RightFootForceSensor", "leg_right_6_link", sva::PTransformd(Eigen::Vector3d(0, 0, -0.098)));
  _forceSensors.emplace_back("LeftFootForceSensor", "leg_left_6_link", sva::PTransformd(Eigen::Vector3d(0, 0, -0.098)));
  _forceSensors.emplace_back("RightHandForceSensor", "arm_right_7_link", sva::PTransformd(Eigen::Vector3d(0, 0, -0.14)));
  _forceSensors.emplace_back("LeftHandForceSensor", "arm_left_7_link", sva::PTransformd(Eigen::Vector3d(0, 0, -0.14)));

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

	_ref_joint_order ={"arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint","arm_left_5_joint", "arm_left_6_joint", 
				             "arm_left_7_joint", "arm_right_1_joint","arm_right_2_joint","arm_right_3_joint","arm_right_4_joint",
				             "arm_right_5_joint","arm_right_6_joint", "arm_right_7_joint","gripper_left_joint", "gripper_right_joint","head_1_joint", "head_2_joint",
				             "leg_left_1_joint",  "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint", 
										 "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint","leg_right_6_joint",
										 "torso_1_joint", "torso_2_joint"
				 						 };
}

TalosRobotModule::TalosRobotModule(bool fixed) : TalosCommonRobotModule()
{
  //urdf_path = path + "/urdf/talos_full_v2.urdf";
  //_real_urdf = urdf_path;
  init(rbd::parsers::from_urdf_file(urdf_path, fixed, {}, true, "base_link"));
  setupCommon();
}

void TalosCommonRobotModule::initConvexHull(
    const std::map<std::string, std::pair<std::string, std::string>> & files)
{
  std::string convexPath = path + "/convex/";
  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & f : files)
  {
    _convexHull[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
  }
}

std::map<std::string, std::pair<std::string, std::string>> TalosCommonRobotModule::stdCollisionsFiles(
    const rbd::MultiBody & mb) const
{
  // Add Bodies
  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & b : mb.bodies())
  {
    // Filter out virtual links without convex files
    if(std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
    {
			auto fname = boost::algorithm::replace_first_copy(b.name(), "_link", "");
			fname = boost::algorithm::replace_first_copy(fname, "left_", "");
			fname = boost::algorithm::replace_first_copy(fname, "right_", "");
      res[b.name()] = {b.name(), fname};
    }
  }

  auto addBody = [&res](const std::string & body, const std::string & file) { res[body] = {body, file}; };
  addBody("body", "base_link");
  addBody("torso", "torso_2_link");
  addBody("r_ankle", "leg_right_6_link");
  addBody("l_ankle", "leg_left_6_link");
  addBody("CHEST_P_LINK", "torso_1_link");
  addBody("r_wrist", "arm_right_7_link");
  addBody("l_wrist", "arm_left_7_link");
  //addBody("R_HIP_Y_LINK", "HIP_Y");
  //addBody("R_HIP_R_LINK", "CHEST_P");
  //addBody("R_ANKLE_P_LINK", "L_ANKLE_P");
  //addBody("L_HIP_Y_LINK", "HIP_Y");
  //addBody("L_HIP_R_LINK", "CHEST_P");
  //addBody("R_SHOULDER_Y_LINK", "SHOULDER_Y");
  //addBody("R_ELBOW_P_LINK", "ELBOW_P");
  //addBody("R_WRIST_P_LINK", "WRIST_P");



  // auto finger = [&addBody](const std::string & prefix) {
  //   addBody(prefix + "_HAND_J0_LINK", prefix + "_THUMB");
  //   addBody(prefix + "_HAND_J1_LINK", prefix + "_F1");
  //   for(unsigned int i = 2; i < 6; ++i)
  //   {
  //     std::stringstream key1;
  //     key1 << prefix << "_F" << i << "2_LINK";
  //     std::stringstream key2;
  //     key2 << prefix << "_F" << i << "3_LINK";
  //     addBody(key1.str(), "F2");
  //     addBody(key2.str(), "F3");
  //   }
  // };
  // finger("R");
  // finger("L");

  auto addWristSubConvex = [&res](const std::string & prefix) {
    // std::string wristY = prefix + "_WRIST_Y_LINK";
    std::string wristR = boost::algorithm::to_lower_copy(prefix) + "_wrist";
    // res[wristY + "_sub0"] = {wristY, prefix + "_WRIST_Y_sub0"};
    res[wristR + "_sub0"] = {wristR, prefix + "_WRIST_R_sub0"};
    res[wristR + "_sub1"] = {wristR, prefix + "_WRIST_R_sub1"};
  };
  addWristSubConvex("L");
  addWristSubConvex("R");

  return res;
}

} // namespace mc_robots
