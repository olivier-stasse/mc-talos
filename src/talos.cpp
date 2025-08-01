#include "talos.h"
#include "config.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/constants.h>
#include <RBDyn/parsers/urdf.h>
#include <sch/S_Object/S_Sphere.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <sch/S_Object/S_Sphere.h>



namespace mc_robots
{

TalosCommonRobotModule::TalosCommonRobotModule() : RobotModule(TALOS_DESCRIPTION_PATH, "talos")
{
  //rsdf_dir = path + "/rsdf";
	urdf_path = TALOS_URDF_PATH;
	_real_urdf = urdf_path;
	rsdf_dir = TALOS_RSDF_DIR;
  calib_dir = TALOS_CALIB_DIR;

}

void TalosCommonRobotModule::setupCommon()
{
  _grippers = {{"l_gripper", {"gripper_left_joint"}, false}, {"r_gripper", {"gripper_right_joint"}, false}};

  // clang-format off
  _stance = {
      {"torso_1_joint", {                      0.0}},
      {"torso_2_joint", {                      0.0505}},
      {"head_1_joint", {                       0.0}},
      {"head_2_joint", {                       0.0}},
      {"arm_left_1_joint", {                   0.4}},
      {"arm_left_2_joint", {                   0.24}},
      {"arm_left_3_joint", {                   -0.6}},
      {"arm_left_4_joint", {                  -1.45}},
      {"arm_left_5_joint", {                   0.0}},
      {"arm_left_6_joint", {                   0.0}},
      {"arm_left_7_joint", {                   0.0}},
      {"arm_right_1_joint", {                 -0.4}},
      {"arm_right_2_joint", {                 -0.24}},
      {"arm_right_3_joint", {                  0.6}},
      {"arm_right_4_joint", {                 -1.45}},
      {"arm_right_5_joint", {                  0.0}},
      {"arm_right_6_joint", {                  0.0}},
      {"arm_right_7_joint", {                  0.0}},
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
      {"leg_left_2_joint", {                   0.003}},
      {"leg_left_3_joint", {                  -0.26}},
      {"leg_left_4_joint", {                   0.6}},
      {"leg_left_5_joint", {                  -0.336}},
      {"leg_left_6_joint", {                  -0.005}},

      {"leg_right_1_joint", {                  0.0}},
      {"leg_right_2_joint", {                  0.003}},
      {"leg_right_3_joint", {                 -0.26}},
      {"leg_right_4_joint", {                  0.6}},
      {"leg_right_5_joint", {                 -0.336}},
      {"leg_right_6_joint", {                  -0.005}}
  };
  // clang-format on

  _default_attitude = {1, 0, 0, 0, 0.025, -0.0038, 1.05105};

  auto fileByBodyName = stdCollisionsFiles(mb);
  initConvexHull(fileByBodyName);

  // Force Sensor attachments
  // XXX according to PAL Talos Handbook, p11
  _forceSensors.emplace_back("RightFootForceSensor", "leg_right_6_link", sva::PTransformd::Identity());
  _forceSensors.emplace_back("LeftFootForceSensor", "leg_left_6_link", sva::PTransformd::Identity());
  _forceSensors.emplace_back("RightHandForceSensor", "wrist_right_ft_link", sva::PTransformd::Identity());
  _forceSensors.emplace_back("LeftHandForceSensor", "wrist_left_ft_link", sva::PTransformd::Identity());

  _bodySensors.clear();
  _bodySensors.emplace_back("Accelerometer", "imu_link", sva::PTransformd::Identity());
  _bodySensors.emplace_back("FloatingBase", "base_link", sva::PTransformd::Identity());


// conservative collision objects to reduce collision pairs
  _collisionObjects["L_HAND_SPHERE"] = {"gripper_left_base_link", std::make_shared<sch::S_Sphere>(0.09)};
  _collisionTransforms["L_HAND_SPHERE"] = sva::PTransformd(Eigen::Vector3d(-0.0, 0, -0.08));
  _collisionObjects["R_HAND_SPHERE"] = {"gripper_right_base_link", std::make_shared<sch::S_Sphere>(0.09)};
  _collisionTransforms["R_HAND_SPHERE"] = sva::PTransformd(Eigen::Vector3d(-0.0, 0, -0.08));
  // clang-format off
  _minimalSelfCollisions = {
                            // torso link - right arm
                            {"torso_2_link", "arm_right_2_link", 0.02, 0.005, 0.},
                            {"torso_2_link", "arm_right_4_link", 0.03, 0.01, 0.},
                            {"torso_2_link", "arm_right_5_link", 0.03, 0.01, 0.},
                            {"torso_2_link", "R_HAND_SPHERE", 0.03, 0.01, 0.},

                            // torso link - left arm
                            {"torso_2_link", "arm_left_2_link", 0.02, 0.005, 0.},
                            {"torso_2_link", "arm_left_4_link", 0.03, 0.01, 0.},
                            {"torso_2_link", "arm_left_5_link", 0.03, 0.01, 0.},
                            {"torso_2_link", "L_HAND_SPHERE", 0.03, 0.01, 0.},

                            // base link - right arm
                            {"base_link", "arm_right_4_link", 0.03, 0.01, 0.},
                            {"base_link", "arm_right_5_link", 0.03, 0.01, 0.},
                            {"base_link", "R_HAND_SPHERE", 0.03, 0.01, 0.},

                            // base link - left arm
                            {"base_link", "arm_left_4_link", 0.03, 0.01, 0.},
                            {"base_link", "arm_left_5_link", 0.03, 0.01, 0.},
                            {"base_link", "L_HAND_SPHERE", 0.03, 0.01, 0.},


                            // Right leg - arms
                            {"leg_right_1_link", "R_HAND_SPHERE", 0.03, 0.01, 0.},
                            {"leg_right_1_link", "L_HAND_SPHERE", 0.03, 0.01, 0.},
                            {"leg_right_3_link", "R_HAND_SPHERE", 0.03, 0.01, 0.},
                            {"leg_right_3_link", "L_HAND_SPHERE", 0.03, 0.01, 0.},
                            {"leg_right_3_link", "arm_right_5_link", 0.03, 0.01, 0.},

                            // Left leg - arms
                            {"leg_left_1_link", "L_HAND_SPHERE", 0.03, 0.01, 0.},
                            {"leg_left_1_link", "R_HAND_SPHERE", 0.03, 0.01, 0.},
                            {"leg_left_3_link", "R_HAND_SPHERE", 0.03, 0.01, 0.},
                            {"leg_left_3_link", "L_HAND_SPHERE", 0.03, 0.01, 0.},
                            {"leg_left_3_link", "arm_left_5_link", 0.03, 0.01, 0.},

                            // Leg - Leg collision
                            {"leg_right_3_link", "leg_left_3_link", 0.01, 0.001, 0.},
                            {"leg_right_4_link", "leg_left_4_link", 0.01, 0.001, 0.},
                            {"leg_right_6_link", "leg_left_6_link", 0.01, 0.001, 0.},
                            {"leg_right_6_link", "leg_left_4_link", 0.02, 0.005, 0.},
                            {"leg_right_4_link", "leg_left_6_link", 0.02, 0.005, 0.}
  };
  // clang-format on

  _commonSelfCollisions = _minimalSelfCollisions;


  // Configure the stabilizer
  _lipmStabilizerConfig.leftFootSurface = "LeftFootCenter";
  _lipmStabilizerConfig.rightFootSurface = "RightFootCenter";
  _lipmStabilizerConfig.torsoBodyName = "torso_2_link";
  _lipmStabilizerConfig.comHeight = 0.879;
  _lipmStabilizerConfig.torsoWeight = 100;
  // _lipmStabilizerConfig.dfzAdmittance = 0.0001;
  // _lipmStabilizerConfig.dfzDamping = 0.01;

  // clang-format off
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
  // clang-format on

  // _lipmStabilizerConfig.torsoPitch = 0.0;
  // _lipmStabilizerConfig.copAdmittance = Eigen::Vector2d{0.03, 0.04};
  // _lipmStabilizerConfig.dcmPropGain = 4.0;
  // _lipmStabilizerConfig.dcmIntegralGain = 10;
  // _lipmStabilizerConfig.dcmDerivGain = 0.2;
  // _lipmStabilizerConfig.dcmDerivatorTimeConstant = 5;
  // _lipmStabilizerConfig.dcmIntegratorTimeConstant = 10;

  _lipmStabilizerConfig.torsoPitch = 0.0;
  _lipmStabilizerConfig.copAdmittance = Eigen::Vector2d{0.001, 0.001};
  _lipmStabilizerConfig.dcmPropGain = 7.0;
  _lipmStabilizerConfig.dcmIntegralGain = 8.0;
  _lipmStabilizerConfig.dcmDerivGain = 0.0;
  _lipmStabilizerConfig.dcmDerivatorTimeConstant = 1;
  _lipmStabilizerConfig.dcmIntegratorTimeConstant = 10;

  _ref_joint_order ={"arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint","arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint",
    "arm_right_1_joint","arm_right_2_joint","arm_right_3_joint","arm_right_4_joint", "arm_right_5_joint","arm_right_6_joint", "arm_right_7_joint",
    "gripper_left_joint", "gripper_left_motor_single_joint", "gripper_left_inner_double_joint", "gripper_left_inner_single_joint",
    "gripper_left_fingertip_1_joint", "gripper_left_fingertip_2_joint", "gripper_left_fingertip_3_joint",
    "gripper_right_joint", "gripper_right_motor_single_joint", "gripper_right_inner_double_joint", "gripper_right_inner_single_joint",
    "gripper_right_fingertip_1_joint", "gripper_right_fingertip_2_joint", "gripper_right_fingertip_3_joint",
    "head_1_joint", "head_2_joint",
    "leg_left_1_joint",  "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint",
    "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint","leg_right_6_joint",
    "torso_1_joint", "torso_2_joint"
  };
}

  TalosRobotModule::TalosRobotModule(bool fixed, bool filter_mimics) : TalosCommonRobotModule()
{

  std::vector<std::string> filter_links = {};
  if(filter_mimics)
  {
    // clang-format off
    filter_links = {
      "gripper_left_inner_double_link",
      "gripper_left_inner_single_link",
      "gripper_left_motor_single_link",
      "gripper_left_motor_double_link",
      "gripper_left_fingertip_1_link",
      "gripper_left_fingertip_2_link",
      "gripper_left_fingertip_3_link",
      "gripper_right_inner_double_link",
      "gripper_right_inner_single_link",
      "gripper_right_motor_single_link",
      "gripper_right_motor_double_link",
      "gripper_right_fingertip_1_link",
      "gripper_right_fingertip_2_link",
      "gripper_right_fingertip_3_link",
    };
    // clang-format on
  }
  init(rbd::parsers::from_urdf_file(urdf_path, fixed, {}, true, "base_link"));
  setupCommon();
}

void TalosCommonRobotModule::initConvexHull(
    const std::map<std::string, std::pair<std::string, std::string>> & files)
{
  std::string convexPath = TALOS_CONVEX_DIR;
  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & f : files)
  {
    _convexHull[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + "/" + f.second.second + "-ch.txt");
  }
}

std::map<std::string, std::pair<std::string, std::string>> TalosCommonRobotModule::stdCollisionsFiles(
    const rbd::MultiBody & mb) const
{
  // Add Bodies
  std::map<std::string, std::pair<std::string, std::string>> res;
  const std::vector<std::string> prefix{"arm", "torso", "head", "gripper"};
  auto mirroredConvexes = std::vector<std::string>{};
  for(unsigned i = 1; i<8; ++i) { mirroredConvexes.push_back(fmt::format("arm_right_{}_link", i)); };
  for(unsigned i = 1; i<7; ++i) { mirroredConvexes.push_back(fmt::format("leg_right_{}_link", i)); };
  for(const auto & b : mb.bodies())
  {

    // Filter out virtual links without convex files
    if(std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
    {
      // Extract convex paths from the urdf <collision> geometry.
      // For the right arm and leg, the convex geometry is mirrored. As we currently don't have support for
      // mirroring sch geometry, we use manually mirrored convex files located in the mirrored/ directory.
      const auto &geom = _collision;
      if(geom.count(b.name()))
      {
        const auto & geoms = geom.at(b.name());
        for(const auto & gg : geoms)
        {
          const auto & g = gg.geometry;
          if(g.type == rbd::parsers::Geometry::MESH)
          {
            auto path = boost::get<rbd::parsers::Geometry::Mesh>(g.data).filename;
            auto prefix = std::string{"package://talos_description/meshes/"};
            if (path.rfind(prefix, 0) == 0)
            {
              path = path.substr(prefix.size());
              auto bpath = boost::filesystem::path{path};
              path = bpath.parent_path().string() + "/" + bpath.stem().string();
            }
            if(std::find(mirroredConvexes.begin(), mirroredConvexes.end(), b.name()) != mirroredConvexes.end())
            {
              path = "mirrored/" + path;
              mc_rtc::log::info("Body {} has a mirrored convex file, using {}", b.name(), path);
            }
            res[b.name()] = {b.name(), path};
          }
        }
      }
    }
  }

  auto addBody = [&res](const std::string & body, const std::string & file) { res[body] = {body, file}; };
  addBody("body", "base_link");
  addBody("torso", "torso_2_link");
  addBody("r_ankle", "leg_right_6_link");
  addBody("l_ankle", "leg_left_6_link");
  addBody("r_wrist", "arm_right_7_link");
  addBody("l_wrist", "arm_left_7_link");

  return res;
}

} // namespace mc_robots
