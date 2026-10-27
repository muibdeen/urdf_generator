// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// The plan: every decision made, with every topic name and parameter written out. The ticked
// suggestions of the detection, the YAML overrides and the user's own bridge entries are merged
// here; the emitters only print what the plan says.
#pragma once

#include <optional>
#include <set>
#include <string>
#include <vector>

#include "gz_bridge_helper/bridge.hpp"
#include "gz_bridge_helper/detect.hpp"

namespace gzb {

struct PlanSensor {
  std::string name;
  SensorKind kind = SensorKind::Camera;
  std::string link, frameLink;
  bool existing = false;      // in the URDF already
  bool addToUrdf = false;     // a <gazebo><sensor> block is added to the copy
  bool addTopicToUrdf = false;  // existing sensor: a <topic> is inserted
  std::string gzTopic;        // the sensor's main topic in Gazebo
  std::string gzSecondTopic;  // camera_info (camera) or points (lidar)
  std::string rosTopic, rosSecondTopic;
  double updateRate = 0;
  int width = 640, height = 480;    // camera
  double horizontalFov = 1.047;     // camera, rad
  int samples = 360;                // lidar
  double rangeMin = 0.12, rangeMax = 12.0;  // lidar
};

struct PlanDrive {
  DriveCandidate d;
  std::string gzCmdVel, gzOdom, gzTf;
  std::string rosCmdVel = "/cmd_vel", rosOdom = "/odom";
  std::string odomFrame = "odom", baseFrame;   // frame_id / child_frame_id of the odometry
  double maxLinear = 0, maxAngular = 0;        // 0: no limit written
};

struct PlanJointControl {
  std::string joint, gzTopic, rosTopic;
  double lower = 0, upper = 0;
};

struct Plan {
  ControlMode mode = ControlMode::GzSystems;
  std::string packageName, modelName, worldName;
  double stepSize = 0.001;
  Vec3 gravity{0, 0, -9.81};
  bool useSimTime = true;
  Vec3 spawnXyz{0, 0, 0.1};
  double spawnRoll = 0, spawnPitch = 0, spawnYaw = 0;
  std::optional<Overrides::Spherical> spherical;

  std::vector<PlanSensor> sensors;
  std::vector<PlanDrive> drives;   // none or one
  bool jointStates = false;
  std::string gzJointStateTopic, rosJointStateTopic = "/joint_states";
  std::vector<PlanJointControl> jointControls;
  double pGain = 10, iGain = 0, dGain = 0.5;
  bool odometry = false;           // OdometryPublisher
  std::string gzOdomTopic, gzOdomTfTopic, rosOdomTopic = "/odom";
  std::string odomFrame = "odom", baseFrame;

  // ros2_control
  bool addControlPlugin = false;
  bool patchHardware = false;
  std::vector<ControllerSpec> controllers;      // spawn order: broadcasters first
  struct ConfigFile {
    std::string sourcePath, name, text;         // name = the file name inside the package's config/
  };
  std::vector<ConfigFile> controllerFiles;

  std::vector<std::string> meshPackages;        // packages whose share/ folder Gazebo must search
  std::vector<BridgeEntry> bridge;

  bool wantsSensorsSystem() const;               // a camera or lidar needs the rendering system
  bool hasKind(SensorKind k) const;
  const PlanSensor* sensor(const std::string& name) const;
};

/// Valid ROS 2 package name: lowercase letters, digits and '_', starting with a letter.
bool validPackageName(const std::string& s);
/// "My-Robot 2" -> "my_robot_2".
std::string packageNameFor(const std::string& robotName);

Plan buildPlan(const UrdfModel& model, const Detection& detection, const std::set<std::string>& selected,
               const std::vector<ParsedYaml>& yamls, Issues& issues);

}  // namespace gzb
