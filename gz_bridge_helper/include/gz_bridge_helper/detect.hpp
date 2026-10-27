// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Looks through the URDF (and the YAML files) and suggests what Gazebo needs next to the
// model: sensors, a drive system, joint publishers. Nothing is decided here: every suggestion is
// an Item with a tick box, and only the ticked ones reach the plan.
#pragma once

#include <set>
#include <string>
#include <vector>

#include "gz_bridge_helper/common.hpp"
#include "gz_bridge_helper/urdf_model.hpp"
#include "gz_bridge_helper/yaml_input.hpp"

namespace gzb {

/// Who moves the joints: Gazebo's own systems (DiffDrive, JointStatePublisher, ...) or
/// ros2_control controllers running inside gz_ros2_control.
enum class ControlMode { GzSystems, Ros2Control };

enum class SensorKind { Camera, Imu, Lidar, Gps, Magnetometer, AirPressure, Altimeter, Unsupported };
const char* sensorKindLabel(SensorKind k);
/// The type attribute of the SDF <sensor>: camera, imu, gpu_lidar, navsat, ...
const char* sdfSensorType(SensorKind k);
SensorKind sensorKindFromSdfType(const std::string& type);

struct SensorCandidate {
  std::string id;          // "sensor:<name>"
  std::string name;        // unique; the sensor in the URDF, or the link name without "_link"
  SensorKind kind = SensorKind::Camera;
  std::string sdfType;     // as written in the URDF (existing sensors)
  std::string link;        // the link the <gazebo reference="..."> belongs to
  std::string frameLink;   // frame of the messages' header.frame_id
  bool existing = false;   // already written in the URDF
  bool needsTopic = false; // existing, but without a <topic>: Gazebo's name for it is unpredictable
  std::string gzTopic;     // existing sensors: the <topic> of the URDF, absolute
  std::string reason;
};

enum class DriveKind { Diff, Mecanum, Ackermann };
const char* driveKindLabel(DriveKind k);

struct DriveCandidate {
  std::string id;                          // "drive:diff" | "drive:mecanum" | "drive:ackermann"
  DriveKind kind = DriveKind::Diff;
  std::vector<std::string> left, right;    // driven wheel joints (diff, ackermann)
  std::string frontLeft, frontRight, backLeft, backRight;  // mecanum
  std::string leftSteering, rightSteering;                 // ackermann
  double wheelSeparation = 0, wheelRadius = 0, wheelBase = 0, kingpinWidth = 0, steeringLimit = 0;
  std::string reason;
  /// Every joint the drive system moves.
  std::vector<std::string> joints() const;
};

struct JointControlCandidate {
  std::string id;      // "jointpos:<joint>"
  std::string joint;
  double lower = 0, upper = 0;
};

struct OdometryCandidate {
  bool offered = false;     // a publisher of the model's pose can be added
  std::string link;         // a link named like an odometry frame, if any
};

struct Ros2ControlInfo {
  bool urdfHasTag = false;           // the URDF has a <ros2_control> element
  bool pluginPresent = false;        // the URDF has the gz_ros2_control plugin
  bool hardwareNeedsPatch = false;   // <hardware> names something other than gz_ros2_control/GazeboSimSystem
  std::string hardwarePlugin;
  std::vector<ControllerSpec> controllers;
  std::vector<std::string> controllerFiles;  // the YAML files that define controllers, as given
};

/// One tick box of the "detected" list.
struct Item {
  std::string id;
  std::string category;   // Sensors, Drive, Joints, Odometry, ros2_control
  std::string title;
  std::string detail;     // why it is suggested and what it will add
  bool selected = true;   // the suggestion
  bool existing = false;  // already in the URDF: it only gets bridged
};

struct Detection {
  ControlMode mode = ControlMode::GzSystems;
  std::vector<SensorCandidate> sensors;
  std::vector<DriveCandidate> drives;
  OdometryCandidate odometry;
  bool jointStatesOffered = false;
  std::vector<JointControlCandidate> jointControls;
  Ros2ControlInfo ros2;
  std::vector<Item> items;

  std::set<std::string> defaultSelection() const;
  const Item* item(const std::string& id) const;
  const SensorCandidate* sensor(const std::string& name) const;
  const DriveCandidate* drive(const std::string& id) const;
};

Detection detect(const UrdfModel& model, const std::vector<ParsedYaml>& yamls, Issues& issues);

}  // namespace gzb
