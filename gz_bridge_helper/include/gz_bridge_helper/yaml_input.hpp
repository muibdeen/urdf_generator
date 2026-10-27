// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// The optional YAML files. Three kinds are understood and told apart by what is inside:
//
//   ros2_control controllers  a controller_manager section with the controllers to load
//   ros_gz_bridge config      a sequence of bridge entries (topic_name / ros_type_name / ...)
//   overrides                 this helper's own settings (package_name, sensors:, drive:, ...)
//
// One file may be several kinds at once (a map can hold both a controller_manager section and
// an overrides section).
#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "gz_bridge_helper/bridge.hpp"
#include "gz_bridge_helper/common.hpp"

namespace gzb {

enum class YamlKind { Controllers, BridgeConfig, Overrides };
const char* yamlKindName(YamlKind k);

// ------------------------------------------------------------------ ros2_control controllers

struct ControllerSpec {
  std::string name;   // as in controller_manager.ros__parameters
  std::string type;   // e.g. diff_drive_controller/DiffDriveController
  bool hasParameters = false;  // a section with the controller's own ros__parameters exists
  /// (parameter name, joint name) for every joint the controller's parameters mention.
  std::vector<std::pair<std::string, std::string>> jointRefs;
  std::vector<std::string> commandInterfaces, stateInterfaces;
};

struct ControllersConfig {
  double updateRate = 0;  // 0: not given
  std::vector<ControllerSpec> controllers;
  bool isBroadcaster(const ControllerSpec& c) const;
};

// ------------------------------------------------------------------------------ overrides

struct SensorOverride {
  std::optional<bool> enabled;
  std::optional<std::string> rosTopic;   // replaces the ROS name of the sensor's main topic
  std::optional<std::string> frameId;
  std::optional<double> updateRate;
  std::optional<int> width, height;      // camera
  std::optional<double> horizontalFov;   // camera, rad
  std::optional<int> samples;            // lidar: horizontal samples
  std::optional<double> rangeMin, rangeMax;  // lidar
};

struct DriveOverride {
  std::optional<bool> enabled;
  std::optional<double> wheelSeparation, wheelRadius;
  std::optional<std::string> rosCmdVelTopic, rosOdomTopic;
  std::optional<double> maxLinearVelocity, maxAngularVelocity;
};

struct Overrides {
  std::optional<std::string> packageName, modelName, worldName;
  std::optional<bool> useSimTime;
  std::optional<double> stepSize;
  std::optional<Vec3> gravity;
  struct Spherical {
    double latitude = 0, longitude = 0, elevation = 0, heading = 0;
  };
  std::optional<Spherical> spherical;
  std::optional<double> spawnX, spawnY, spawnZ;  // z unset: computed from the lowest shape
  std::optional<double> spawnRoll, spawnPitch, spawnYaw;
  std::map<std::string, SensorOverride> sensors;
  DriveOverride drive;
  std::optional<bool> jointStates;
  std::optional<std::vector<std::string>> positionControl;
  std::optional<double> pGain, iGain, dGain;
  std::vector<BridgeEntry> bridge;     // extra entries, same format as a bridge config
  std::vector<std::string> exclude;    // ROS topic names that must not be bridged
};

// ------------------------------------------------------------------------------ one file

struct ParsedYaml {
  std::string path;   // as given (shown to the user)
  std::string text;   // the whole file, kept so that it can be copied into the package
  bool ok = false;    // it is valid YAML
  std::set<YamlKind> kinds;
  ControllersConfig controllers;
  std::vector<BridgeEntry> bridge;   // kind BridgeConfig
  Overrides overrides;               // kind Overrides

  bool is(YamlKind k) const { return kinds.count(k) > 0; }
  /// "ros2_control controllers + overrides", or "unrecognised".
  std::string kindText() const;
};

/// Parses and classifies one YAML file's text. Problems are appended to `issues`.
ParsedYaml parseYaml(const std::string& path, const std::string& text, Issues& issues);

/// The keys of the overrides format, for the documentation and the template.
std::string overridesTemplate();

}  // namespace gzb
