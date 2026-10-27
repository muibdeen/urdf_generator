// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Turns a plan into the files of a ROS 2 package: the bridge YAML, the launch file, the world,
// a patched copy of the URDF, package.xml, CMakeLists.txt and a README.
#pragma once

#include <string>
#include <vector>

#include "gz_bridge_helper/plan.hpp"

namespace gzb {

struct GeneratedFile {
  std::string path;         // relative to the package folder, e.g. "launch/sim.launch.py"
  std::string content;
  std::string description;  // one line for the GUI
};

struct Generated {
  std::vector<GeneratedFile> files;
  /// Only the lines added to the URDF (for showing what changed).
  std::string urdfAdditions;
  const GeneratedFile* file(const std::string& path) const;
};

/// The URDF with the Gazebo plugins and sensors added. The input text is not modified; the part
/// that was added is returned in `additions`.
std::string patchUrdf(const std::string& urdfText, const UrdfModel& model, const Plan& plan, std::string& additions, Issues& issues);

std::string bridgeConfigFile(const Plan& plan);
std::string launchFile(const Plan& plan, const UrdfModel& model);
std::string worldFile(const Plan& plan);
std::string packageXml(const Plan& plan);
std::string cmakeLists(const Plan& plan);
std::string readmeFile(const Plan& plan, const UrdfModel& model);

/// All the files of the package. `urdfFileName` is the name of the URDF on disk (shown in comments).
Generated generate(const UrdfModel& model, const std::string& urdfText, const std::string& urdfFileName, const Plan& plan, Issues& issues);

/// Writes the files under `directory/<package name>/`. Returns false and sets `error` on failure.
bool writePackage(const std::string& directory, const Plan& plan, const Generated& g, std::string& error);

}  // namespace gzb
