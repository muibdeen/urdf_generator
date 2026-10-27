// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// One line of the ros_gz_bridge configuration: which topic is copied, in which direction, and
// as which message type on each side. The YAML written by bridgeYaml() is exactly the format that
//   ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=<file>
// reads (the top level is a sequence of maps; see ros_gz_bridge/src/bridge_config.cpp).
#pragma once

#include <string>
#include <vector>

#include "gz_bridge_helper/common.hpp"

namespace gzb {

enum class Direction { Bidirectional, GzToRos, RosToGz };
const char* directionName(Direction d);          // BIDIRECTIONAL | GZ_TO_ROS | ROS_TO_GZ
bool parseDirection(const std::string& s, Direction& out);
/// "[" for GZ_TO_ROS, "]" for ROS_TO_GZ, "@" for both: the symbol of the command-line form.
const char* directionSymbol(Direction d);

struct BridgeEntry {
  // A topic bridge ...
  std::string rosTopic, gzTopic;
  std::string rosType, gzType;
  Direction direction = Direction::GzToRos;
  bool lazy = false;
  std::string qos;             // CLOCK, SENSOR_DATA, ... or empty
  int subscriberQueue = -1;    // -1: not written
  int publisherQueue = -1;
  std::string frameId;         // override of header.frame_id on GZ -> ROS messages
  // ... or a service bridge (then the topic fields above are unused).
  std::string serviceName, gzRequestType, gzReplyType;

  // Bookkeeping, not written to the YAML.
  std::string origin;    // "generated", or "file <name>"
  std::string purpose;   // one line for humans: what this carries
  std::string group;     // clock, sensor, drive, joint, odometry or user (colours in the GUI)
  bool enabled = true;

  bool isService() const { return !serviceName.empty(); }
};

/// The YAML text of the enabled entries, with the purpose of each as a comment above it.
std::string bridgeYaml(const std::vector<BridgeEntry>& entries, const std::string& header = "");

/// Checks one entry the way ros_gz_bridge would when it reads it; every problem is added to
/// `issues` (entries the bridge cannot read are skipped by it with only a log line).
void validateEntry(const BridgeEntry& e, const std::string& where, Issues& issues);

/// "/a/b/c" -> "/a/b"; "/a" -> "/"; used for the camera_info topic that sits next to the image topic.
std::string topicParent(const std::string& topic);
/// Adds the leading '/' that gz-transport and the bridge expect in absolute names.
std::string absoluteTopic(const std::string& topic);

}  // namespace gzb
