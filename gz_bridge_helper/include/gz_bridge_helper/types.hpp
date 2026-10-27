// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Which ROS 2 message types ros_gz_bridge can convert to which Gazebo (gz.msgs) types.
// The table in types_table.cpp is generated from the ros_gz_bridge README of the Jazzy branch.
#pragma once

#include <string>
#include <vector>

namespace gzb {

struct TypePair {
  std::string ros;  // e.g. "sensor_msgs/msg/Imu"
  std::string gz;   // e.g. "gz.msgs.IMU"
};

struct ServiceTypePair {
  std::string ros;  // e.g. "ros_gz_interfaces/srv/SpawnEntity"
  std::string gzRequest;
  std::string gzReply;
};

const std::vector<TypePair>& messageTypePairs();
const std::vector<ServiceTypePair>& serviceTypePairs();

bool knownRosType(const std::string& ros);
bool knownGzType(const std::string& gz);
/// True when the bridge converts `ros` to `gz` (and back).
bool isKnownPair(const std::string& ros, const std::string& gz);
/// All Gazebo types the bridge can convert the ROS type to.
std::vector<std::string> gzTypesFor(const std::string& ros);
/// The ROS types for a Gazebo type.
std::vector<std::string> rosTypesFor(const std::string& gz);

}  // namespace gzb
