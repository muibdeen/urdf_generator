// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Sensors: a camera, IMU, LiDAR, GPS or odometry frame mounted on any link of a robot.
//
// A sensor is mounted at the root of a link (the origin of the link's frame, which is where
// its joint is) and slid along the link: "distance" is measured in metres from the root along
// the link's own axis (the length of a rod, the longest side of a box). On top of that it can
// be shifted by a free offset and turned by roll / pitch / yaw, all in the frame of the link.
// The sensor itself is a small link with a fixed joint to the link it is mounted on; its frame
// is the sensor frame: x forward, y left, z up (REP 103). A camera also gets an optical frame
// (z forward, x right, y down) as a child.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "urdf_generator/model.hpp"
#include "urdf_generator/scene.hpp"

namespace urdfgen {

struct SensorConfig {
  SensorType type = SensorType::Camera;
  std::string name = "camera";  // letters, digits and '_'; the sensor link is <name>_link
  std::string link = "base_link";
  double distance = 0.0;                           // [m] from the root of the link, along its axis
  double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;  // [m] extra shift, in the link's frame
  double rollDeg = 0.0, pitchDeg = 0.0, yawDeg = 0.0;  // orientation relative to the link's frame
};

std::vector<SensorType> allSensorTypes();
/// "camera", "imu", "lidar", "gps", "odometry": the default sensor name and its file-name form.
const char* sensorId(SensorType t);
/// "Camera", "IMU", "LiDAR", "GPS", "Odometry".
const char* sensorLabel(SensorType t);
/// Colour of the sensor's body in the model and the 3D view.
std::array<float, 3> sensorColor(SensorType t);
/// Height of the sensor's body (along its z axis), in metres.
double sensorHeight(SensorType t);

/// The line along which a sensor slides on a link, in the link's frame.
struct LinkAxis {
  Vec3 direction{1, 0, 0};     // unit vector from the root towards the tip of the link
  double lower = 0, upper = 0;  // reachable distances from the root; the range always contains 0
};

/// The link's axis: the axis of a cylinder, the longest side of a box (x before y before z),
/// z for a sphere, x for a bare frame. It points away from the root, so that the body lies at
/// positive distances; a body centred on the root (a wheel, a chassis) gets a range on both sides.
LinkAxis linkAxis(const Link& link);

/// Highest point of the link's body in the link's frame (0 for a bare frame).
double linkTop(const Link& link);

/// Names of the links of `robot` that a sensor can be mounted on, in URDF order.
std::vector<std::string> attachableLinks(const Robot& robot);

/// A new sensor of the given type, mounted on top of the robot's main body and named so
/// that it does not clash with the names in `existing`.
SensorConfig defaultSensor(const Robot& robot, SensorType type, const std::vector<SensorConfig>& existing);

/// Adds the sensors to the robot: a link and a fixed joint each, plus xacro properties for the
/// placement. A sensor that cannot be added (unknown link, bad or repeated name) is skipped and
/// reported as an error by validate().
void addSensors(Robot& robot, const std::vector<SensorConfig>& sensors);

}  // namespace urdfgen
