// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// User-facing parameters for each supported robot. All lengths are metres,
// masses kilograms, and angles degrees (the builders convert to radians).
#pragma once

#include <string>
#include <vector>

namespace urdfgen {

enum class RobotType { Hexapod, Arm, DiffDrive, FourWheel, Ackermann, Quadruped, Biped };

/// Chassis shared by the three wheeled robots.
enum class ChassisShape { Box, Cylinder };
struct ChassisConfig {
  ChassisShape shape = ChassisShape::Box;
  double length = 0.30, width = 0.20, height = 0.10;  // Box (x, y, z)
  double radius = 0.15;                                // Cylinder (height as above)
  double mass = 5.0;
};

struct HexapodConfig {
  std::string name = "hexapod";
  double coxaLength = 0.06, femurLength = 0.10, tibiaLength = 0.15;
  double bodyRadius = 0.12;  // distance of the six leg mounts from the centre
  double bodyHeight = 0.05;
  double legRadius = 0.008;
  double bodyMass = 1.0, coxaMass = 0.05, femurMass = 0.08, tibiaMass = 0.08, footMass = 0.01;
  double coxaLimitDeg = 45, femurLimitDeg = 90, tibiaLimitDeg = 120;  // +/- limits
  double jointEffort = 5.0, jointVelocity = 3.0;
};

enum class ArmJointType { Revolute, Prismatic };
enum class ArmAxis { X, Y, Z };
enum class OffsetMode { Cartesian, Polar };

/// One joint+link of the arm. The offset places this joint relative to the
/// previous link's frame:
///   Cartesian: (a, b, c) = (x, y, z) in metres
///   Polar:     (a, b, c) = (r in metres, theta in degrees about +z, z in metres),
///              i.e. x = r*cos(theta), y = r*sin(theta)
struct ArmLinkConfig {
  ArmJointType type = ArmJointType::Revolute;
  ArmAxis axis = ArmAxis::Z;
  OffsetMode mode = OffsetMode::Cartesian;
  double a = 0, b = 0, c = 0.1;
  double lower = -170, upper = 170;  // degrees if Revolute, metres if Prismatic
  double mass = 0.5;
};

struct ArmConfig {
  std::string name = "arm";
  double baseRadius = 0.06, baseHeight = 0.05, baseMass = 2.0;
  double linkRadius = 0.02;
  double toolX = 0, toolY = 0, toolZ = 0.10;  // end-effector frame, in the last link's frame
  double jointEffort = 20.0, jointVelocity = 2.0;
  std::vector<ArmLinkConfig> links = {
      {ArmJointType::Revolute, ArmAxis::Z, OffsetMode::Cartesian, 0, 0, 0.05, -170, 170, 1.0},
      {ArmJointType::Revolute, ArmAxis::Y, OffsetMode::Cartesian, 0, 0, 0.10, -90, 90, 0.8},
      {ArmJointType::Revolute, ArmAxis::Y, OffsetMode::Cartesian, 0, 0, 0.25, -120, 120, 0.5},
  };
};

/// A plain cylinder wheel, or a mecanum wheel: a hub with rollers set at 45 degrees around
/// its rim. The slant of the rollers follows the usual layout (front-left and rear-right
/// one way, front-right and rear-left the other, so that the roller axes of the four wheels
/// point at the middle of the robot).
enum class WheelKind { Standard, Mecanum };

/// Two drive wheels (left/right) plus optional ball casters front and rear.
struct DiffDriveConfig {
  std::string name = "diff_drive_robot";
  ChassisConfig chassis;
  double wheelRadius = 0.05, wheelWidth = 0.03, wheelMass = 0.5;
  WheelKind leftWheel = WheelKind::Standard, rightWheel = WheelKind::Standard;
  int mecanumRollers = 8;  // rollers around each mecanum wheel
  // Left wheel centre relative to the chassis centre; the right wheel mirrors y.
  double wheelX = 0.0, wheelY = 0.12, wheelZ = -0.03;
  bool casters = true;
  double casterRadius = 0.02, casterMass = 0.1;
};

/// Four driven wheels (skid-steer / 4WD), mirrored front-back and left-right.
struct FourWheelConfig {
  std::string name = "four_wheel_robot";
  ChassisConfig chassis;
  double wheelRadius = 0.06, wheelWidth = 0.04, wheelMass = 0.5;
  WheelKind frontLeftWheel = WheelKind::Standard, frontRightWheel = WheelKind::Standard,
            rearLeftWheel = WheelKind::Standard, rearRightWheel = WheelKind::Standard;
  int mecanumRollers = 8;  // rollers around each mecanum wheel
  double wheelX = 0.10, wheelY = 0.14, wheelZ = -0.03;  // front-left; others mirror x and y
};

/// Rear wheels driven, front wheels steered (Ackermann geometry).
struct AckermannConfig {
  std::string name = "ackermann_robot";
  ChassisConfig chassis;
  double wheelRadius = 0.06, wheelWidth = 0.04, wheelMass = 0.5;
  WheelKind frontLeftWheel = WheelKind::Standard, frontRightWheel = WheelKind::Standard,
            rearLeftWheel = WheelKind::Standard, rearRightWheel = WheelKind::Standard;
  int mecanumRollers = 8;  // rollers around each mecanum wheel
  double wheelbase = 0.30;    // front axle x = +wheelbase/2, rear axle x = -wheelbase/2
  double trackWidth = 0.26;   // distance between left and right wheels
  double wheelZ = -0.03;
  double steerLimitDeg = 30;  // +/- steering angle
  double steerEffort = 5.0, steerVelocity = 2.0;
};

/// Four 3-DOF legs: coxa (roll), femur (pitch), tibia (pitch).
struct QuadrupedConfig {
  std::string name = "quadruped";
  double bodyLength = 0.40, bodyWidth = 0.20, bodyHeight = 0.08;
  double legMountX = 0.15, legMountY = 0.10;  // distance of the hips from the centre
  double coxaLength = 0.05, femurLength = 0.20, tibiaLength = 0.20;
  double legRadius = 0.012;
  double bodyMass = 4.0, coxaMass = 0.2, femurMass = 0.3, tibiaMass = 0.15, footMass = 0.02;
  double coxaLimitDeg = 40, femurLimitDeg = 90, tibiaLimitDeg = 120;
  double jointEffort = 20.0, jointVelocity = 5.0;
};

/// Two 3-DOF legs: hip, knee, ankle (all pitch), plus a box foot.
struct BipedConfig {
  std::string name = "biped";
  double torsoDepth = 0.12, torsoWidth = 0.20, torsoHeight = 0.25;
  double hipOffset = 0.07;  // distance of each hip from the centre, along y
  double thighLength = 0.20, shankLength = 0.20;
  double footLength = 0.14, footWidth = 0.07, footHeight = 0.03;
  double legRadius = 0.02;
  double torsoMass = 3.0, thighMass = 0.6, shankMass = 0.4, footMass = 0.15;
  double hipLimitDeg = 90, kneeLimitDeg = 140, ankleLimitDeg = 45;  // knee: 0..limit
  double jointEffort = 30.0, jointVelocity = 4.0;
};

}  // namespace urdfgen
