// Copyright 2026. Licensed under the Apache License, Version 2.0.
#pragma once

#include <string>
#include <vector>

#include "urdf_generator/configs.hpp"
#include "urdf_generator/model.hpp"

namespace urdfgen {

Robot buildHexapod(const HexapodConfig& cfg);
Robot buildArm(const ArmConfig& cfg);
Robot buildDiffDrive(const DiffDriveConfig& cfg);
Robot buildFourWheel(const FourWheelConfig& cfg);
Robot buildAckermann(const AckermannConfig& cfg);
Robot buildQuadruped(const QuadrupedConfig& cfg);
Robot buildBiped(const BipedConfig& cfg);

/// All supported robot types, in menu order.
const std::vector<RobotType>& allRobotTypes();
/// Machine name ("diff_drive") and human name ("Differential drive").
std::string robotTypeId(RobotType t);
std::string robotTypeLabel(RobotType t);
/// Build a robot of the given type from its default configuration.
Robot buildDefault(RobotType t);

}  // namespace urdfgen
