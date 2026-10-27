// Copyright 2026. Licensed under the Apache License, Version 2.0.
#pragma once

#include <string>

#include "urdf_generator/model.hpp"

namespace urdfgen {

/// Parameterised xacro: named properties, inertia macros and repeated
/// structures (legs, wheels) as <xacro:macro>s. Needs the `xacro` tool to
/// become a URDF.
std::string emitXacro(const Robot& robot);

/// Flat URDF with every number resolved and every macro expanded. Loads
/// directly into robot_state_publisher, RViz, Gazebo.
std::string emitUrdf(const Robot& robot);

}  // namespace urdfgen
