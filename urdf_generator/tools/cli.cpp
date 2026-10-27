// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Headless generator: writes <name>.xacro and <name>.urdf for the default
// configuration of one robot type (or all of them). Handy for CI and for
// checking what the GUI starts from.
//
//   urdf_generator_cli [type|all] [output_dir]
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "urdf_generator/builders.hpp"
#include "urdf_generator/emitter.hpp"

namespace {

void usage() {
  std::cerr << "usage: urdf_generator_cli [type|all] [output_dir]\ntypes:";
  for (auto t : urdfgen::allRobotTypes()) std::cerr << ' ' << urdfgen::robotTypeId(t);
  std::cerr << "\n";
}

bool writeFile(const std::filesystem::path& path, const std::string& text) {
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << text;
  return static_cast<bool>(f);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string which = argc > 1 ? argv[1] : "all";
  const std::filesystem::path outDir = argc > 2 ? argv[2] : ".";
  if (which == "-h" || which == "--help") {
    usage();
    return 0;
  }

  std::error_code ec;
  std::filesystem::create_directories(outDir, ec);
  if (ec) {
    std::cerr << "cannot create " << outDir << ": " << ec.message() << "\n";
    return 1;
  }

  int generated = 0;
  int failed = 0;
  for (auto t : urdfgen::allRobotTypes()) {
    if (which != "all" && which != urdfgen::robotTypeId(t)) continue;
    const urdfgen::Robot robot = urdfgen::buildDefault(t);
    const auto issues = urdfgen::validate(robot);
    for (const auto& i : issues) {
      std::cerr << (i.severity == urdfgen::Issue::Severity::Error ? "error: " : "warning: ")
                << robot.name << ": " << i.message << "\n";
    }
    if (urdfgen::hasErrors(issues)) {
      ++failed;
      continue;
    }
    const auto xacroPath = outDir / (robot.name + ".xacro");
    const auto urdfPath = outDir / (robot.name + ".urdf");
    if (!writeFile(xacroPath, urdfgen::emitXacro(robot)) ||
        !writeFile(urdfPath, urdfgen::emitUrdf(robot))) {
      std::cerr << "cannot write into " << outDir << "\n";
      return 1;
    }
    std::cout << "wrote " << xacroPath.string() << " and " << urdfPath.string() << "\n";
    ++generated;
  }
  if (generated == 0 && failed == 0) {
    usage();
    return 2;
  }
  return failed == 0 ? 0 : 1;
}
