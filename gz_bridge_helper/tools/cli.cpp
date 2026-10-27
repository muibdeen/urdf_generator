// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Command line version of the helper:
//   gz_bridge_helper_cli robot.urdf [more.yaml ...] [--out DIR] [--list] [--only id,id] [--without id,id]
// Prints what was detected and the issues; with --out it writes the package.
#include <cstdio>
#include <iostream>
#include <set>

#include "gz_bridge_helper/project.hpp"

using namespace gzb;

namespace {
void usage() {
  std::cerr << "usage: gz_bridge_helper_cli robot.urdf [config.yaml ...] [--out DIR] [--list] [--only id,id] [--without id,id]\n"
               "  --list     print the detected items and the issues\n"
               "  --out DIR  write the package to DIR/<package name>\n"
               "  --only     use exactly these item ids (see --list)\n"
               "  --without  use the suggestions except these ids\n";
}
}  // namespace

int main(int argc, char** argv) {
  std::string urdf, out;
  std::vector<std::string> yamls;
  bool list = false;
  std::vector<std::string> only, without;
  bool haveOnly = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--out" && i + 1 < argc) out = argv[++i];
    else if (a == "--list") list = true;
    else if (a == "--only" && i + 1 < argc) { only = split(argv[++i], ','); haveOnly = true; }
    else if (a == "--without" && i + 1 < argc) without = split(argv[++i], ',');
    else if (a == "--help" || a == "-h") { usage(); return 0; }
    else if (startsWith(a, "--")) { usage(); return 2; }
    else if (urdf.empty()) urdf = a;
    else yamls.push_back(a);
  }
  if (urdf.empty()) { usage(); return 2; }

  Project p;
  p.loadUrdfFile(urdf);
  for (const std::string& y : yamls) p.addYamlFile(y);
  if (haveOnly) {
    for (const Item& it : p.detection().items) p.setSelected(it.id, false);
    for (const std::string& id : only) p.setSelected(id, true);
  }
  for (const std::string& id : without) p.setSelected(id, false);

  if (list || out.empty()) {
    std::cout << "URDF: " << urdf << (p.model().valid ? "" : "  (not usable)") << "\n";
    for (const ParsedYaml& y : p.yamls()) std::cout << "YAML: " << y.path << "  [" << y.kindText() << "]\n";
    for (const Item& it : p.detection().items) {
      std::cout << "  [" << (p.selected().count(it.id) ? "x" : " ") << "] " << it.id << "\n        " << it.detail << "\n";
    }
    if (p.model().valid) {
      std::cout << "\nBridge (" << p.plan().bridge.size() << " entries):\n";
      for (const BridgeEntry& e : p.plan().bridge) {
        if (e.isService()) {
          std::cout << "  " << (e.enabled ? " " : "-") << " service " << e.serviceName << " (" << e.rosType << ")\n";
          continue;
        }
        const char* arrow = e.direction == Direction::GzToRos ? "<-" : e.direction == Direction::RosToGz ? "->" : "<->";
        std::cout << "  " << (e.enabled ? " " : "-") << " ROS " << e.rosTopic << " " << arrow << " GZ " << e.gzTopic << "   [" << e.rosType << " / " << e.gzType << "]\n";
      }
    }
  }
  int errors = 0, warnings = 0;
  for (const Issue& i : p.issues()) {
    std::cout << severityName(i.severity) << " [" << i.where << "] " << i.message << "\n";
    if (i.severity == Severity::Error) ++errors;
    if (i.severity == Severity::Warning) ++warnings;
  }
  if (!out.empty()) {
    std::string error;
    if (!p.write(out, error)) {
      std::cerr << "not written: " << error << "\n";
      return 1;
    }
    std::cout << "wrote " << out << "/" << p.plan().packageName << " (" << p.generated().files.size() << " files, " << warnings << " warning(s))\n";
  }
  return errors ? 1 : 0;
}
