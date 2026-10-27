// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// The whole job in one object, shared by the command line tool and the GUI:
//   load the URDF and the YAML files -> detect -> choose -> plan -> generate the package.
#pragma once

#include <set>
#include <string>
#include <vector>

#include "gz_bridge_helper/detect.hpp"
#include "gz_bridge_helper/emit.hpp"
#include "gz_bridge_helper/plan.hpp"

namespace gzb {

class Project {
 public:
  /// Reads the URDF from disk. False when the file cannot be read; parse problems are issues().
  bool loadUrdfFile(const std::string& path);
  /// The same from text already in memory (`path` is only a label and the name of the file).
  void setUrdf(const std::string& path, const std::string& text);
  bool addYamlFile(const std::string& path);
  void addYamlText(const std::string& path, const std::string& text);
  void removeYaml(size_t index);
  void clearYaml();

  /// Detects again from the loaded files and resets the choices to the suggestions.
  void analyze();
  /// Ticks or unticks one item of the detection list and rebuilds the plan.
  void setSelected(const std::string& id, bool on);
  /// Replaces the whole choice at once (one rebuild).
  void setSelection(const std::set<std::string>& ids);
  /// Settings typed into the window; they win over every YAML file. Rebuilds the plan.
  void setWindowOverrides(const Overrides& o);
  const Overrides& windowOverrides() const { return window_overrides_; }
  /// Rebuilds the plan and the package files from the current choices.
  void rebuild();

  /// All problems found so far, in the order of the steps.
  Issues issues() const;
  /// A usable URDF and no errors anywhere (a package is only written when this is true).
  bool ready() const { return model_.valid && !hasErrors(issues()); }

  const std::string& urdfPath() const { return urdf_path_; }
  const std::string& urdfText() const { return urdf_text_; }
  const UrdfModel& model() const { return model_; }
  const std::vector<ParsedYaml>& yamls() const { return yamls_; }
  const Detection& detection() const { return detection_; }
  const std::set<std::string>& selected() const { return selected_; }
  const Plan& plan() const { return plan_; }
  const Generated& generated() const { return generated_; }

  /// Writes `<directory>/<package name>/...`; false and `error` on failure or when not ready().
  bool write(const std::string& directory, std::string& error) const;

 private:
  std::string urdf_path_, urdf_text_;
  UrdfModel model_;
  Issues urdf_issues_;
  std::vector<ParsedYaml> yamls_;
  std::vector<Issues> yaml_issues_;
  Overrides window_overrides_;
  Detection detection_;
  Issues detect_issues_;
  std::set<std::string> selected_;
  std::vector<ParsedYaml> effective_yamls() const;
  Plan plan_;
  Issues plan_issues_;
  Generated generated_;
  Issues generate_issues_;
};

}  // namespace gzb
