// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/project.hpp"

namespace gzb {

bool Project::loadUrdfFile(const std::string& path) {
  std::string text, error;
  if (!readTextFile(path, text, error)) {
    urdf_path_ = path;
    urdf_text_.clear();
    model_ = UrdfModel();
    urdf_issues_.clear();
    addIssue(urdf_issues_, Severity::Error, "URDF", "Cannot read the URDF: " + error + ".");
    analyze();
    return false;
  }
  setUrdf(path, text);
  return true;
}

void Project::setUrdf(const std::string& path, const std::string& text) {
  urdf_path_ = path;
  urdf_text_ = text;
  urdf_issues_.clear();
  model_ = parseUrdf(text, urdf_issues_);
  analyze();
}

bool Project::addYamlFile(const std::string& path) {
  std::string text, error;
  if (!readTextFile(path, text, error)) {
    ParsedYaml y;
    y.path = path;
    Issues is;
    addIssue(is, Severity::Error, "YAML " + baseName(path), "Cannot read the YAML file: " + error + ".");
    yamls_.push_back(y);
    yaml_issues_.push_back(is);
    analyze();
    return false;
  }
  addYamlText(path, text);
  return true;
}

void Project::addYamlText(const std::string& path, const std::string& text) {
  Issues is;
  yamls_.push_back(parseYaml(path, text, is));
  yaml_issues_.push_back(is);
  analyze();
}

void Project::removeYaml(size_t index) {
  if (index >= yamls_.size()) return;
  yamls_.erase(yamls_.begin() + static_cast<long>(index));
  yaml_issues_.erase(yaml_issues_.begin() + static_cast<long>(index));
  analyze();
}

void Project::clearYaml() {
  yamls_.clear();
  yaml_issues_.clear();
  analyze();
}

void Project::analyze() {
  detect_issues_.clear();
  detection_ = detect(model_, effective_yamls(), detect_issues_);
  selected_ = detection_.defaultSelection();
  rebuild();
}

std::vector<ParsedYaml> Project::effective_yamls() const {
  std::vector<ParsedYaml> all = yamls_;
  ParsedYaml w;
  w.path = "(settings in the window)";
  w.ok = true;
  w.kinds.insert(YamlKind::Overrides);
  w.overrides = window_overrides_;
  all.push_back(w);
  return all;
}

void Project::setSelection(const std::set<std::string>& ids) {
  selected_ = ids;
  rebuild();
}

void Project::setWindowOverrides(const Overrides& o) {
  window_overrides_ = o;
  rebuild();
}

void Project::setSelected(const std::string& id, bool on) {
  if (on) selected_.insert(id);
  else selected_.erase(id);
  rebuild();
}

void Project::rebuild() {
  plan_issues_.clear();
  generate_issues_.clear();
  plan_ = Plan();
  generated_ = Generated();
  if (!model_.valid) return;
  plan_ = buildPlan(model_, detection_, selected_, effective_yamls(), plan_issues_);
  generated_ = generate(model_, urdf_text_, baseName(urdf_path_), plan_, generate_issues_);
}

Issues Project::issues() const {
  Issues all = urdf_issues_;
  for (const Issues& y : yaml_issues_) all.insert(all.end(), y.begin(), y.end());
  all.insert(all.end(), detect_issues_.begin(), detect_issues_.end());
  all.insert(all.end(), plan_issues_.begin(), plan_issues_.end());
  all.insert(all.end(), generate_issues_.begin(), generate_issues_.end());
  return all;
}

bool Project::write(const std::string& directory, std::string& error) const {
  if (!ready()) {
    error = "There are errors to fix first (see the Issues list).";
    return false;
  }
  return writePackage(directory, plan_, generated_, error);
}

}  // namespace gzb
