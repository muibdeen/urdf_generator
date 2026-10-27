// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/verify.hpp"

#include <cstdlib>
#include <sstream>

namespace gzb {

const char* checkStateName(CheckState s) {
  switch (s) {
    case CheckState::Pending: return "pending";
    case CheckState::Running: return "running";
    case CheckState::Pass: return "pass";
    case CheckState::Fail: return "fail";
    case CheckState::Skipped: return "skipped";
  }
  return "pending";
}

std::string samplePayload(const std::string& t) {
  if (t == "geometry_msgs/msg/Twist") return "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}";
  if (t == "geometry_msgs/msg/TwistStamped") return "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}";
  if (t == "std_msgs/msg/Float64" || t == "std_msgs/msg/Float32") return "{data: 0.0}";
  if (t == "std_msgs/msg/Bool") return "{data: false}";
  if (t == "std_msgs/msg/Int32" || t == "std_msgs/msg/UInt32") return "{data: 0}";
  if (t == "std_msgs/msg/String") return "{data: 'gz_bridge_helper check'}";
  if (t == "std_msgs/msg/Empty") return "{}";
  return "";
}

std::vector<TopicCheck> checksFromPlan(const Plan& plan) {
  std::vector<TopicCheck> out;
  for (const BridgeEntry& e : plan.bridge) {
    if (!e.enabled || e.isService()) continue;
    TopicCheck c;
    c.rosTopic = e.rosTopic;
    c.rosType = e.rosType;
    c.gzTopic = e.gzTopic;
    c.gzType = e.gzType;
    c.direction = e.direction;
    c.description = e.purpose;
    c.payload = samplePayload(e.rosType);
    out.push_back(c);
  }
  return out;
}

std::map<std::string, std::vector<std::string>> parseTopicList(const std::string& text) {
  std::map<std::string, std::vector<std::string>> out;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty() || line[0] != '/') continue;
    const size_t br = line.find(" [");
    if (br == std::string::npos) {
      out[line];
      continue;
    }
    const std::string topic = line.substr(0, br);
    std::string types = line.substr(br + 2);
    if (!types.empty() && types.back() == ']') types.pop_back();
    for (const std::string& t : split(types, ',')) {
      if (!trim(t).empty()) out[topic].push_back(trim(t));
    }
  }
  return out;
}

bool parseHzAverage(const std::string& text, double& hz) {
  bool found = false;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    const size_t at = line.find("average rate:");
    if (at == std::string::npos) continue;
    const std::string rest = trim(line.substr(at + 13));
    char* end = nullptr;
    const double v = std::strtod(rest.c_str(), &end);
    if (end != rest.c_str()) {
      hz = v;
      found = true;
    }
  }
  return found;
}

int parseSubscriptionCount(const std::string& text) {
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    const size_t at = line.find("Subscription count:");
    if (at == std::string::npos) continue;
    return std::atoi(trim(line.substr(at + 19)).c_str());
  }
  return -1;
}

bool echoShowsMessage(const std::string& text) {
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (trim(line) == "---") return true;
  }
  return false;
}

namespace {
bool typeListed(const std::map<std::string, std::vector<std::string>>& topics, const TopicCheck& c, std::string& listedTypes) {
  auto it = topics.find(c.rosTopic);
  if (it == topics.end()) return false;
  for (const std::string& t : it->second) {
    listedTypes += (listedTypes.empty() ? "" : ", ") + t;
  }
  return true;
}
bool hasType(const std::map<std::string, std::vector<std::string>>& topics, const TopicCheck& c) {
  auto it = topics.find(c.rosTopic);
  if (it == topics.end()) return false;
  for (const std::string& t : it->second) {
    if (t == c.rosType) return true;
  }
  return false;
}
}  // namespace

CheckResult evaluatePublished(const TopicCheck& c, const std::map<std::string, std::vector<std::string>>& topics, const std::string& hzText) {
  CheckResult r;
  std::string types;
  if (!typeListed(topics, c, types)) {
    r.state = CheckState::Fail;
    r.detail = "The topic is not in `ros2 topic list`: the bridge is not running or did not accept this entry (see the output).";
    return r;
  }
  if (!hasType(topics, c)) {
    r.state = CheckState::Fail;
    r.detail = "The topic exists with the type " + types + " instead of " + c.rosType + ".";
    return r;
  }
  double hz = 0;
  if (!parseHzAverage(hzText, hz) || hz <= 0) {
    r.state = CheckState::Fail;
    r.detail = "The topic exists but no messages arrived. Is the simulation running (not paused), and does Gazebo publish " + c.gzTopic + "?";
    return r;
  }
  r.state = CheckState::Pass;
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.1f Hz", hz);
  r.detail = std::string("Messages arrive at ") + buf + ".";
  return r;
}

CheckResult evaluateCommanded(const TopicCheck& c, const std::map<std::string, std::vector<std::string>>& topics, const std::string& infoText,
                              bool gzAvailable, bool gzReceived) {
  CheckResult r;
  std::string types;
  if (!typeListed(topics, c, types)) {
    r.state = CheckState::Fail;
    r.detail = "The topic is not in `ros2 topic list`: the bridge did not create it.";
    return r;
  }
  if (!hasType(topics, c)) {
    r.state = CheckState::Fail;
    r.detail = "The topic exists with the type " + types + " instead of " + c.rosType + ".";
    return r;
  }
  const int subs = parseSubscriptionCount(infoText);
  if (subs == 0) {
    r.state = CheckState::Fail;
    r.detail = "Nothing in ROS is subscribed to the topic, so the bridge is not listening for commands.";
    return r;
  }
  if (c.payload.empty()) {
    r.state = CheckState::Skipped;
    r.detail = "The bridge listens, but no harmless sample message is known for " + c.rosType + ", so delivery to Gazebo was not tried.";
    return r;
  }
  if (!gzAvailable) {
    r.state = CheckState::Pass;
    r.detail = "The bridge listens. The `gz` command was not found, so delivery to Gazebo was not verified.";
    return r;
  }
  if (!gzReceived) {
    r.state = CheckState::Fail;
    r.detail = "A message was published in ROS, but nothing arrived on the Gazebo topic " + c.gzTopic + ".";
    return r;
  }
  r.state = CheckState::Pass;
  r.detail = "A ROS message reached Gazebo on " + c.gzTopic + ".";
  return r;
}

std::vector<std::string> launchCommand(const std::string& packageName) { return {"ros2", "launch", packageName, "sim.launch.py"}; }
std::vector<std::string> topicListCommand() { return {"ros2", "topic", "list", "-t"}; }
std::vector<std::string> hzCommand(const std::string& topic) { return {"ros2", "topic", "hz", topic}; }
std::vector<std::string> infoCommand(const std::string& topic) { return {"ros2", "topic", "info", topic}; }
std::vector<std::string> publishCommand(const TopicCheck& c) {
  return {"ros2", "topic", "pub", "-r", "5", "-t", "10", c.rosTopic, c.rosType, c.payload};
}
std::vector<std::string> gzEchoCommand(const std::string& gzTopic) { return {"gz", "topic", "-e", "-t", gzTopic, "-n", "1"}; }

}  // namespace gzb
