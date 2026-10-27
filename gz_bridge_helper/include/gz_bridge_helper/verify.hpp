// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// The logic of "is the bridge alive?" without any process handling: which checks to run for a
// plan, which commands to run for them, and how to read what those commands print. The GUI
// starts the processes; everything it has to decide is decided here so that it can be tested.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "gz_bridge_helper/plan.hpp"

namespace gzb {

/// One bridged topic to be checked.
struct TopicCheck {
  std::string rosTopic, rosType, gzTopic, gzType;
  Direction direction = Direction::GzToRos;
  std::string description;
  /// A message to publish for a ROS -> Gazebo topic (YAML for `ros2 topic pub`); empty when no
  /// harmless sample is known for the type.
  std::string payload;
  bool publishesInRos() const { return direction != Direction::RosToGz; }
  bool listensInRos() const { return direction != Direction::GzToRos; }
};

enum class CheckState { Pending, Running, Pass, Fail, Skipped };
const char* checkStateName(CheckState s);

struct CheckResult {
  CheckState state = CheckState::Pending;
  std::string detail;
};

/// The enabled topic entries of the plan, in order. Services are not checked.
std::vector<TopicCheck> checksFromPlan(const Plan& plan);

/// A harmless message of the given ROS type, or "".
std::string samplePayload(const std::string& rosType);

/// "ros2 topic list -t" -> { "/clock": ["rosgraph_msgs/msg/Clock"], ... }
std::map<std::string, std::vector<std::string>> parseTopicList(const std::string& text);
/// The last "average rate:" of "ros2 topic hz"; false when there is none.
bool parseHzAverage(const std::string& text, double& hz);
/// Number of subscriptions in the text of "ros2 topic info"; -1 when it cannot be read.
int parseSubscriptionCount(const std::string& text);
/// True when "ros2 topic echo" printed at least one message (a "---" separator line).
bool echoShowsMessage(const std::string& text);

/// Outcome of checking a topic that Gazebo publishes (the bridge publishes it in ROS).
CheckResult evaluatePublished(const TopicCheck& c, const std::map<std::string, std::vector<std::string>>& topics, const std::string& hzText);
/// Outcome of checking a topic ROS sends into Gazebo. `gzReceived` is true when `gz topic -e -n 1`
/// ended by itself after one message (its exit status, not its text: protobuf prints a message
/// whose fields are all zero as nothing at all). `gzAvailable` is false when `gz` does not exist.
CheckResult evaluateCommanded(const TopicCheck& c, const std::map<std::string, std::vector<std::string>>& topics, const std::string& infoText,
                              bool gzAvailable, bool gzReceived);

/// The command lines (program, arguments) the GUI runs.
std::vector<std::string> launchCommand(const std::string& packageName);
std::vector<std::string> topicListCommand();
std::vector<std::string> hzCommand(const std::string& topic);
std::vector<std::string> infoCommand(const std::string& topic);
std::vector<std::string> publishCommand(const TopicCheck& c);
std::vector<std::string> gzEchoCommand(const std::string& gzTopic);

}  // namespace gzb
