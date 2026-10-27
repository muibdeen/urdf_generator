// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/bridge.hpp"

#include <set>
#include <sstream>

#include "gz_bridge_helper/types.hpp"

namespace gzb {

const char* directionName(Direction d) {
  switch (d) {
    case Direction::Bidirectional: return "BIDIRECTIONAL";
    case Direction::GzToRos: return "GZ_TO_ROS";
    case Direction::RosToGz: return "ROS_TO_GZ";
  }
  return "BIDIRECTIONAL";
}

bool parseDirection(const std::string& s, Direction& out) {
  if (s == "BIDIRECTIONAL") out = Direction::Bidirectional;
  else if (s == "GZ_TO_ROS") out = Direction::GzToRos;
  else if (s == "ROS_TO_GZ") out = Direction::RosToGz;
  else return false;
  return true;
}

const char* directionSymbol(Direction d) {
  switch (d) {
    case Direction::Bidirectional: return "@";
    case Direction::GzToRos: return "[";
    case Direction::RosToGz: return "]";
  }
  return "@";
}

std::string topicParent(const std::string& topic) {
  const size_t slash = topic.find_last_of('/');
  if (slash == std::string::npos || slash == 0) return "/";
  return topic.substr(0, slash);
}

std::string absoluteTopic(const std::string& topic) { return startsWith(topic, "/") ? topic : "/" + topic; }

std::string bridgeYaml(const std::vector<BridgeEntry>& entries, const std::string& header) {
  std::ostringstream out;
  if (!header.empty()) out << header << "\n";
  for (const BridgeEntry& e : entries) {
    if (!e.enabled) continue;
    if (!e.purpose.empty()) out << "# " << e.purpose << "\n";
    if (e.isService()) {
      out << "- service_name: " << yamlScalar(e.serviceName) << "\n";
      out << "  ros_type_name: " << yamlScalar(e.rosType) << "\n";
      out << "  gz_req_type_name: " << yamlScalar(e.gzRequestType) << "\n";
      out << "  gz_rep_type_name: " << yamlScalar(e.gzReplyType) << "\n\n";
      continue;
    }
    if (e.rosTopic == e.gzTopic) {
      out << "- topic_name: " << yamlScalar(e.rosTopic) << "\n";
    } else {
      out << "- ros_topic_name: " << yamlScalar(e.rosTopic) << "\n";
      out << "  gz_topic_name: " << yamlScalar(e.gzTopic) << "\n";
    }
    out << "  ros_type_name: " << yamlScalar(e.rosType) << "\n";
    out << "  gz_type_name: " << yamlScalar(e.gzType) << "\n";
    out << "  direction: " << directionName(e.direction) << "\n";
    if (!e.qos.empty()) out << "  qos_profile: " << e.qos << "\n";
    if (e.subscriberQueue >= 0) out << "  subscriber_queue: " << e.subscriberQueue << "\n";
    if (e.publisherQueue >= 0) out << "  publisher_queue: " << e.publisherQueue << "\n";
    if (e.lazy) out << "  lazy: true\n";
    if (!e.frameId.empty()) out << "  frame_id: " << yamlScalar(e.frameId) << "\n";
    out << "\n";
  }
  return out.str();
}

void validateEntry(const BridgeEntry& e, const std::string& where, Issues& issues) {
  const std::string name = e.isService() ? e.serviceName : (e.rosTopic.empty() ? e.gzTopic : e.rosTopic);
  const std::string label = "Bridge entry '" + name + "': ";
  if (e.isService()) {
    if (e.gzRequestType.empty() || e.gzReplyType.empty()) {
      addIssue(issues, Severity::Error, where, label + "a service needs both gz_req_type_name and gz_rep_type_name; ros_gz_bridge skips this entry.");
    }
    if (e.rosType.empty()) addIssue(issues, Severity::Error, where, label + "ros_type_name is missing.");
    return;
  }
  if (e.rosTopic.empty() && e.gzTopic.empty()) {
    addIssue(issues, Severity::Error, where, "A bridge entry has no topic name (topic_name, ros_topic_name or gz_topic_name).");
    return;
  }
  if (e.rosType.empty() || e.gzType.empty()) {
    addIssue(issues, Severity::Error, where, label + "ros_type_name and gz_type_name must both be set; ros_gz_bridge skips this entry.");
    return;
  }
  if (!knownRosType(e.rosType)) {
    addIssue(issues, Severity::Warning, where,
             label + "'" + e.rosType + "' is not in the Jazzy ros_gz_bridge type table (a misspelling, or a type from a package you added).");
  } else if (!knownGzType(e.gzType)) {
    addIssue(issues, Severity::Warning, where, label + "'" + e.gzType + "' is not in the Jazzy ros_gz_bridge type table.");
  } else if (!isKnownPair(e.rosType, e.gzType)) {
    std::string want;
    for (const std::string& g : gzTypesFor(e.rosType)) want += (want.empty() ? "" : " or ") + g;
    addIssue(issues, Severity::Error, where,
             label + e.rosType + " cannot be converted to " + e.gzType + "; the bridge pairs it with " + want + ".");
  }
}

}  // namespace gzb
