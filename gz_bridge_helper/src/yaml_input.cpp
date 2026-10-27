// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/yaml_input.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>

namespace gzb {

const char* yamlKindName(YamlKind k) {
  switch (k) {
    case YamlKind::Controllers: return "ros2_control controllers";
    case YamlKind::BridgeConfig: return "ros_gz_bridge config";
    case YamlKind::Overrides: return "overrides";
  }
  return "?";
}

std::string ParsedYaml::kindText() const {
  if (kinds.empty()) return "unrecognised";
  std::string out;
  for (YamlKind k : {YamlKind::Controllers, YamlKind::BridgeConfig, YamlKind::Overrides}) {
    if (kinds.count(k)) out += (out.empty() ? "" : " + ") + std::string(yamlKindName(k));
  }
  return out;
}

bool ControllersConfig::isBroadcaster(const ControllerSpec& c) const {
  return c.type.find("Broadcaster") != std::string::npos;
}

namespace {

const std::vector<std::string> kOverrideKeys = {"package_name", "model_name", "world", "spawn", "sensors",
                                                "drive", "joints", "bridge", "exclude", "use_sim_time"};
const std::vector<std::string> kBridgeKeys = {"service_name", "topic_name", "ros_topic_name", "gz_topic_name",
                                              "ros_type_name", "gz_type_name", "direction", "publisher_queue",
                                              "subscriber_queue", "qos_profile", "lazy", "gz_req_type_name",
                                              "gz_rep_type_name", "frame_id"};
const std::vector<std::string> kQosProfiles = {"CLOCK", "SENSOR_DATA", "PARAMETERS", "SERVICES",
                                               "PARAMETER_EVENTS", "ROSOUT", "SYSTEM_DEFAULT", "BEST_AVAILABLE"};

/// "/ns/controller_manager" -> "controller_manager"; "/**" -> "**".
std::string lastComponent(const std::string& key) {
  std::string last;
  for (const std::string& part : split(key, '/')) {
    if (!part.empty()) last = part;
  }
  return last;
}

struct Reader {
  Issues& issues;
  std::string where;
  std::string file;  // the file's name, for the origin of what is read
  void error(const std::string& m) { addIssue(issues, Severity::Error, where, m); }
  void warn(const std::string& m) { addIssue(issues, Severity::Warning, where, m); }
};

bool isScalarText(const YAML::Node& n) { return n.IsScalar(); }

/// Warns about keys of `map` that are not in `allowed` and suggests the closest known key.
/// `path` is put in front of the key in the message ("world."); `context` names where the map is.
void checkKeys(Reader& r, const YAML::Node& map, const std::vector<std::string>& allowed, const std::string& path,
               const std::string& context = "") {
  if (!map.IsMap()) return;
  for (auto it = map.begin(); it != map.end(); ++it) {
    if (!isScalarText(it->first)) continue;
    const std::string key = it->first.as<std::string>();
    if (std::find(allowed.begin(), allowed.end(), key) != allowed.end()) continue;
    const std::string hint = closestMatch(key, allowed);
    r.warn("Unknown key '" + path + key + "'" + (context.empty() ? "" : " in " + context) + " is ignored" +
           (hint.empty() ? "." : "; did you mean '" + hint + "'?"));
  }
}

template <class T>
void getOpt(Reader& r, const YAML::Node& map, const char* key, const std::string& path, const char* expected,
            std::optional<T>& out) {
  if (!map.IsMap()) return;
  const YAML::Node v = map[key];
  if (!v) return;
  try {
    out = v.template as<T>();
  } catch (const YAML::Exception&) {
    r.error("'" + path + key + "' must be " + expected + ".");
  }
}

bool readVec3(const YAML::Node& v, Vec3& out) {
  if (!v.IsSequence() || v.size() != 3) return false;
  try {
    out = {v[0].as<double>(), v[1].as<double>(), v[2].as<double>()};
  } catch (const YAML::Exception&) {
    return false;
  }
  return true;
}

std::vector<std::string> stringList(const YAML::Node& v) {
  std::vector<std::string> out;
  if (v.IsScalar()) {
    out.push_back(v.as<std::string>());
  } else if (v.IsSequence()) {
    for (const YAML::Node& e : v) {
      if (e.IsScalar()) out.push_back(e.as<std::string>());
    }
  }
  return out;
}

// ------------------------------------------------------------------------ bridge entries

/// Mirrors ros_gz_bridge's parseEntry: false when the real bridge would skip the entry.
bool parseBridgeEntry(Reader& r, const YAML::Node& node, const std::string& origin, BridgeEntry& e) {
  if (!node.IsMap()) {
    r.error("A bridge entry must be a YAML map (key: value lines), but this one is not; ros_gz_bridge skips it.");
    return false;
  }
  auto str = [&](const char* k) -> std::string {
    const YAML::Node v = node[k];
    if (!v || !v.IsScalar()) return "";
    return v.as<std::string>();
  };
  const std::string topic = str("topic_name"), rosT = str("ros_topic_name"), gzT = str("gz_topic_name");
  const std::string label = !topic.empty() ? topic : (!rosT.empty() ? rosT : (!gzT.empty() ? gzT : str("service_name")));
  const std::string prefix = "Bridge entry '" + label + "': ";
  checkKeys(r, node, kBridgeKeys, "", "bridge entry '" + label + "'");
  if (!topic.empty() && !rosT.empty()) {
    r.error(prefix + "topic_name and ros_topic_name are mutually exclusive; ros_gz_bridge skips this entry.");
    return false;
  }
  if (!topic.empty() && !gzT.empty()) {
    r.error(prefix + "topic_name and gz_topic_name are mutually exclusive; ros_gz_bridge skips this entry.");
    return false;
  }
  e = BridgeEntry();
  e.origin = origin;
  e.rosType = str("ros_type_name");
  e.serviceName = str("service_name");
  if (!e.serviceName.empty()) {
    e.gzRequestType = str("gz_req_type_name");
    e.gzReplyType = str("gz_rep_type_name");
  } else {
    if (!topic.empty()) {
      e.rosTopic = e.gzTopic = topic;
    } else if (!rosT.empty() && gzT.empty()) {
      e.rosTopic = e.gzTopic = rosT;
    } else if (!gzT.empty() && rosT.empty()) {
      e.rosTopic = e.gzTopic = gzT;
    } else {
      e.rosTopic = rosT;
      e.gzTopic = gzT;
    }
    e.gzType = str("gz_type_name");
    e.frameId = str("frame_id");
  }
  if (node["direction"]) {
    e.direction = Direction::Bidirectional;
    if (!parseDirection(str("direction"), e.direction)) {
      r.error(prefix + "direction '" + str("direction") + "' is not BIDIRECTIONAL, GZ_TO_ROS or ROS_TO_GZ; ros_gz_bridge skips this entry.");
      return false;
    }
  } else {
    e.direction = Direction::Bidirectional;  // the bridge's own default
  }
  if (node["qos_profile"]) {
    e.qos = str("qos_profile");
    if (!e.qos.empty() && std::find(kQosProfiles.begin(), kQosProfiles.end(), e.qos) == kQosProfiles.end()) {
      r.error(prefix + "qos_profile '" + e.qos + "' is not one of CLOCK, SENSOR_DATA, PARAMETERS, SERVICES, PARAMETER_EVENTS, ROSOUT, SYSTEM_DEFAULT, BEST_AVAILABLE; ros_gz_bridge skips this entry.");
      return false;
    }
  }
  try {
    if (node["publisher_queue"]) e.publisherQueue = node["publisher_queue"].as<int>();
    if (node["subscriber_queue"]) e.subscriberQueue = node["subscriber_queue"].as<int>();
    if (node["lazy"]) e.lazy = node["lazy"].as<bool>();
  } catch (const YAML::Exception&) {
    r.error(prefix + "publisher_queue and subscriber_queue must be integers and lazy must be true or false; ros_gz_bridge skips this entry.");
    return false;
  }
  Issues local;
  validateEntry(e, r.where, local);
  bool bad = false;
  for (const Issue& i : local) {
    r.issues.push_back(i);
    if (i.severity == Severity::Error) bad = true;
  }
  return !bad;
}

void parseBridgeList(Reader& r, const YAML::Node& seq, const std::string& origin, std::vector<BridgeEntry>& out) {
  for (const YAML::Node& n : seq) {
    BridgeEntry e;
    if (parseBridgeEntry(r, n, origin, e)) out.push_back(e);
  }
}

// ----------------------------------------------------------------------------- controllers

bool isJointKey(const std::string& k) {
  return k == "joints" || k == "joint" || endsWith(k, "_joint") || endsWith(k, "_joints") ||
         endsWith(k, "_joint_name") || endsWith(k, "_joint_names") || endsWith(k, "_wheel_names") ||
         endsWith(k, "_wheel_name");
}

void parseControllers(Reader& r, const YAML::Node& top, ControllersConfig& out) {
  YAML::Node manager;
  for (auto it = top.begin(); it != top.end(); ++it) {
    if (it->first.IsScalar() && lastComponent(it->first.as<std::string>()) == "controller_manager") manager = it->second;
  }
  const YAML::Node params = manager["ros__parameters"];
  if (!params || !params.IsMap()) {
    r.error("The controller_manager section has no ros__parameters map, so it lists no controllers.");
    return;
  }
  try {
    if (params["update_rate"]) out.updateRate = params["update_rate"].as<double>();
  } catch (const YAML::Exception&) {
    r.error("controller_manager update_rate must be a number.");
  }
  for (auto it = params.begin(); it != params.end(); ++it) {
    if (!it->first.IsScalar() || !it->second.IsMap() || !it->second["type"]) continue;
    ControllerSpec c;
    c.name = it->first.as<std::string>();
    c.type = it->second["type"].IsScalar() ? it->second["type"].as<std::string>() : "";
    if (c.type.empty()) {
      r.error("Controller '" + c.name + "' has no type.");
      continue;
    }
    // Its own parameter section: a top-level key with the controller's name.
    for (auto sec = top.begin(); sec != top.end(); ++sec) {
      if (!sec->first.IsScalar() || lastComponent(sec->first.as<std::string>()) != c.name) continue;
      const YAML::Node own = sec->second["ros__parameters"];
      if (!own || !own.IsMap()) continue;
      c.hasParameters = true;
      for (auto p = own.begin(); p != own.end(); ++p) {
        if (!p->first.IsScalar()) continue;
        const std::string key = p->first.as<std::string>();
        if (key == "command_interfaces") c.commandInterfaces = stringList(p->second);
        else if (key == "state_interfaces") c.stateInterfaces = stringList(p->second);
        else if (isJointKey(key)) {
          for (const std::string& j : stringList(p->second)) c.jointRefs.emplace_back(key, j);
        }
      }
    }
    out.controllers.push_back(c);
  }
  if (out.controllers.empty()) r.warn("The controller_manager section does not define any controller (an entry with a 'type').");
}

// ------------------------------------------------------------------------------- overrides

void parseOverrides(Reader& r, const YAML::Node& top, Overrides& o, bool alsoControllers) {
  if (!alsoControllers) checkKeys(r, top, kOverrideKeys, "");
  getOpt(r, top, "package_name", "", "text", o.packageName);
  getOpt(r, top, "model_name", "", "text", o.modelName);
  getOpt(r, top, "use_sim_time", "", "true or false", o.useSimTime);

  if (const YAML::Node w = top["world"]) {
    if (!w.IsMap()) {
      r.error("'world' must be a map (name, step_size, gravity, spherical_coordinates).");
    } else {
      checkKeys(r, w, {"name", "step_size", "gravity", "spherical_coordinates"}, "world.");
      getOpt(r, w, "name", "world.", "text", o.worldName);
      getOpt(r, w, "step_size", "world.", "a number (seconds)", o.stepSize);
      if (o.stepSize && !(*o.stepSize > 0)) {
        r.error("'world.step_size' must be greater than 0.");
        o.stepSize.reset();
      }
      if (w["gravity"]) {
        Vec3 g;
        if (readVec3(w["gravity"], g)) o.gravity = g;
        else r.error("'world.gravity' must be three numbers, for example [0, 0, -9.81].");
      }
      if (const YAML::Node s = w["spherical_coordinates"]) {
        if (!s.IsMap()) {
          r.error("'world.spherical_coordinates' must be a map (latitude, longitude, elevation, heading).");
        } else {
          checkKeys(r, s, {"latitude", "longitude", "elevation", "heading"}, "world.spherical_coordinates.");
          Overrides::Spherical sc;
          std::optional<double> lat, lon, el, hd;
          getOpt(r, s, "latitude", "world.spherical_coordinates.", "a number (degrees)", lat);
          getOpt(r, s, "longitude", "world.spherical_coordinates.", "a number (degrees)", lon);
          getOpt(r, s, "elevation", "world.spherical_coordinates.", "a number (metres)", el);
          getOpt(r, s, "heading", "world.spherical_coordinates.", "a number (degrees)", hd);
          if (!lat || !lon) {
            r.error("'world.spherical_coordinates' needs at least latitude and longitude.");
          } else if (*lat < -90 || *lat > 90 || *lon < -180 || *lon > 180) {
            r.error("'world.spherical_coordinates' latitude must be within -90..90 and longitude within -180..180 degrees.");
          } else {
            sc.latitude = *lat;
            sc.longitude = *lon;
            sc.elevation = el.value_or(0);
            sc.heading = hd.value_or(0);
            o.spherical = sc;
          }
        }
      }
    }
  }

  if (const YAML::Node s = top["spawn"]) {
    if (!s.IsMap()) {
      r.error("'spawn' must be a map (x, y, z, roll, pitch, yaw).");
    } else {
      checkKeys(r, s, {"x", "y", "z", "roll", "pitch", "yaw"}, "spawn.");
      getOpt(r, s, "x", "spawn.", "a number (metres)", o.spawnX);
      getOpt(r, s, "y", "spawn.", "a number (metres)", o.spawnY);
      getOpt(r, s, "z", "spawn.", "a number (metres)", o.spawnZ);
      getOpt(r, s, "roll", "spawn.", "a number (radians)", o.spawnRoll);
      getOpt(r, s, "pitch", "spawn.", "a number (radians)", o.spawnPitch);
      getOpt(r, s, "yaw", "spawn.", "a number (radians)", o.spawnYaw);
    }
  }

  if (const YAML::Node sensors = top["sensors"]) {
    if (!sensors.IsMap()) {
      r.error("'sensors' must be a map from a sensor name to its settings.");
    } else {
      for (auto it = sensors.begin(); it != sensors.end(); ++it) {
        if (!it->first.IsScalar()) continue;
        const std::string name = it->first.as<std::string>();
        const std::string path = "sensors." + name + ".";
        if (!it->second.IsMap()) {
          r.error("'sensors." + name + "' must be a map (enabled, ros_topic, update_rate, ...).");
          continue;
        }
        checkKeys(r, it->second, {"enabled", "ros_topic", "frame_id", "update_rate", "width", "height",
                                   "horizontal_fov", "samples", "range_min", "range_max"}, path);
        SensorOverride so;
        getOpt(r, it->second, "enabled", path, "true or false", so.enabled);
        getOpt(r, it->second, "ros_topic", path, "text", so.rosTopic);
        getOpt(r, it->second, "frame_id", path, "text", so.frameId);
        getOpt(r, it->second, "update_rate", path, "a number (Hz)", so.updateRate);
        getOpt(r, it->second, "width", path, "an integer (pixels)", so.width);
        getOpt(r, it->second, "height", path, "an integer (pixels)", so.height);
        getOpt(r, it->second, "horizontal_fov", path, "a number (radians)", so.horizontalFov);
        getOpt(r, it->second, "samples", path, "an integer", so.samples);
        getOpt(r, it->second, "range_min", path, "a number (metres)", so.rangeMin);
        getOpt(r, it->second, "range_max", path, "a number (metres)", so.rangeMax);
        if (so.updateRate && !(*so.updateRate > 0)) {
          r.error("'" + path + "update_rate' must be greater than 0.");
          so.updateRate.reset();
        }
        if ((so.width && *so.width < 1) || (so.height && *so.height < 1) || (so.samples && *so.samples < 1)) {
          r.error("'" + path + "' width, height and samples must be at least 1.");
          so.width.reset();
          so.height.reset();
          so.samples.reset();
        }
        if (so.rangeMin && so.rangeMax && !(*so.rangeMin < *so.rangeMax)) {
          r.error("'" + path + "range_min' must be smaller than range_max.");
          so.rangeMin.reset();
          so.rangeMax.reset();
        }
        o.sensors[name] = so;
      }
    }
  }

  if (const YAML::Node d = top["drive"]) {
    if (!d.IsMap()) {
      r.error("'drive' must be a map (enabled, wheel_separation, wheel_radius, ...).");
    } else {
      checkKeys(r, d, {"enabled", "wheel_separation", "wheel_radius", "ros_cmd_vel_topic", "ros_odom_topic",
                       "max_linear_velocity", "max_angular_velocity"}, "drive.");
      getOpt(r, d, "enabled", "drive.", "true or false", o.drive.enabled);
      getOpt(r, d, "wheel_separation", "drive.", "a number (metres)", o.drive.wheelSeparation);
      getOpt(r, d, "wheel_radius", "drive.", "a number (metres)", o.drive.wheelRadius);
      getOpt(r, d, "ros_cmd_vel_topic", "drive.", "text", o.drive.rosCmdVelTopic);
      getOpt(r, d, "ros_odom_topic", "drive.", "text", o.drive.rosOdomTopic);
      getOpt(r, d, "max_linear_velocity", "drive.", "a number (m/s)", o.drive.maxLinearVelocity);
      getOpt(r, d, "max_angular_velocity", "drive.", "a number (rad/s)", o.drive.maxAngularVelocity);
      if ((o.drive.wheelSeparation && !(*o.drive.wheelSeparation > 0)) || (o.drive.wheelRadius && !(*o.drive.wheelRadius > 0))) {
        r.error("'drive.wheel_separation' and 'drive.wheel_radius' must be greater than 0.");
        o.drive.wheelSeparation.reset();
        o.drive.wheelRadius.reset();
      }
    }
  }

  if (const YAML::Node j = top["joints"]) {
    if (!j.IsMap()) {
      r.error("'joints' must be a map (state_publisher, position_control, p_gain, i_gain, d_gain).");
    } else {
      checkKeys(r, j, {"state_publisher", "position_control", "p_gain", "i_gain", "d_gain"}, "joints.");
      getOpt(r, j, "state_publisher", "joints.", "true or false", o.jointStates);
      if (j["position_control"]) {
        if (j["position_control"].IsSequence() || j["position_control"].IsScalar()) o.positionControl = stringList(j["position_control"]);
        else r.error("'joints.position_control' must be a list of joint names.");
      }
      getOpt(r, j, "p_gain", "joints.", "a number", o.pGain);
      getOpt(r, j, "i_gain", "joints.", "a number", o.iGain);
      getOpt(r, j, "d_gain", "joints.", "a number", o.dGain);
    }
  }

  if (const YAML::Node b = top["bridge"]) {
    if (!b.IsSequence()) r.error("'bridge' must be a list of bridge entries (the same format as a ros_gz_bridge config).");
    else parseBridgeList(r, b, "overrides " + r.file, o.bridge);
  }
  if (const YAML::Node e = top["exclude"]) {
    if (e.IsSequence() || e.IsScalar()) o.exclude = stringList(e);
    else r.error("'exclude' must be a list of ROS topic names.");
  }
}

}  // namespace

// ----------------------------------------------------------------------------- the entry point

ParsedYaml parseYaml(const std::string& path, const std::string& text, Issues& issues) {
  ParsedYaml out;
  out.path = path;
  out.text = text;
  Reader r{issues, "YAML " + baseName(path), baseName(path)};

  YAML::Node top;
  try {
    top = YAML::Load(text);
  } catch (const YAML::Exception& e) {
    r.error(std::string("This is not valid YAML: ") + e.what() + " (line " + std::to_string(e.mark.line + 1) + ").");
    return out;
  }
  out.ok = true;

  if (top.IsNull() || !top) {
    r.warn("The file is empty, so it changes nothing.");
    return out;
  }

  if (top.IsSequence()) {
    bool bridgeLike = false;
    for (const YAML::Node& n : top) {
      if (!n.IsMap()) continue;
      for (const std::string& k : kBridgeKeys) {
        if (n[k]) bridgeLike = true;
      }
    }
    if (bridgeLike || top.size() == 0) {
      out.kinds.insert(YamlKind::BridgeConfig);
      if (top.size() == 0) r.warn("The bridge config is an empty list.");
      parseBridgeList(r, top, "file " + baseName(path), out.bridge);
    } else {
      r.error("A top-level list is read as a ros_gz_bridge config, but none of its entries has bridge keys (topic_name, ros_type_name, ...).");
    }
    return out;
  }

  if (!top.IsMap()) {
    r.error("The top level of the file must be a map (settings, controllers) or a list (bridge entries).");
    return out;
  }

  bool hasManager = false, hasOverrideKey = false;
  std::string firstKeys;
  int shown = 0;
  for (auto it = top.begin(); it != top.end(); ++it) {
    if (!it->first.IsScalar()) continue;
    const std::string key = it->first.as<std::string>();
    if (lastComponent(key) == "controller_manager") hasManager = true;
    if (std::find(kOverrideKeys.begin(), kOverrideKeys.end(), key) != kOverrideKeys.end()) hasOverrideKey = true;
    if (shown++ < 6) firstKeys += (firstKeys.empty() ? "" : ", ") + key;
  }
  if (hasManager) {
    out.kinds.insert(YamlKind::Controllers);
    parseControllers(r, top, out.controllers);
  }
  if (hasOverrideKey) {
    out.kinds.insert(YamlKind::Overrides);
    parseOverrides(r, top, out.overrides, hasManager);
  }
  if (out.kinds.empty()) {
    const std::string hint = closestMatch(firstKeys.substr(0, firstKeys.find(',')), kOverrideKeys);
    r.warn("Could not tell what this file is: its top-level keys (" + firstKeys +
           ") are neither a controller_manager section nor override keys" +
           (hint.empty() ? "." : "; is '" + hint + "' meant?") + " It is ignored.");
  }
  return out;
}

std::string overridesTemplate() {
  return R"(# Overrides for gz_bridge_helper. Every key is optional; delete what you do not need.
# Anything written here wins over what the helper detected in the URDF.

package_name: my_robot_sim      # name of the generated ROS 2 package
model_name: my_robot            # name of the model inside Gazebo (default: the URDF's robot name)
use_sim_time: true

world:
  name: sim_world
  step_size: 0.001              # physics step in seconds
  gravity: [0.0, 0.0, -9.81]
  spherical_coordinates:        # needed for a GPS; where (0, 0, 0) of the world is on Earth
    latitude: 48.8566
    longitude: 2.3522
    elevation: 35.0
    heading: 0.0

spawn:                          # where the robot appears
  x: 0.0
  y: 0.0
  z: 0.1
  yaw: 0.0

sensors:                        # keyed by sensor name (the link name without '_link')
  camera:
    enabled: true
    ros_topic: /camera/image_raw
    update_rate: 30.0
    width: 640
    height: 480
    horizontal_fov: 1.047
  lidar:
    samples: 360
    range_min: 0.12
    range_max: 12.0

drive:
  wheel_separation: 0.4         # metres; detected from the URDF if left out
  wheel_radius: 0.1
  ros_cmd_vel_topic: /cmd_vel
  ros_odom_topic: /odom
  max_linear_velocity: 1.0
  max_angular_velocity: 2.0

joints:
  state_publisher: true         # /joint_states from Gazebo
  position_control: []          # joint names to command with a Float64 on /joint/<name>/cmd_pos
  p_gain: 10.0
  i_gain: 0.0
  d_gain: 0.5

bridge: []                      # extra bridge entries, same format as a ros_gz_bridge config
exclude: []                     # ROS topic names that must not be bridged
)";
}

}  // namespace gzb
