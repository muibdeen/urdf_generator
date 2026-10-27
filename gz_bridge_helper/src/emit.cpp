// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/emit.hpp"

#include <algorithm>
#include <regex>
#include <set>
#include <sstream>

namespace gzb {

const GeneratedFile* Generated::file(const std::string& path) const {
  for (const GeneratedFile& f : files) {
    if (f.path == path) return &f;
  }
  return nullptr;
}

namespace {

const char* kBanner = "gz_bridge_helper";

std::string regexEscape(const std::string& s) {
  std::string out;
  for (char c : s) {
    if (std::string("\\^$.|?*+()[]{}").find(c) != std::string::npos) out.push_back('\\');
    out.push_back(c);
  }
  return out;
}

std::string xmlText(double v) { return num(v); }

std::string bridgeFileName(const Plan& p) { return p.packageName + "_bridge.yaml"; }
std::string urdfFileName(const Plan& p) { return p.modelName + ".urdf"; }
std::string worldFileName(const Plan& p) { return p.worldName + ".sdf"; }

std::string plugin(const std::string& filename, const std::string& name, const std::vector<std::pair<std::string, std::string>>& params,
                   const std::string& indent = "    ") {
  std::ostringstream o;
  o << indent << "<plugin filename=\"" << filename << "\" name=\"" << name << "\">\n";
  for (const auto& kv : params) o << indent << "  <" << kv.first << ">" << xmlEscape(kv.second) << "</" << kv.first << ">\n";
  o << indent << "</plugin>\n";
  return o.str();
}

std::string gazeboPlugin(const std::string& comment, const std::string& filename, const std::string& name,
                         const std::vector<std::pair<std::string, std::string>>& params) {
  std::ostringstream o;
  if (!comment.empty()) o << "  <!-- " << comment << " -->\n";
  o << "  <gazebo>\n" << plugin(filename, name, params) << "  </gazebo>\n";
  return o.str();
}

std::string sensorBlock(const PlanSensor& s) {
  std::ostringstream o;
  o << "  <!-- " << sensorKindLabel(s.kind) << " '" << s.name << "' on " << s.link << ": published on " << s.gzTopic << " -->\n";
  o << "  <gazebo reference=\"" << s.link << "\">\n";
  o << "    <sensor name=\"" << s.name << "\" type=\"" << sdfSensorType(s.kind) << "\">\n";
  o << "      <always_on>true</always_on>\n";
  o << "      <update_rate>" << xmlText(s.updateRate) << "</update_rate>\n";
  o << "      <topic>" << s.gzTopic << "</topic>\n";
  o << "      <gz_frame_id>" << s.frameLink << "</gz_frame_id>\n";
  switch (s.kind) {
    case SensorKind::Camera:
      o << "      <camera>\n"
        << "        <horizontal_fov>" << xmlText(s.horizontalFov) << "</horizontal_fov>\n"
        << "        <image>\n          <width>" << s.width << "</width>\n          <height>" << s.height << "</height>\n        </image>\n"
        << "        <clip>\n          <near>0.1</near>\n          <far>100</far>\n        </clip>\n"
        << "      </camera>\n";
      break;
    case SensorKind::Lidar:
      o << "      <lidar>\n"
        << "        <scan>\n"
        << "          <horizontal>\n            <samples>" << s.samples << "</samples>\n            <resolution>1</resolution>\n"
        << "            <min_angle>-3.14159</min_angle>\n            <max_angle>3.14159</max_angle>\n          </horizontal>\n"
        << "          <vertical>\n            <samples>1</samples>\n            <resolution>1</resolution>\n"
        << "            <min_angle>0</min_angle>\n            <max_angle>0</max_angle>\n          </vertical>\n"
        << "        </scan>\n"
        << "        <range>\n          <min>" << xmlText(s.rangeMin) << "</min>\n          <max>" << xmlText(s.rangeMax)
        << "</max>\n          <resolution>0.01</resolution>\n        </range>\n"
        << "      </lidar>\n";
      break;
    default: break;
  }
  o << "    </sensor>\n  </gazebo>\n";
  return o.str();
}

std::string driveBlocks(const Plan& p, const PlanDrive& d) {
  std::vector<std::pair<std::string, std::string>> params;
  std::string file, name, comment;
  auto joints = [&](const char* tag, const std::vector<std::string>& v) {
    for (const std::string& j : v) params.emplace_back(tag, j);
  };
  switch (d.d.kind) {
    case DriveKind::Diff:
      file = "gz-sim-diff-drive-system";
      name = "gz::sim::systems::DiffDrive";
      comment = "DiffDrive: turns Twist messages on " + d.gzCmdVel + " into wheel velocities and publishes odometry and the odom transform";
      joints("left_joint", d.d.left);
      joints("right_joint", d.d.right);
      params.emplace_back("wheel_separation", xmlText(d.d.wheelSeparation));
      params.emplace_back("wheel_radius", xmlText(d.d.wheelRadius));
      break;
    case DriveKind::Mecanum:
      file = "gz-sim-mecanum-drive-system";
      name = "gz::sim::systems::MecanumDrive";
      comment = "MecanumDrive: turns Twist messages on " + d.gzCmdVel + " (x, y and yaw) into the velocity of the four wheels";
      params.emplace_back("front_left_joint", d.d.frontLeft);
      params.emplace_back("front_right_joint", d.d.frontRight);
      params.emplace_back("back_left_joint", d.d.backLeft);
      params.emplace_back("back_right_joint", d.d.backRight);
      params.emplace_back("wheel_separation", xmlText(d.d.wheelSeparation));
      params.emplace_back("wheelbase", xmlText(d.d.wheelBase));
      params.emplace_back("wheel_radius", xmlText(d.d.wheelRadius));
      break;
    case DriveKind::Ackermann:
      file = "gz-sim-ackermann-steering-system";
      name = "gz::sim::systems::AckermannSteering";
      comment = "AckermannSteering: turns Twist messages on " + d.gzCmdVel + " into wheel velocity and steering angles";
      joints("left_joint", d.d.left);
      joints("right_joint", d.d.right);
      params.emplace_back("left_steering_joint", d.d.leftSteering);
      params.emplace_back("right_steering_joint", d.d.rightSteering);
      params.emplace_back("wheel_separation", xmlText(d.d.wheelSeparation));
      params.emplace_back("kingpin_width", xmlText(d.d.kingpinWidth));
      params.emplace_back("wheel_base", xmlText(d.d.wheelBase));
      params.emplace_back("wheel_radius", xmlText(d.d.wheelRadius));
      if (d.d.steeringLimit > 0) params.emplace_back("steering_limit", xmlText(d.d.steeringLimit));
      break;
  }
  params.emplace_back("topic", d.gzCmdVel);
  params.emplace_back("odom_topic", d.gzOdom);
  params.emplace_back("tf_topic", d.gzTf);
  params.emplace_back("frame_id", d.odomFrame);
  params.emplace_back("child_frame_id", d.baseFrame);
  params.emplace_back("odom_publish_frequency", "50");
  if (d.maxLinear > 0) params.emplace_back("max_linear_velocity", xmlText(d.maxLinear));
  if (d.maxAngular > 0) params.emplace_back("max_angular_velocity", xmlText(d.maxAngular));
  (void)p;
  return gazeboPlugin(comment, file, name, params);
}

std::string controllerParamsText(const Plan& p, size_t i) { return "@CONFIG_DIR@/" + p.controllerFiles[i].name; }

std::string additionsBlock(const Plan& p) {
  std::ostringstream o;
  if (p.mode == ControlMode::GzSystems) {
    if (p.jointStates) {
      o << gazeboPlugin("JointStatePublisher: publishes the position of the movable joints on " + p.gzJointStateTopic,
                        "gz-sim-joint-state-publisher-system", "gz::sim::systems::JointStatePublisher", {{"topic", p.gzJointStateTopic}});
    }
    for (const PlanDrive& d : p.drives) o << driveBlocks(p, d);
    if (p.odometry) {
      o << gazeboPlugin("OdometryPublisher: publishes the pose of the model as odometry on " + p.gzOdomTopic + " and as a transform on " + p.gzOdomTfTopic,
                        "gz-sim-odometry-publisher-system", "gz::sim::systems::OdometryPublisher",
                        {{"odom_frame", p.odomFrame}, {"robot_base_frame", p.baseFrame}, {"odom_publish_frequency", "50"},
                         {"odom_topic", p.gzOdomTopic}, {"tf_topic", p.gzOdomTfTopic}, {"dimensions", "3"}});
    }
    for (const PlanJointControl& c : p.jointControls) {
      o << gazeboPlugin("JointPositionController: a Float64 on " + c.gzTopic + " is the target angle; the gains are a PID you have to tune",
                        "gz-sim-joint-position-controller-system", "gz::sim::systems::JointPositionController",
                        {{"joint_name", c.joint}, {"topic", c.gzTopic}, {"p_gain", xmlText(p.pGain)}, {"i_gain", xmlText(p.iGain)}, {"d_gain", xmlText(p.dGain)}});
    }
  } else if (p.addControlPlugin) {
    std::vector<std::pair<std::string, std::string>> params;
    for (size_t i = 0; i < p.controllerFiles.size(); ++i) params.emplace_back("parameters", controllerParamsText(p, i));
    o << gazeboPlugin("gz_ros2_control: runs the controller manager inside Gazebo; @CONFIG_DIR@ is replaced by the launch file", "gz_ros2_control-system",
                      "gz_ros2_control::GazeboSimROS2ControlPlugin", params);
  }
  for (const PlanSensor& s : p.sensors) {
    if (s.addToUrdf) o << sensorBlock(s);
  }
  return o.str();
}

}  // namespace

std::string patchUrdf(const std::string& urdfText, const UrdfModel& model, const Plan& plan, std::string& additions, Issues& issues) {
  (void)model;
  std::string out = urdfText;

  // 1. Existing sensors that need a <topic>.
  for (const PlanSensor& s : plan.sensors) {
    if (!s.existing || !s.addTopicToUrdf) continue;
    const std::regex re("<sensor\\b[^>]*\\bname\\s*=\\s*([\"'])" + regexEscape(s.name) + "\\1[^>]*>");
    std::smatch m;
    if (!std::regex_search(out, m, re) || endsWith(m.str(), "/>")) {
      addIssue(issues, Severity::Error, "Patch", "Could not find the <sensor name=\"" + s.name + "\"> element to add its <topic>.");
      continue;
    }
    const std::string ins = "\n      <topic>" + s.gzTopic + "</topic>  <!-- added by " + kBanner + " -->";
    out.insert(static_cast<size_t>(m.position(0) + m.length(0)), ins);
  }

  // 2. The ros2_control hardware plugin.
  if (plan.patchHardware) {
    const std::regex re("(<hardware>\\s*<plugin>)\\s*[^<]*?\\s*(</plugin>)");
    const std::string replaced = std::regex_replace(out, re, "$1gz_ros2_control/GazeboSimSystem$2");
    if (replaced == out) addIssue(issues, Severity::Warning, "Patch", "The <hardware><plugin> of <ros2_control> could not be found to change it.");
    out = replaced;
  }

  // 3. New <gazebo> blocks go in front of </robot>.
  const std::string block = additionsBlock(plan);
  additions.clear();
  if (!block.empty()) {
    additions = "  <!-- ===== added by " + std::string(kBanner) + ": the original URDF file is not changed ===== -->\n" + block;
    const size_t end = out.rfind("</robot>");
    if (end == std::string::npos) {
      addIssue(issues, Severity::Error, "Patch", "There is no </robot> to add the Gazebo blocks in front of.");
    } else {
      out.insert(end, additions);
    }
  }
  return out;
}

std::string bridgeConfigFile(const Plan& p) {
  std::ostringstream h;
  h << "# ros_gz_bridge configuration for the model '" << p.modelName << "', written by " << kBanner << ".\n"
    << "# Each entry copies one topic between Gazebo (gz.msgs types) and ROS 2. It is read by\n"
    << "#   ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=<this file>\n"
    << "# direction: GZ_TO_ROS publishes in ROS what Gazebo publishes; ROS_TO_GZ sends ROS messages into Gazebo.\n";
  return bridgeYaml(p.bridge, h.str());
}

std::string worldFile(const Plan& p) {
  std::ostringstream o;
  o << "<?xml version=\"1.0\"?>\n"
    << "<!-- World for the model '" << p.modelName << "', written by " << kBanner << ". The robot itself is spawned by the launch file. -->\n"
    << "<sdf version=\"1.9\">\n"
    << "  <world name=\"" << p.worldName << "\">\n"
    << "    <physics name=\"step\" type=\"ignored\">\n"
    << "      <max_step_size>" << xmlText(p.stepSize) << "</max_step_size>\n"
    << "      <real_time_factor>1.0</real_time_factor>\n"
    << "    </physics>\n"
    << "    <gravity>" << xmlText(p.gravity.x) << " " << xmlText(p.gravity.y) << " " << xmlText(p.gravity.z) << "</gravity>\n";
  if (p.spherical) {
    o << "    <spherical_coordinates>\n"
      << "      <surface_model>EARTH_WGS84</surface_model>\n"
      << "      <world_frame_orientation>ENU</world_frame_orientation>\n"
      << "      <latitude_deg>" << xmlText(p.spherical->latitude) << "</latitude_deg>\n"
      << "      <longitude_deg>" << xmlText(p.spherical->longitude) << "</longitude_deg>\n"
      << "      <elevation>" << xmlText(p.spherical->elevation) << "</elevation>\n"
      << "      <heading_deg>" << xmlText(p.spherical->heading) << "</heading_deg>\n"
      << "    </spherical_coordinates>\n";
  }
  o << "\n    <!-- The systems every world needs: physics, the scene for the GUI, and commands from the GUI/ROS. -->\n"
    << plugin("gz-sim-physics-system", "gz::sim::systems::Physics", {})
    << plugin("gz-sim-user-commands-system", "gz::sim::systems::UserCommands", {})
    << plugin("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster", {});
  if (p.wantsSensorsSystem()) {
    o << "    <!-- Cameras and lidars are rendered, so they need the Sensors system with a render engine. -->\n"
      << plugin("gz-sim-sensors-system", "gz::sim::systems::Sensors", {{"render_engine", "ogre2"}});
  }
  if (p.hasKind(SensorKind::Imu)) o << plugin("gz-sim-imu-system", "gz::sim::systems::Imu", {});
  if (p.hasKind(SensorKind::Gps)) o << plugin("gz-sim-navsat-system", "gz::sim::systems::NavSat", {});
  if (p.hasKind(SensorKind::Magnetometer)) o << plugin("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer", {});
  if (p.hasKind(SensorKind::AirPressure)) o << plugin("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure", {});
  if (p.hasKind(SensorKind::Altimeter)) o << plugin("gz-sim-altimeter-system", "gz::sim::systems::Altimeter", {});
  o << R"(
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
)";
  return o.str();
}

std::string launchFile(const Plan& p, const UrdfModel& model) {
  (void)model;
  const bool rc = p.mode == ControlMode::Ros2Control;
  std::ostringstream o;
  o << "# Written by " << kBanner << ". This is ordinary ROS 2 launch code: read it, change it.\n"
    << "#\n"
    << "# It starts Gazebo with the world, publishes the robot description, puts the robot into the world\n"
    << "# and runs ros_gz_bridge with config/" << bridgeFileName(p) << ".\n"
    << "import os\n\n"
    << "from ament_index_python.packages import PackageNotFoundError, get_package_share_directory\n"
    << "from launch import LaunchDescription\n"
    << "from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription";
  if (rc) o << ", RegisterEventHandler";
  o << "\n"
    << "from launch.conditions import IfCondition, UnlessCondition\n";
  if (rc) o << "from launch.event_handlers import OnProcessExit\n";
  o << "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
    << "from launch.substitutions import LaunchConfiguration\n"
    << "from launch_ros.actions import Node\n\n"
    << "PACKAGE = " << pyString(p.packageName) << "\n"
    << "MODEL = " << pyString(p.modelName) << "\n";
  if (!p.meshPackages.empty()) {
    o << "# Packages the URDF loads meshes from (package://<name>/...). Gazebo looks for them in GZ_SIM_RESOURCE_PATH.\n"
      << "MESH_PACKAGES = [";
    for (size_t i = 0; i < p.meshPackages.size(); ++i) o << (i ? ", " : "") << pyString(p.meshPackages[i]);
    o << "]\n";
  }
  if (rc) {
    o << "# The controllers to start, in this order (broadcasters first), and the files that configure them.\n"
      << "CONTROLLERS = [";
    for (size_t i = 0; i < p.controllers.size(); ++i) o << (i ? ", " : "") << pyString(p.controllers[i].name);
    o << "]\nCONTROLLER_FILES = [";
    for (size_t i = 0; i < p.controllerFiles.size(); ++i) o << (i ? ", " : "") << pyString(p.controllerFiles[i].name);
    o << "]\n";
  }
  o << "\n\ndef generate_launch_description():\n"
    << "    share = get_package_share_directory(PACKAGE)\n"
    << "    gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')\n"
    << "    world = os.path.join(share, 'worlds', " << pyString(worldFileName(p)) << ")\n"
    << "    urdf = os.path.join(share, 'urdf', " << pyString(urdfFileName(p)) << ")\n"
    << "    bridge_config = os.path.join(share, 'config', " << pyString(bridgeFileName(p)) << ")\n\n"
    << "    with open(urdf, 'r') as f:\n"
    << "        robot_description = f.read()\n";
  if (rc) {
    o << "    # The URDF names its controller files as @CONFIG_DIR@/<file>; fill in where they are installed.\n"
      << "    robot_description = robot_description.replace('@CONFIG_DIR@', os.path.join(share, 'config'))\n";
  }
  o << "\n"
    << "    use_sim_time = LaunchConfiguration('use_sim_time')\n"
    << "    gui = LaunchConfiguration('gui')\n\n"
    << "    actions = [\n"
    << "        DeclareLaunchArgument('use_sim_time', default_value=" << pyString(p.useSimTime ? "true" : "false")
    << ", description='Use the clock that Gazebo publishes on /clock'),\n"
    << "        DeclareLaunchArgument('gui', default_value='true', description='false runs Gazebo without its window'),\n"
    << "    ]\n\n";
  if (!p.meshPackages.empty()) {
    o << "    # Let Gazebo find package://<name>/... meshes: it searches the folders listed in GZ_SIM_RESOURCE_PATH.\n"
      << "    for package in MESH_PACKAGES:\n"
      << "        try:\n"
      << "            actions.append(AppendEnvironmentVariable(\n"
      << "                'GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory(package))))\n"
      << "        except PackageNotFoundError:\n"
      << "            print(f'[sim.launch.py] package {package} is not installed; its meshes will be missing in Gazebo')\n\n";
  }
  o << "    # Gazebo itself: -r starts the simulation running, -s runs the server only (no window).\n"
    << "    actions.append(IncludeLaunchDescription(\n"
    << "        PythonLaunchDescriptionSource(gz_sim_launch),\n"
    << "        launch_arguments={'gz_args': '-r -v 2 ' + world}.items(),\n"
    << "        condition=IfCondition(gui)))\n"
    << "    actions.append(IncludeLaunchDescription(\n"
    << "        PythonLaunchDescriptionSource(gz_sim_launch),\n"
    << "        launch_arguments={'gz_args': '-r -s -v 2 ' + world}.items(),\n"
    << "        condition=UnlessCondition(gui)))\n\n"
    << "    # robot_state_publisher turns the URDF and /joint_states into the TF tree.\n"
    << "    actions.append(Node(\n"
    << "        package='robot_state_publisher', executable='robot_state_publisher', output='screen',\n"
    << "        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]))\n\n"
    << "    # Spawn the model from the robot_description topic into the running world.\n"
    << "    create = Node(\n"
    << "        package='ros_gz_sim', executable='create', output='screen',\n"
    << "        arguments=['-topic', 'robot_description', '-name', MODEL,\n"
    << "                   '-x', " << pyString(num(p.spawnXyz.x)) << ", '-y', " << pyString(num(p.spawnXyz.y)) << ", '-z', " << pyString(num(p.spawnXyz.z))
    << ",\n                   '-R', " << pyString(num(p.spawnRoll)) << ", '-P', " << pyString(num(p.spawnPitch)) << ", '-Y', " << pyString(num(p.spawnYaw)) << "])\n"
    << "    actions.append(create)\n\n"
    << "    # The bridge: every topic in the YAML file is copied between Gazebo and ROS 2.\n"
    << "    actions.append(Node(\n"
    << "        package='ros_gz_bridge', executable='parameter_bridge', output='screen',\n"
    << "        parameters=[{'config_file': bridge_config}]))\n";
  if (rc) {
    o << "\n    # Start the controllers one after the other once the robot exists in Gazebo.\n"
      << "    def spawner(name):\n"
      << "        arguments = [name, '--controller-manager', '/controller_manager']\n"
      << "        for file in CONTROLLER_FILES:\n"
      << "            arguments += ['--param-file', os.path.join(share, 'config', file)]\n"
      << "        return Node(package='controller_manager', executable='spawner', output='screen', arguments=arguments)\n\n"
      << "    previous = create\n"
      << "    for name in CONTROLLERS:\n"
      << "        current = spawner(name)\n"
      << "        actions.append(RegisterEventHandler(OnProcessExit(target_action=previous, on_exit=[current])))\n"
      << "        previous = current\n";
  }
  o << "\n    return LaunchDescription(actions)\n";
  return o.str();
}

std::string packageXml(const Plan& p) {
  std::set<std::string> deps = {"ros_gz_sim", "ros_gz_bridge", "robot_state_publisher", "launch", "launch_ros", "ament_index_python"};
  if (p.mode == ControlMode::Ros2Control) {
    deps.insert("gz_ros2_control");
    deps.insert("controller_manager");
    for (const ControllerSpec& c : p.controllers) {
      const size_t slash = c.type.find('/');
      if (slash != std::string::npos && slash > 0) deps.insert(c.type.substr(0, slash));
    }
  }
  for (const std::string& mp : p.meshPackages) deps.insert(mp);
  std::ostringstream o;
  o << "<?xml version=\"1.0\"?>\n"
    << "<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" schematypens=\"http://www.w3.org/2001/XMLSchema\"?>\n"
    << "<package format=\"3\">\n"
    << "  <name>" << p.packageName << "</name>\n"
    << "  <version>0.1.0</version>\n"
    << "  <description>Gazebo simulation of " << xmlEscape(p.modelName) << " with a ros_gz_bridge to ROS 2 (written by " << kBanner << ").</description>\n"
    << "  <maintainer email=\"you@example.com\">TODO your name</maintainer>\n"
    << "  <license>Apache-2.0</license>\n\n"
    << "  <buildtool_depend>ament_cmake</buildtool_depend>\n\n";
  for (const std::string& d : deps) o << "  <exec_depend>" << d << "</exec_depend>\n";
  o << "\n  <export>\n    <build_type>ament_cmake</build_type>\n  </export>\n</package>\n";
  return o.str();
}

std::string cmakeLists(const Plan& p) {
  std::ostringstream o;
  o << "cmake_minimum_required(VERSION 3.8)\n"
    << "project(" << p.packageName << ")\n\n"
    << "find_package(ament_cmake REQUIRED)\n\n"
    << "# Everything in this package is data: the launch file, the bridge and controller configuration,\n"
    << "# the world and the URDF. `ros2 launch " << p.packageName << " sim.launch.py` finds them in the install space.\n"
    << "install(DIRECTORY launch config worlds urdf DESTINATION share/${PROJECT_NAME})\n\n"
    << "ament_package()\n";
  return o.str();
}

std::string readmeFile(const Plan& p, const UrdfModel& model) {
  std::ostringstream o;
  o << "# " << p.packageName << "\n\n"
    << "A Gazebo (Harmonic) simulation of the robot `" << model.name << "` for ROS 2 Jazzy, written by " << kBanner << ".\n\n"
    << "## Run it\n\n```bash\ncolcon build --packages-select " << p.packageName << "\nsource install/setup.bash\nros2 launch " << p.packageName
    << " sim.launch.py            # gui:=false runs without the Gazebo window\n```\n\n"
    << "## What is in here\n\n"
    << "| file | what it is |\n|---|---|\n"
    << "| `launch/sim.launch.py` | starts Gazebo, robot_state_publisher, spawns the robot, runs the bridge |\n"
    << "| `config/" << bridgeFileName(p) << "` | the ros_gz_bridge configuration: which topics are copied, in which direction |\n"
    << "| `worlds/" << worldFileName(p) << "` | the world: physics step, ground, light, and the Gazebo systems the sensors need |\n"
    << "| `urdf/" << urdfFileName(p) << "` | your URDF plus the Gazebo plugins and sensors (everything added is marked) |\n";
  for (const Plan::ConfigFile& f : p.controllerFiles) o << "| `config/" << f.name << "` | your ros2_control controller configuration |\n";
  o << "\n## Topics\n\n| ROS 2 topic | direction | Gazebo topic | ROS type | gz type |\n|---|---|---|---|---|\n";
  for (const BridgeEntry& e : p.bridge) {
    if (!e.enabled || e.isService()) continue;
    o << "| `" << e.rosTopic << "` | " << (e.direction == Direction::GzToRos ? "Gazebo -> ROS" : e.direction == Direction::RosToGz ? "ROS -> Gazebo" : "both ways")
      << " | `" << e.gzTopic << "` | `" << e.rosType << "` | `" << e.gzType << "` |\n";
  }
  o << "\n## Try it\n\n```bash\nros2 topic list -t\n";
  for (const PlanDrive& d : p.drives) {
    o << "ros2 topic pub --rate 5 " << d.rosCmdVel << " geometry_msgs/msg/Twist \"{linear: {x: 0.2}, angular: {z: 0.3}}\"\nros2 topic echo " << d.rosOdom << "\n";
  }
  for (const PlanSensor& s : p.sensors) o << "ros2 topic hz " << s.rosTopic << "\n";
  o << "gz topic -l          # the same topics on the Gazebo side\n```\n\n"
    << "## Things to check yourself\n\n"
    << "* The maintainer in `package.xml` is a placeholder.\n"
    << "* Inertias and masses decide how the robot behaves; the physics is only as good as the URDF.\n";
  if (!p.jointControls.empty()) {
    o << "* The joint position controllers use a PID (p=" << num(p.pGain) << ", i=" << num(p.iGain) << ", d=" << num(p.dGain)
      << "). These numbers are a starting point: tune them for your joints.\n";
  }
  if (p.mode == ControlMode::Ros2Control) {
    o << "* The controllers come from ros2_control; check `ros2 control list_controllers` once the simulation runs.\n";
  }
  return o.str();
}

Generated generate(const UrdfModel& model, const std::string& urdfText, const std::string& urdfName, const Plan& plan, Issues& issues) {
  Generated g;
  (void)urdfName;
  auto add = [&g](const std::string& path, const std::string& content, const std::string& description) {
    g.files.push_back({path, content, description});
  };
  std::string additions;
  const std::string patched = patchUrdf(urdfText, model, plan, additions, issues);
  g.urdfAdditions = additions;

  // The patched URDF must still be a URDF with the same links and joints.
  Issues check;
  const UrdfModel again = parseUrdf(patched, check);
  if (!again.valid || again.links.size() != model.links.size() || again.joints.size() != model.joints.size()) {
    addIssue(issues, Severity::Error, "Patch", "The patched URDF does not read back as the same robot; it is not written. This is a bug in " + std::string(kBanner) + ".");
  }

  add("package.xml", packageXml(plan), "Package description and dependencies");
  add("CMakeLists.txt", cmakeLists(plan), "Installs launch, config, worlds and urdf");
  add("launch/sim.launch.py", launchFile(plan, model), "Starts Gazebo, spawns the robot, runs the bridge");
  add("config/" + bridgeFileName(plan), bridgeConfigFile(plan), "ros_gz_bridge configuration");
  add("worlds/" + worldFileName(plan), worldFile(plan), "Gazebo world");
  add("urdf/" + urdfFileName(plan), patched, "Your URDF with the Gazebo plugins and sensors added");
  for (const Plan::ConfigFile& f : plan.controllerFiles) add("config/" + f.name, f.text, "Your controllers (copied unchanged)");
  add("README.md", readmeFile(plan, model), "How to run it and what the topics are");
  return g;
}

bool writePackage(const std::string& directory, const Plan& plan, const Generated& g, std::string& error) {
  const std::string root = directory + "/" + plan.packageName;
  for (const GeneratedFile& f : g.files) {
    if (!writeTextFile(root + "/" + f.path, f.content, error)) return false;
  }
  return true;
}

}  // namespace gzb
