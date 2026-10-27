// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Second half of test_core.cpp: detection, the plan, the emitters, the project and the verify logic.
// (Included at the end of test_core.cpp; it uses the helpers defined there.)

namespace {

int countSub(const std::string& text, const std::string& needle) {
  int n = 0;
  for (size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + needle.size())) ++n;
  return n;
}

bool contains(const std::string& text, const std::string& needle) { return text.find(needle) != std::string::npos; }

// ----------------------------------------------------------------------- detection and plan

void testDetectDiffbot() {
  Project p = load("diffbot_plain.urdf");
  const Detection& d = p.detection();
  CHECK(p.model().valid);
  CHECK(d.mode == ControlMode::GzSystems);
  CHECK_EQ(d.drives.size(), static_cast<size_t>(1));
  if (d.drives.empty()) return;
  const DriveCandidate& dc = d.drives[0];
  CHECK(dc.kind == DriveKind::Diff);
  CHECK_EQ(dc.id, "drive:diff");
  CHECK_EQ(dc.left.size(), static_cast<size_t>(1));
  CHECK_EQ(dc.right.size(), static_cast<size_t>(1));
  CHECK_EQ(dc.left[0], "left_wheel_joint");
  CHECK_EQ(dc.right[0], "right_wheel_joint");
  CHECK_NEAR(dc.wheelSeparation, 0.24, 1e-9);
  CHECK_NEAR(dc.wheelRadius, 0.05, 1e-9);
  {
    // The wheel's cylinder is mounted 1 cm off the axle line: its surface reaches 0.05 + 0.01 m.
    std::string text = slurp(data("diffbot_plain.urdf"));
    const std::string from = "<origin rpy=\"1.5707963 0 0\"/>";
    const std::string to = "<origin xyz=\"0 0 0.01\" rpy=\"1.5707963 0 0\"/>";
    for (size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) text.replace(at, from.size(), to);
    CHECK(text.find("0 0 0.01") != std::string::npos);
    Project q;
    q.setUrdf("offset.urdf", text);
    CHECK(!q.detection().drives.empty());
    if (!q.detection().drives.empty()) CHECK_NEAR(q.detection().drives[0].wheelRadius, 0.06, 1e-9);
  }
  {
    // Wheels that are not mirror images: the separation is the distance between the two axles,
    // not twice the distance of one of them from the centre line.
    std::string text = slurp(data("diffbot_plain.urdf"));
    const std::string from = "<origin xyz=\"0 0.12 0\"/>";
    CHECK(text.find(from) != std::string::npos);
    text.replace(text.find(from), from.size(), "<origin xyz=\"0 0.20 0\"/>");
    Project q;
    q.setUrdf("asymmetric.urdf", text);
    CHECK(!q.detection().drives.empty());
    if (!q.detection().drives.empty()) CHECK_NEAR(q.detection().drives[0].wheelSeparation, 0.32, 1e-9);
  }
  {
    // Ackermann with the left rear wheel moved outwards: the rear track grows, the kingpin width does not.
    std::string text = slurp(data("corpus/ackermann_robot.urdf"));
    const std::string from = "<origin xyz=\"-0.15 0.13 -0.03\" rpy=\"0 0 0\"/>";
    CHECK(text.find(from) != std::string::npos);
    text.replace(text.find(from), from.size(), "<origin xyz=\"-0.15 0.20 -0.03\" rpy=\"0 0 0\"/>");
    Project q;
    q.setUrdf("ackermann_asymmetric.urdf", text);
    const DriveCandidate* dc = q.detection().drive("drive:ackermann");
    CHECK(dc != nullptr);
    if (dc) {
      CHECK_NEAR(dc->wheelSeparation, 0.33, 1e-9);
      CHECK_NEAR(dc->kingpinWidth, 0.26, 1e-9);
    }
  }
  {
    // A turret that turns about the vertical axis and carries a cylinder is not a wheel.
    std::string text = slurp(data("diffbot_plain.urdf"));
    const std::string turret =
        "<link name=\"turret_link\"><visual><geometry><cylinder radius=\"0.04\" length=\"0.02\"/></geometry></visual>"
        "<inertial><mass value=\"0.2\"/><inertia ixx=\"1e-4\" ixy=\"0\" ixz=\"0\" iyy=\"1e-4\" iyz=\"0\" izz=\"1e-4\"/></inertial></link>"
        "<joint name=\"turret_joint\" type=\"continuous\"><parent link=\"base_link\"/><child link=\"turret_link\"/>"
        "<origin xyz=\"0 0.1 0.08\"/><axis xyz=\"0 0 1\"/></joint></robot>";
    text.replace(text.rfind("</robot>"), 8, turret);
    Project q;
    q.setUrdf("turret.urdf", text);
    CHECK(q.model().valid);
    CHECK_EQ(q.detection().drives.size(), static_cast<size_t>(1));
    if (!q.detection().drives.empty()) {
      CHECK_EQ(q.detection().drives[0].left.size() + q.detection().drives[0].right.size(), static_cast<size_t>(2));
    }
  }
  CHECK(d.item("drive:diff") && d.item("drive:diff")->selected);
  CHECK(d.item("joint_states") && d.item("joint_states")->selected);
  CHECK(d.item("odometry") == nullptr);  // the drive publishes odometry itself
  CHECK(d.jointControls.empty());        // both movable joints are wheels
  CHECK(d.sensors.empty());

  const Plan& pl = p.plan();
  CHECK_EQ(pl.packageName, "diffbot_sim");
  CHECK_EQ(pl.modelName, "diffbot");
  CHECK_EQ(pl.worldName, "sim_world");
  CHECK_NEAR(pl.spawnXyz.z, 0.10, 1e-9);
  CHECK_EQ(pl.bridge.size(), static_cast<size_t>(5));
  const BridgeEntry* clock = findEntry(pl, "/clock");
  CHECK(clock && clock->direction == Direction::GzToRos && clock->qos == "CLOCK" && clock->gzType == "gz.msgs.Clock");
  const BridgeEntry* cmd = findEntry(pl, "/cmd_vel");
  CHECK(cmd && cmd->direction == Direction::RosToGz && cmd->gzTopic == "/model/diffbot/cmd_vel" && cmd->rosType == "geometry_msgs/msg/Twist" &&
        cmd->gzType == "gz.msgs.Twist");
  const BridgeEntry* odom = findEntry(pl, "/odom");
  CHECK(odom && odom->direction == Direction::GzToRos && odom->gzTopic == "/model/diffbot/odometry" && odom->rosType == "nav_msgs/msg/Odometry");
  const BridgeEntry* tf = findEntry(pl, "/tf");
  CHECK(tf && tf->gzTopic == "/model/diffbot/tf" && tf->gzType == "gz.msgs.Pose_V");
  const BridgeEntry* js = findEntry(pl, "/joint_states");
  CHECK(js && js->gzTopic == "/model/diffbot/joint_state" && js->gzType == "gz.msgs.Model");
  for (const BridgeEntry& e : pl.bridge) {
    CHECK(isKnownPair(e.rosType, e.gzType));
    CHECK(e.origin == "generated");
  }
  CHECK(!hasErrors(p.issues()));
  CHECK(p.ready());

  // Unticking the drive removes its three topics and the plugin, and offers odometry instead.
  p.setSelected("drive:diff", false);
  CHECK(findEntry(p.plan(), "/cmd_vel") == nullptr);
  CHECK(findEntry(p.plan(), "/odom") == nullptr);
  CHECK(findEntry(p.plan(), "/joint_states") != nullptr);
  CHECK(p.plan().drives.empty());
  CHECK(!contains(p.generated().file("urdf/diffbot.urdf")->content, "DiffDrive"));
  p.setSelected("drive:diff", true);
  CHECK(findEntry(p.plan(), "/cmd_vel") != nullptr);
}

void testDetectSensors() {
  Project p = load("diffbot_gz.urdf");
  const Detection& d = p.detection();
  CHECK_EQ(d.sensors.size(), static_cast<size_t>(3));  // depth_camera is not supported
  CHECK(hasIssue(p.issues(), Severity::Info, "depth_camera"));
  const SensorCandidate* cam = d.sensor("front_camera");
  CHECK(cam && cam->existing && !cam->needsTopic && cam->gzTopic == "/front_cam/image" && cam->kind == SensorKind::Camera);
  if (cam) CHECK_EQ(cam->frameLink, "camera_optical_frame");
  const SensorCandidate* imu = d.sensor("body_imu");
  CHECK(imu && imu->existing && imu->needsTopic && imu->kind == SensorKind::Imu);
  CHECK(d.item("sensortopic:body_imu") != nullptr);
  const SensorCandidate* laser = d.sensor("laser");
  CHECK(laser && !laser->existing && laser->kind == SensorKind::Lidar && laser->link == "laser_link");
  CHECK(d.item("sensor:laser") && d.item("sensor:laser")->selected);

  const Plan& pl = p.plan();
  const PlanSensor* fc = pl.sensor("front_camera");
  CHECK(fc && !fc->addToUrdf && fc->gzTopic == "/front_cam/image" && fc->gzSecondTopic == "/front_cam/camera_info" &&
        fc->rosTopic == "/front_camera/image_raw" && fc->rosSecondTopic == "/front_camera/camera_info");
  const PlanSensor* bi = pl.sensor("body_imu");
  CHECK(bi && bi->addTopicToUrdf && bi->gzTopic == "/model/diffbot/sensor/body_imu/imu" && bi->rosTopic == "/body_imu/data");
  const PlanSensor* la = pl.sensor("laser");
  CHECK(la && la->addToUrdf && la->gzTopic == "/model/diffbot/sensor/laser/scan" && la->gzSecondTopic == "/model/diffbot/sensor/laser/scan/points" &&
        la->rosTopic == "/laser/scan" && la->rosSecondTopic == "/laser/points");
  const BridgeEntry* img = findEntry(pl, "/front_camera/image_raw");
  CHECK(img && img->frameId == "camera_optical_frame" && img->gzType == "gz.msgs.Image" && img->direction == Direction::GzToRos);
  const BridgeEntry* scan = findEntry(pl, "/laser/scan");
  CHECK(scan && scan->frameId == "laser_link" && scan->rosType == "sensor_msgs/msg/LaserScan");
  CHECK(findEntry(pl, "/laser/points") != nullptr);
  CHECK(findEntry(pl, "/body_imu/data") != nullptr);

  // Without the <topic> patch the existing IMU cannot be bridged.
  p.setSelected("sensortopic:body_imu", false);
  CHECK(p.plan().sensor("body_imu") == nullptr);
  CHECK(hasIssue(p.issues(), Severity::Warning, "has no <topic>"));
  p.setSelected("sensortopic:body_imu", true);
  CHECK(p.plan().sensor("body_imu") != nullptr);

  // Turning a sensor off removes its topics.
  p.setSelected("sensor:laser", false);
  CHECK(findEntry(p.plan(), "/laser/scan") == nullptr);
  CHECK(contains(p.generated().file("worlds/sim_world.sdf")->content, "gz::sim::systems::Sensors"));  // the camera still needs it
  p.setSelected("sensor:front_camera", false);
  CHECK(!contains(p.generated().file("worlds/sim_world.sdf")->content, "gz::sim::systems::Sensors"));
  CHECK(contains(p.generated().file("worlds/sim_world.sdf")->content, "gz::sim::systems::Imu"));
}

void testDetectCorpus() {
  {
    Project p = load("corpus/diff_drive_robot.urdf");
    const DriveCandidate* d = p.detection().drive("drive:diff");
    CHECK(d != nullptr);
    if (d) {
      CHECK_NEAR(d->wheelSeparation, 0.24, 1e-9);
      CHECK_NEAR(d->wheelRadius, 0.05, 1e-9);
    }
  }
  {
    Project p = load("corpus/four_wheel_robot.urdf");
    const DriveCandidate* d = p.detection().drive("drive:diff");
    CHECK(d != nullptr);
    if (d) {
      CHECK_EQ(d->left.size(), static_cast<size_t>(2));
      CHECK_EQ(d->right.size(), static_cast<size_t>(2));
      CHECK_NEAR(d->wheelSeparation, 0.28, 1e-9);
      CHECK_NEAR(d->wheelRadius, 0.06, 1e-9);
    }
    CHECK_EQ(countSub(p.generated().file("urdf/four_wheel_robot.urdf")->content, "<left_joint>"), 2);
    CHECK_EQ(countSub(p.generated().file("urdf/four_wheel_robot.urdf")->content, "<right_joint>"), 2);
  }
  {
    Project p = load("corpus/variant_four_wheel_mecanum.urdf");
    const DriveCandidate* d = p.detection().drive("drive:mecanum");
    CHECK(d != nullptr);
    if (d) {
      CHECK_EQ(d->frontLeft, "front_left_wheel_joint");
      CHECK_EQ(d->frontRight, "front_right_wheel_joint");
      CHECK_EQ(d->backLeft, "rear_left_wheel_joint");
      CHECK_EQ(d->backRight, "rear_right_wheel_joint");
      CHECK_NEAR(d->wheelSeparation, 0.28, 1e-9);
      CHECK_NEAR(d->wheelBase, 0.20, 1e-9);
      CHECK_NEAR(d->wheelRadius, 0.06, 1e-3);  // the ring of rollers, not the smaller hub
    }
    const std::string& urdf = p.generated().file("urdf/variant_four_wheel_mecanum.urdf")->content;
    CHECK(contains(urdf, "<wheelbase>0.2</wheelbase>"));
    CHECK(contains(urdf, "gz::sim::systems::MecanumDrive"));
  }
  {
    Project p = load("corpus/ackermann_robot.urdf");
    const DriveCandidate* d = p.detection().drive("drive:ackermann");
    CHECK(d != nullptr);
    if (d) {
      CHECK_EQ(d->leftSteering, "front_left_steer_joint");
      CHECK_EQ(d->rightSteering, "front_right_steer_joint");
      CHECK_EQ(d->left.size(), static_cast<size_t>(1));
      CHECK_EQ(d->left[0], "rear_left_wheel_joint");
      CHECK_EQ(d->right[0], "rear_right_wheel_joint");
      CHECK_NEAR(d->wheelSeparation, 0.26, 1e-9);
      CHECK_NEAR(d->kingpinWidth, 0.26, 1e-9);
      CHECK_NEAR(d->wheelBase, 0.30, 1e-9);
      CHECK_NEAR(d->steeringLimit, 0.52359878, 1e-9);
      CHECK_NEAR(d->wheelRadius, 0.06, 1e-9);
    }
    const std::string& urdf = p.generated().file("urdf/ackermann_robot.urdf")->content;
    CHECK(contains(urdf, "gz::sim::systems::AckermannSteering"));
    CHECK(contains(urdf, "<kingpin_width>0.26</kingpin_width>"));
    CHECK(contains(urdf, "<steering_limit>0.52359878</steering_limit>"));
  }
  {
    Project p = load("corpus/variant_ackermann_mecanum_all.urdf");
    const DriveCandidate* d = p.detection().drive("drive:ackermann");
    CHECK(d != nullptr);
    if (d) CHECK_NEAR(d->wheelRadius, 0.06, 1e-3);
  }
  {
    Project p = load("corpus/hexapod.urdf");
    CHECK(p.detection().drives.empty());
    CHECK_EQ(p.detection().jointControls.size(), static_cast<size_t>(18));
    CHECK(p.detection().item("joint_states")->selected);
    CHECK(p.detection().item("odometry") != nullptr && !p.detection().item("odometry")->selected);
    CHECK_EQ(p.plan().bridge.size(), static_cast<size_t>(20));  // /clock, /joint_states, 18 joint commands
    const BridgeEntry* e = findEntry(p.plan(), "/joint/leg1_coxa_joint/cmd_pos");
    CHECK(e && e->direction == Direction::RosToGz && e->gzTopic == "/model/hexapod/joint/leg1_coxa_joint/cmd_pos" && e->gzType == "gz.msgs.Double");
    CHECK_EQ(countSub(p.generated().file("urdf/hexapod.urdf")->content, "systems::JointPositionController\""), 18);
    // The odometry source for a robot without wheels.
    p.setSelected("odometry", true);
    CHECK(findEntry(p.plan(), "/odom") != nullptr);
    CHECK(contains(p.generated().file("urdf/hexapod.urdf")->content, "OdometryPublisher"));
  }
  {
    Project p = load("corpus/arm.urdf");
    CHECK_EQ(p.detection().jointControls.size(), static_cast<size_t>(3));
    CHECK_EQ(p.plan().jointControls.size(), static_cast<size_t>(3));
    p.setSelected("jointpos:joint_2", false);
    CHECK_EQ(p.plan().jointControls.size(), static_cast<size_t>(2));
  }
  {
    Project p = load("corpus/variant_sensors_all_types.urdf");
    std::vector<std::string> names;
    for (const SensorCandidate& s : p.detection().sensors) names.push_back(s.name);
    CHECK_EQ(names.size(), static_cast<size_t>(5));
    CHECK((names == std::vector<std::string>{"camera", "imu", "lidar", "gps", "camera_2"}));
    CHECK_EQ(p.detection().sensor("camera")->frameLink, "camera_optical_frame");
    CHECK_EQ(p.detection().sensor("camera_2")->frameLink, "camera_2_optical_frame");
    CHECK(p.plan().spherical.has_value());
    // A camera box bolted to the left wheel is not part of the wheel's radius.
    if (p.detection().drive("drive:diff")) CHECK_NEAR(p.detection().drive("drive:diff")->wheelRadius, 0.05, 1e-9);
    CHECK(hasIssue(p.issues(), Severity::Info, "latitude 0, longitude 0"));
    const std::string& world = p.generated().file("worlds/sim_world.sdf")->content;
    CHECK(contains(world, "gz::sim::systems::NavSat"));
    CHECK(contains(world, "gz::sim::systems::Imu"));
    CHECK(contains(world, "<render_engine>ogre2</render_engine>"));
    CHECK(contains(world, "<spherical_coordinates>"));
  }
  {
    // Mixed wheels: no mecanum drive, and the user is told why.
    Project p = load("corpus/variant_four_wheel_mecanum_mixed.urdf");
    CHECK(p.detection().drive("drive:mecanum") == nullptr);
    CHECK(p.detection().drive("drive:diff") != nullptr);
    CHECK(hasIssue(p.issues(), Severity::Warning, "mixes mecanum and ordinary wheels"));
  }
}

void testOverrides() {
  Project p = load("diffbot_gz.urdf", {"overrides.yaml"});
  const Plan& pl = p.plan();
  CHECK_EQ(pl.packageName, "tiny_sim");
  CHECK_EQ(pl.modelName, "tiny");
  CHECK_EQ(pl.worldName, "lab");
  CHECK_NEAR(pl.stepSize, 0.002, 1e-12);
  CHECK(pl.spherical && std::fabs(pl.spherical->latitude - 48.8566) < 1e-12);
  CHECK_NEAR(pl.spawnXyz.x, 1.0, 1e-12);
  CHECK_NEAR(pl.spawnXyz.y, -2.0, 1e-12);
  CHECK_NEAR(pl.spawnXyz.z, 0.3, 1e-12);
  CHECK_NEAR(pl.spawnYaw, 1.57, 1e-12);
  CHECK(!hasIssue(p.issues(), Severity::Info, "Spawn height"));
  const PlanSensor* fc = pl.sensor("front_camera");
  CHECK(fc && fc->rosTopic == "/cam/rgb" && fc->rosSecondTopic == "/cam/camera_info");
  const PlanSensor* la = pl.sensor("laser");
  CHECK(la && la->samples == 720 && std::fabs(la->rangeMin - 0.2) < 1e-12 && std::fabs(la->rangeMax - 20) < 1e-12 && std::fabs(la->updateRate - 20) < 1e-12);
  CHECK(la && la->rosTopic == "/scan" && la->rosSecondTopic == "/points");
  CHECK(pl.sensor("body_imu") == nullptr);  // enabled: false
  CHECK(p.detection().item("sensor:body_imu") && !p.detection().item("sensor:body_imu")->selected);
  CHECK(pl.drives.size() == 1 && pl.drives[0].rosCmdVel == "/robot/cmd_vel" && pl.drives[0].rosOdom == "/robot/odom");
  CHECK(pl.drives.size() == 1 && pl.drives[0].gzCmdVel == "/model/tiny/cmd_vel");
  CHECK(findEntry(pl, "/robot/cmd_vel") != nullptr);
  CHECK(findEntry(pl, "/tf") == nullptr);  // excluded
  bool tfDisabled = false;
  for (const BridgeEntry& e : pl.bridge) {
    if (e.rosTopic == "/tf" && !e.enabled) tfDisabled = true;
  }
  CHECK(tfDisabled);
  const std::string& urdf = p.generated().file("urdf/tiny.urdf")->content;
  CHECK(contains(urdf, "<max_linear_velocity>1.5</max_linear_velocity>"));
  CHECK(contains(urdf, "<max_angular_velocity>2.5</max_angular_velocity>"));
  CHECK(contains(urdf, "<samples>720</samples>"));
  CHECK(contains(urdf, "<update_rate>20</update_rate>"));
  CHECK(!hasErrors(p.issues()));

  // A wrong sensor name is reported with a suggestion; settings of a sensor that is already in
  // the URDF are not silently dropped.
  Project q = load("diffbot_gz.urdf");
  q.addYamlText("typo.yaml", "sensors:\n  lazer: {samples: 10}\n  front_camera: {width: 100}\n");
  CHECK(hasIssue(q.issues(), Severity::Warning, "sensors.lazer does not match a sensor found in the URDF; did you mean 'laser'?"));
  CHECK(hasIssue(q.issues(), Severity::Warning, "already written in the URDF"));

  // Invalid names fall back and are reported.
  Project r = load("diffbot_plain.urdf");
  r.addYamlText("names.yaml", "package_name: Bad-Name\nmodel_name: has space\nworld: {name: 'a b'}\n");
  CHECK_EQ(r.plan().packageName, "diffbot_sim");
  CHECK_EQ(r.plan().modelName, "diffbot");
  CHECK_EQ(r.plan().worldName, "sim_world");
  CHECK(hasIssue(r.issues(), Severity::Error, "not a valid ROS 2 package name"));
  CHECK(!r.ready());

  // Later files win key by key.
  Project s = load("diffbot_plain.urdf");
  s.addYamlText("a.yaml", "model_name: first\nspawn: {z: 1.0}\n");
  s.addYamlText("b.yaml", "model_name: second\n");
  CHECK_EQ(s.plan().modelName, "second");
  CHECK_NEAR(s.plan().spawnXyz.z, 1.0, 1e-12);

  // Setting only x keeps the computed height instead of dropping the robot into the ground.
  Project sp = load("diffbot_plain.urdf");
  sp.addYamlText("x.yaml", "spawn: {x: 2.0}\n");
  CHECK_NEAR(sp.plan().spawnXyz.x, 2.0, 1e-12);
  CHECK_NEAR(sp.plan().spawnXyz.z, 0.10, 1e-9);
}

void testBridgeMerge() {
  Project p = load("diffbot_plain.urdf", {"bridge_existing.yaml"});
  const Plan& pl = p.plan();
  // The user's /cmd_vel replaces the generated one; /battery is added.
  const BridgeEntry* cmd = findEntry(pl, "/cmd_vel");
  CHECK(cmd && cmd->gzTopic == "/my_own/cmd_vel" && cmd->origin == "file bridge_existing.yaml");
  int generatedCmd = 0, userCmd = 0;
  for (const BridgeEntry& e : pl.bridge) {
    if (e.rosTopic == "/cmd_vel" && e.origin == "generated") {
      ++generatedCmd;
      CHECK(!e.enabled);
    }
    if (e.rosTopic == "/cmd_vel" && e.origin != "generated") ++userCmd;
  }
  CHECK_EQ(generatedCmd, 1);
  CHECK_EQ(userCmd, 1);
  CHECK(findEntry(pl, "/battery") != nullptr);
  CHECK(hasIssue(p.issues(), Severity::Info, "replaces the generated one"));
  CHECK(hasIssue(p.issues(), Severity::Warning, "uses the Gazebo topic /my_own/cmd_vel, but the plugin or sensor that this helper adds"));
  // The file has broken entries, so the package is not ready until they are fixed.
  CHECK(hasErrors(p.issues()));
  CHECK(!p.ready());  // the broken entries of the file have to be fixed (or the file removed) first

  // Two bridges that would publish the same ROS topic from different sources.
  Project q = load("diffbot_gz.urdf");
  q.addYamlText("clash.yaml", "sensors:\n  front_camera: {ros_topic: /scan}\n  laser: {ros_topic: /scan}\n");
  CHECK(hasIssue(q.issues(), Severity::Error, "Two bridges publish /scan"));
  CHECK(!q.ready());

  // An identical entry twice is only a warning.
  Project r = load("diffbot_plain.urdf");
  r.addYamlText("one.yaml", "- topic_name: /battery\n  ros_type_name: sensor_msgs/msg/BatteryState\n  gz_type_name: gz.msgs.BatteryState\n  direction: GZ_TO_ROS\n");
  r.addYamlText("two.yaml", "- topic_name: /battery\n  ros_type_name: sensor_msgs/msg/BatteryState\n  gz_type_name: gz.msgs.BatteryState\n  direction: GZ_TO_ROS\n");
  CHECK(hasIssue(r.issues(), Severity::Warning, "appears twice"));

  // exclude of something that is not bridged is reported.
  Project s = load("diffbot_plain.urdf");
  s.addYamlText("ex.yaml", "exclude: [/nothing_here, /odom]\n");
  CHECK(hasIssue(s.issues(), Severity::Warning, "'/nothing_here'"));
  CHECK(findEntry(s.plan(), "/odom") == nullptr);
}

void testRos2Control() {
  Project p = load("diffbot_ros2control.urdf", {"controllers.yaml"});
  const Detection& d = p.detection();
  CHECK(d.mode == ControlMode::Ros2Control);
  CHECK(d.ros2.urdfHasTag);
  CHECK(!d.ros2.pluginPresent);
  CHECK(d.ros2.hardwareNeedsPatch);
  CHECK_EQ(d.ros2.controllers.size(), static_cast<size_t>(2));
  CHECK(d.drives.empty());
  CHECK(d.item("ros2control:plugin") && d.item("ros2control:plugin")->selected);
  CHECK(d.item("ros2control:hardware") && d.item("ros2control:hardware")->selected);
  CHECK(d.item("joint_states") == nullptr);  // joint_state_broadcaster does that

  const Plan& pl = p.plan();
  CHECK(pl.mode == ControlMode::Ros2Control);
  CHECK_EQ(pl.bridge.size(), static_cast<size_t>(1));  // only /clock goes through the bridge
  CHECK(pl.addControlPlugin && pl.patchHardware);
  CHECK_EQ(pl.controllers.size(), static_cast<size_t>(2));
  CHECK_EQ(pl.controllers[0].name, "joint_state_broadcaster");
  CHECK_EQ(pl.controllers[1].name, "diff_drive_controller");
  CHECK_EQ(pl.controllerFiles.size(), static_cast<size_t>(1));
  CHECK_EQ(pl.controllerFiles[0].name, "controllers.yaml");
  CHECK_EQ(pl.controllerFiles[0].text, slurp(data("controllers.yaml")));
  CHECK(!hasErrors(p.issues()));
  CHECK(p.ready());

  const std::string& urdf = p.generated().file("urdf/diffbot.urdf")->content;
  CHECK(contains(urdf, "<plugin>gz_ros2_control/GazeboSimSystem</plugin>"));
  CHECK(!contains(urdf, "mock_components"));
  CHECK(contains(urdf, "filename=\"gz_ros2_control-system\" name=\"gz_ros2_control::GazeboSimROS2ControlPlugin\""));
  CHECK(contains(urdf, "<parameters>@CONFIG_DIR@/controllers.yaml</parameters>"));
  CHECK(!contains(urdf, "DiffDrive"));
  const std::string& launch = p.generated().file("launch/sim.launch.py")->content;
  CHECK(contains(launch, "CONTROLLERS = ['joint_state_broadcaster', 'diff_drive_controller']"));
  CHECK(contains(launch, "CONTROLLER_FILES = ['controllers.yaml']"));
  CHECK(contains(launch, "replace('@CONFIG_DIR@'"));
  CHECK(contains(launch, "OnProcessExit"));
  const std::string& pkg = p.generated().file("package.xml")->content;
  for (const char* dep : {"gz_ros2_control", "controller_manager", "diff_drive_controller", "joint_state_broadcaster", "ros_gz_sim", "ros_gz_bridge"}) {
    CHECK(contains(pkg, std::string("<exec_depend>") + dep + "</exec_depend>"));
  }
  CHECK(p.generated().file("config/controllers.yaml") != nullptr);
  CHECK_EQ(p.generated().file("config/controllers.yaml")->content, slurp(data("controllers.yaml")));

  // Unticking the plugin leaves the URDF without it.
  p.setSelected("ros2control:plugin", false);
  CHECK(!contains(p.generated().file("urdf/diffbot.urdf")->content, "gz_ros2_control-system"));
  p.setSelected("ros2control:plugin", true);

  // A controller that names a joint the URDF does not have.
  Project q = load("diffbot_ros2control.urdf");
  q.addYamlText("bad.yaml",
                "controller_manager:\n  ros__parameters:\n    update_rate: 50\n    dd:\n      type: diff_drive_controller/DiffDriveController\n"
                "dd:\n  ros__parameters:\n    left_wheel_names: [left_wheel_joint2]\n    right_wheel_names: [right_wheel_joint]\n    command_interfaces: [position]\n");
  CHECK(hasIssue(q.issues(), Severity::Error, "'left_wheel_joint2', which is not in the URDF; did you mean 'left_wheel_joint'?"));
  CHECK(hasIssue(q.issues(), Severity::Error, "wants the command interface 'position' on 'right_wheel_joint', but <ros2_control> declares only: velocity"));
  CHECK(!q.ready());

  // Controllers faster than the physics step.
  Project r = load("diffbot_ros2control.urdf", {"controllers.yaml"});
  r.addYamlText("slow.yaml", "world: {step_size: 0.02}\n");
  CHECK(hasIssue(r.issues(), Severity::Warning, "faster than the physics step"));

  // A file that is both controllers and overrides is warned about.
  Project s = load("diffbot_ros2control.urdf");
  s.addYamlText("mixed.yaml", slurp(data("controllers.yaml")) + "package_name: mixed_sim\n");
  CHECK(hasIssue(s.issues(), Severity::Warning, "keep overrides in a separate file"));

  // Controllers without <ros2_control> in the URDF.
  Project t = load("diffbot_plain.urdf", {"controllers.yaml"});
  CHECK(t.detection().mode == ControlMode::Ros2Control);
  CHECK(hasIssue(t.issues(), Severity::Warning, "has no <ros2_control> element"));
}

}  // namespace

void testDetectAndPlan() {
  testDetectDiffbot();
  testDetectSensors();
  testDetectCorpus();
  testOverrides();
  testBridgeMerge();
  testRos2Control();
}

// ------------------------------------------------------------------------------- emitters

namespace {

std::string withoutAdditions(const std::string& patched, const std::string& additions) {
  const size_t at = patched.find(additions);
  return at == std::string::npos ? patched : patched.substr(0, at) + patched.substr(at + additions.size());
}

void testPatch() {
  {
    Project p = load("diffbot_plain.urdf");
    const std::string original = slurp(data("diffbot_plain.urdf"));
    const std::string& patched = p.generated().file("urdf/diffbot.urdf")->content;
    CHECK(!p.generated().urdfAdditions.empty());
    // Taking the additions out gives the original back, byte for byte.
    CHECK_EQ(withoutAdditions(patched, p.generated().urdfAdditions), original);
    CHECK(startsWith(p.generated().urdfAdditions, "  <!-- ===== added by gz_bridge_helper"));
    CHECK(hasErrors(p.issues()) == false);
    Issues is;
    const UrdfModel again = parseUrdf(patched, is);
    CHECK(again.valid);
    CHECK_EQ(again.links.size(), p.model().links.size());
    CHECK_EQ(again.joints.size(), p.model().joints.size());
    CHECK(again.hasGazeboPlugin("gz-sim-diff-drive-system"));
    CHECK(again.hasGazeboPlugin("gz-sim-joint-state-publisher-system"));
    CHECK(contains(patched, "<left_joint>left_wheel_joint</left_joint>"));
    CHECK(contains(patched, "<right_joint>right_wheel_joint</right_joint>"));
    CHECK(contains(patched, "<wheel_separation>0.24</wheel_separation>"));
    CHECK(contains(patched, "<wheel_radius>0.05</wheel_radius>"));
    CHECK(contains(patched, "<frame_id>odom</frame_id>"));
    CHECK(contains(patched, "<child_frame_id>base_link</child_frame_id>"));
    CHECK(contains(patched, "<topic>/model/diffbot/cmd_vel</topic>"));
  }
  {
    Project p = load("diffbot_gz.urdf");
    const std::string& patched = p.generated().file("urdf/diffbot.urdf")->content;
    Issues is;
    const UrdfModel again = parseUrdf(patched, is);
    CHECK_EQ(again.sensors.size(), static_cast<size_t>(4));
    for (const GazeboSensor& s : again.sensors) {
      if (s.name == "body_imu") CHECK_EQ(s.topic, "/model/diffbot/sensor/body_imu/imu");
      if (s.name == "front_camera") CHECK_EQ(s.topic, "/front_cam/image");
      if (s.name == "laser") {
        CHECK_EQ(s.type, "gpu_lidar");
        CHECK_EQ(s.reference, "laser_link");
        CHECK_EQ(s.gzFrameId, "laser_link");
        CHECK_EQ(s.topic, "/model/diffbot/sensor/laser/scan");
      }
    }
    CHECK_EQ(countSub(patched, "added by gz_bridge_helper"), 2);  // the banner and the inserted <topic>
    // Only the topic insertion and the additions differ from the original.
    const std::string original = slurp(data("diffbot_gz.urdf"));
    std::string stripped = withoutAdditions(patched, p.generated().urdfAdditions);
    const std::string ins = "\n      <topic>/model/diffbot/sensor/body_imu/imu</topic>  <!-- added by gz_bridge_helper -->";
    const size_t at = stripped.find(ins);
    CHECK(at != std::string::npos);
    if (at != std::string::npos) stripped.erase(at, ins.size());
    CHECK_EQ(stripped, original);
  }
  {
    // The patch must not break on self-closing or odd sensor elements.
    const std::string urdf =
        "<robot name='x'><link name='a'><inertial><mass value='1'/></inertial></link>"
        "<gazebo reference='a'><sensor type='imu' name='s1'/></gazebo></robot>";
    Project p;
    p.setUrdf("x.urdf", urdf);
    CHECK(p.model().valid);
    CHECK(hasIssue(p.issues(), Severity::Error, "Could not find the <sensor name=\"s1\"> element"));
    CHECK(!p.ready());
  }
}

void testWorldAndLaunch() {
  {
    Project p = load("diffbot_plain.urdf");
    const std::string& world = p.generated().file("worlds/sim_world.sdf")->content;
    CHECK(contains(world, "<world name=\"sim_world\">"));
    CHECK(contains(world, "<max_step_size>0.001</max_step_size>"));
    CHECK(contains(world, "gz::sim::systems::Physics"));
    CHECK(contains(world, "gz::sim::systems::UserCommands"));
    CHECK(contains(world, "gz::sim::systems::SceneBroadcaster"));
    CHECK(!contains(world, "gz::sim::systems::Sensors"));  // nothing needs rendering
    CHECK(!contains(world, "spherical_coordinates"));
    const std::string& launch = p.generated().file("launch/sim.launch.py")->content;
    CHECK(contains(launch, "PACKAGE = 'diffbot_sim'"));
    CHECK(contains(launch, "MODEL = 'diffbot'"));
    CHECK(contains(launch, "'-z', '0.1'"));
    CHECK(contains(launch, "'config_file': bridge_config"));
    CHECK(!contains(launch, "CONTROLLERS"));
    CHECK(!contains(launch, "MESH_PACKAGES"));
    CHECK(contains(launch, "executable='create'"));
    CHECK(contains(launch, "executable='parameter_bridge'"));
    CHECK(contains(launch, "'use_sim_time': use_sim_time"));
    const std::string& cmake = p.generated().file("CMakeLists.txt")->content;
    CHECK(contains(cmake, "project(diffbot_sim)"));
    CHECK(contains(cmake, "install(DIRECTORY launch config worlds urdf DESTINATION share/${PROJECT_NAME})"));
    const std::string& readme = p.generated().file("README.md")->content;
    CHECK(contains(readme, "ros2 launch diffbot_sim sim.launch.py"));
    CHECK(contains(readme, "| `/cmd_vel` | ROS -> Gazebo | `/model/diffbot/cmd_vel`"));
  }
  {
    // A model with meshes from another package: Gazebo is told where to look.
    const std::string urdf =
        "<robot name='m'><link name='a'><visual><geometry><mesh filename='package://my_meshes/a.dae'/></geometry></visual>"
        "<inertial><mass value='1'/></inertial></link></robot>";
    Project p;
    p.setUrdf("m.urdf", urdf);
    const std::string& launch = p.generated().file("launch/sim.launch.py")->content;
    CHECK(contains(launch, "MESH_PACKAGES = ['my_meshes']"));
    CHECK(contains(launch, "AppendEnvironmentVariable"));
    CHECK(contains(launch, "GZ_SIM_RESOURCE_PATH"));
    CHECK(contains(p.generated().file("package.xml")->content, "<exec_depend>my_meshes</exec_depend>"));
  }
}

void testRoundTrip() {
  // The bridge YAML that is written reads back as the entries of the plan.
  for (const char* f : {"diffbot_plain.urdf", "diffbot_gz.urdf", "corpus/hexapod.urdf", "corpus/variant_sensors_all_types.urdf"}) {
    Project p = load(f);
    const Plan& pl = p.plan();
    const GeneratedFile* file = nullptr;
    for (const GeneratedFile& g : p.generated().files) {
      if (startsWith(g.path, "config/") && endsWith(g.path, "_bridge.yaml")) file = &g;
    }
    CHECK(file != nullptr);
    if (!file) continue;
    Issues is;
    const ParsedYaml y = parseYaml("bridge.yaml", file->content, is);
    CHECK(y.is(YamlKind::BridgeConfig));
    CHECK(!hasErrors(is));
    CHECK(is.empty());
    std::vector<BridgeEntry> enabled;
    for (const BridgeEntry& e : pl.bridge) {
      if (e.enabled) enabled.push_back(e);
    }
    CHECK_EQ(y.bridge.size(), enabled.size());
    for (size_t i = 0; i < std::min(y.bridge.size(), enabled.size()); ++i) {
      CHECK_EQ(y.bridge[i].rosTopic, enabled[i].rosTopic);
      CHECK_EQ(y.bridge[i].gzTopic, enabled[i].gzTopic);
      CHECK_EQ(y.bridge[i].rosType, enabled[i].rosType);
      CHECK_EQ(y.bridge[i].gzType, enabled[i].gzType);
      CHECK(y.bridge[i].direction == enabled[i].direction);
      CHECK_EQ(y.bridge[i].qos, enabled[i].qos);
      CHECK_EQ(y.bridge[i].frameId, enabled[i].frameId);
    }
    // And yaml-cpp, which ros_gz_bridge itself uses, sees a sequence of maps with the same count.
    const YAML::Node node = YAML::Load(file->content);
    CHECK(node.IsSequence());
    CHECK_EQ(node.size(), enabled.size());
  }
  // The generated bridge file is read again when it is given as the "existing" config.
  Project q = load("diffbot_plain.urdf");
  std::string bridgeText;
  for (const GeneratedFile& g : q.generated().files) {
    if (endsWith(g.path, "_bridge.yaml")) bridgeText = g.content;
  }
  Project r = load("diffbot_plain.urdf");
  r.addYamlText("again.yaml", bridgeText);
  CHECK(!hasErrors(r.issues()));
  int enabledPerTopic = 0;
  for (const BridgeEntry& e : r.plan().bridge) {
    if (e.rosTopic == "/cmd_vel" && e.enabled) ++enabledPerTopic;
  }
  CHECK_EQ(enabledPerTopic, 1);
}

void testWholeCorpus() {
  namespace fs = std::filesystem;
  int models = 0;
  for (const auto& de : fs::directory_iterator(data("corpus"))) {
    if (de.path().extension() != ".urdf") continue;
    ++models;
    const std::string name = de.path().filename().string();
    Project p;
    p.loadUrdfFile(de.path().string());
    CHECK(p.model().valid);
    const Issues all = p.issues();
    if (hasErrors(all)) std::cerr << name << ":\n" << issuesText(all);
    CHECK(!hasErrors(all));
    CHECK(p.ready());
    std::set<std::string> published;
    for (const BridgeEntry& e : p.plan().bridge) {
      Issues is;
      validateEntry(e, "test", is);
      CHECK(!hasErrors(is));
      if (e.direction == Direction::GzToRos && e.rosTopic != "/tf") CHECK(published.insert(e.rosTopic).second);
    }
    const GeneratedFile* urdf = nullptr;
    for (const GeneratedFile& g : p.generated().files) {
      if (startsWith(g.path, "urdf/")) urdf = &g;
    }
    CHECK(urdf != nullptr);
    if (urdf) {
      Issues is;
      const UrdfModel again = parseUrdf(urdf->content, is);
      CHECK(again.valid);
      CHECK_EQ(again.links.size(), p.model().links.size());
      CHECK_EQ(again.joints.size(), p.model().joints.size());
      CHECK_EQ(withoutAdditions(urdf->content, p.generated().urdfAdditions), slurp(de.path().string()));
    }
  }
  CHECK(models >= 12);
}

}  // namespace

void testEmit() {
  testPatch();
  testWorldAndLaunch();
  testRoundTrip();
  testWholeCorpus();
}

// --------------------------------------------------------------------------------- project

void testProject() {
  const std::string out = tempDir("project");
  Project p = load("diffbot_gz.urdf", {"overrides.yaml"});
  CHECK(p.ready());
  std::string err;
  CHECK(p.write(out, err));
  const std::string root = out + "/tiny_sim";
  for (const GeneratedFile& g : p.generated().files) {
    std::string text, e2;
    CHECK(readTextFile(root + "/" + g.path, text, e2));
    CHECK_EQ(text, g.content);
  }
  for (const char* f : {"package.xml", "CMakeLists.txt", "README.md", "launch/sim.launch.py", "worlds/lab.sdf", "urdf/tiny.urdf", "config/tiny_sim_bridge.yaml"}) {
    CHECK(std::filesystem::exists(root + "/" + f));
  }
  // The input URDF is only read.
  CHECK_EQ(slurp(data("diffbot_gz.urdf")).find("added by gz_bridge_helper"), std::string::npos);

  // Not ready: nothing is written.
  Project bad;
  bad.loadUrdfFile(data("does_not_exist.urdf"));
  CHECK(!bad.ready());
  CHECK(hasIssue(bad.issues(), Severity::Error, "Cannot read the URDF"));
  const std::string out2 = tempDir("project_bad");
  CHECK(!bad.write(out2, err));
  CHECK(!std::filesystem::exists(out2 + "/diffbot_sim"));

  Project xacro;
  xacro.setUrdf("a.xacro", "<robot name='x' xmlns:xacro='http://www.ros.org/wiki/xacro'><xacro:include filename='a.xacro'/></robot>");
  CHECK(!xacro.ready());
  CHECK(hasIssue(xacro.issues(), Severity::Error, "xacro"));

  // YAML handling: unreadable file, removing a file, clearing.
  Project q = load("diffbot_plain.urdf");
  CHECK(!q.addYamlFile(data("missing.yaml")));
  CHECK(hasIssue(q.issues(), Severity::Error, "Cannot read the YAML file"));
  q.removeYaml(0);
  CHECK(!hasErrors(q.issues()));
  q.addYamlFile(data("overrides.yaml"));
  CHECK_EQ(q.yamls().size(), static_cast<size_t>(1));
  CHECK_EQ(q.plan().modelName, "tiny");
  q.clearYaml();
  CHECK_EQ(q.plan().modelName, "diffbot");
}

// ---------------------------------------------------------------------------------- verify

void testVerify() {
  Project p = load("diffbot_gz.urdf");
  const std::vector<TopicCheck> checks = checksFromPlan(p.plan());
  CHECK_EQ(checks.size(), p.plan().bridge.size());
  const TopicCheck* cmd = nullptr;
  const TopicCheck* clock = nullptr;
  for (const TopicCheck& c : checks) {
    if (c.rosTopic == "/cmd_vel") cmd = &c;
    if (c.rosTopic == "/clock") clock = &c;
  }
  CHECK(cmd && cmd->direction == Direction::RosToGz && !cmd->payload.empty() && cmd->listensInRos() && !cmd->publishesInRos());
  CHECK(clock && clock->publishesInRos() && clock->payload.empty());
  CHECK_EQ(samplePayload("geometry_msgs/msg/Twist"), "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}");
  CHECK_EQ(samplePayload("std_msgs/msg/Float64"), "{data: 0.0}");
  CHECK_EQ(samplePayload("sensor_msgs/msg/Image"), "");

  const auto topics = parseTopicList(
      "/clock [rosgraph_msgs/msg/Clock]\n/cmd_vel [geometry_msgs/msg/Twist]\n/multi [a/msg/A, b/msg/B]\n/parameter_events [rcl_interfaces/msg/ParameterEvent]\nnoise line\n");
  CHECK_EQ(topics.size(), static_cast<size_t>(4));
  CHECK_EQ(topics.at("/multi").size(), static_cast<size_t>(2));
  CHECK_EQ(topics.at("/clock")[0], "rosgraph_msgs/msg/Clock");

  double hz = 0;
  CHECK(parseHzAverage("average rate: 10.002\n\tmin: 0.099s max: 0.101s std dev: 0.00031s window: 11\naverage rate: 9.998\n", hz));
  CHECK_NEAR(hz, 9.998, 1e-9);
  CHECK(!parseHzAverage("WARNING: topic [/x] does not appear to be published yet\n", hz));
  CHECK_EQ(parseSubscriptionCount("Type: geometry_msgs/msg/Twist\nPublisher count: 0\nSubscription count: 1\n"), 1);
  CHECK_EQ(parseSubscriptionCount("Subscription count: 0"), 0);
  CHECK_EQ(parseSubscriptionCount("nothing"), -1);
  CHECK(echoShowsMessage("data: 1\n---\n"));
  CHECK(!echoShowsMessage("WARNING: topic does not appear to be published yet\n"));

  TopicCheck pub;
  pub.rosTopic = "/clock";
  pub.rosType = "rosgraph_msgs/msg/Clock";
  pub.gzTopic = "/clock";
  CHECK(evaluatePublished(pub, topics, "average rate: 998.1\n").state == CheckState::Pass);
  CHECK(evaluatePublished(pub, topics, "average rate: 998.1\n").detail.find("998.1 Hz") != std::string::npos);
  CHECK(evaluatePublished(pub, topics, "").state == CheckState::Fail);
  CHECK(evaluatePublished(pub, topics, "average rate: 0.000\n").state == CheckState::Fail);  // a rate of zero is no data
  CHECK(evaluatePublished(pub, topics, "average rate: 0.000\n").detail.find("no messages arrived") != std::string::npos);
  CHECK(evaluatePublished(pub, {}, "average rate: 1\n").state == CheckState::Fail);
  CHECK(evaluatePublished(pub, {}, "average rate: 1\n").detail.find("not in `ros2 topic list`") != std::string::npos);
  TopicCheck wrong = pub;
  wrong.rosType = "sensor_msgs/msg/Imu";
  CHECK(evaluatePublished(wrong, topics, "average rate: 1\n").detail.find("instead of sensor_msgs/msg/Imu") != std::string::npos);

  TopicCheck cmdCheck;
  cmdCheck.rosTopic = "/cmd_vel";
  cmdCheck.rosType = "geometry_msgs/msg/Twist";
  cmdCheck.gzTopic = "/model/diffbot/cmd_vel";
  cmdCheck.direction = Direction::RosToGz;
  cmdCheck.payload = samplePayload(cmdCheck.rosType);
  const std::string info1 = "Subscription count: 1\n", info0 = "Subscription count: 0\n";
  CHECK(evaluateCommanded(cmdCheck, topics, info1, true, true).state == CheckState::Pass);
  CHECK(evaluateCommanded(cmdCheck, topics, info1, true, false).state == CheckState::Fail);
  CHECK(evaluateCommanded(cmdCheck, topics, info1, false, false).state == CheckState::Pass);
  CHECK(evaluateCommanded(cmdCheck, topics, info1, false, false).detail.find("not verified") != std::string::npos);
  CHECK(evaluateCommanded(cmdCheck, topics, info0, true, true).state == CheckState::Fail);
  CHECK(evaluateCommanded(cmdCheck, {}, info1, true, true).state == CheckState::Fail);
  TopicCheck noPayload = cmdCheck;
  noPayload.payload.clear();
  CHECK(evaluateCommanded(noPayload, topics, info1, true, true).state == CheckState::Skipped);

  CHECK((launchCommand("pkg") == std::vector<std::string>{"ros2", "launch", "pkg", "sim.launch.py"}));
  CHECK((hzCommand("/x") == std::vector<std::string>{"ros2", "topic", "hz", "/x"}));
  CHECK((gzEchoCommand("/g") == std::vector<std::string>{"gz", "topic", "-e", "-t", "/g", "-n", "1"}));
  const std::vector<std::string> pc = publishCommand(cmdCheck);
  CHECK(pc.size() == 10 && pc[7] == "/cmd_vel" && pc[8] == "geometry_msgs/msg/Twist" && pc[9] == cmdCheck.payload);
  CHECK(std::string(checkStateName(CheckState::Pass)) == "pass");
}
