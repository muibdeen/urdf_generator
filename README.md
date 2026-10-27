# gz_bridge_helper

A Qt6 tool and ROS 2 (Jazzy) colcon package that takes a **URDF** (and optional YAML files),
works out which Gazebo systems and sensors the model needs, and writes a small ROS 2 package that
loads the model into **Gazebo Harmonic** with the right `ros_gz_bridge` configuration, so the
right messages are echoed in both directions. It can then build the package, launch it, and check
every bridged topic: pass or fail per row.

It is the second tool next to `urdf_generator` and is separate from it: it reads any URDF, not
only the ones that tool writes.

## What it takes away, and what it does not

It takes away the chores that show no robotics skill: writing the world file, the
`<gazebo>` plugin blocks, the sensor SDF, the bridge YAML with the right message-type pairs, the
launch file that wires them together.

It does not write your controllers, kinematics, state estimation or any C++/Python for you.
Everything it generates is ordinary, readable text with comments saying why each line is there
(open the tabs in the window, or the files in the package), and your URDF is never modified.
The numbers it measures (wheel separation, wheel radius, steering limit) are shown with the
reason, so you can check them against your own derivation.

## Build and run

```bash
cd ~/ros2_ws/src
cp -r /path/to/gz_bridge_helper .
cd ~/ros2_ws
sudo apt install qt6-base-dev libtinyxml2-dev libyaml-cpp-dev ros-jazzy-ros-gz
colcon build --packages-select gz_bridge_helper
source install/setup.bash

ros2 run gz_bridge_helper gz_bridge_helper_gui                    # the window
ros2 run gz_bridge_helper gz_bridge_helper_gui robot.urdf a.yaml  # with files already open
ros2 run gz_bridge_helper gz_bridge_helper_cli robot.urdf --list  # no window, see below
```

Without ROS (plain CMake) it builds too: `cmake -S . -B build && cmake --build build && ctest --test-dir build`.
Qt6 is only needed for the window; `-DGZBH_BUILD_GUI=OFF` builds the command line tool alone.

## Using the window

The layout follows `urdf_generator`: inputs on the left, results on the right.

**Left**
1. **URDF**: Open (or drop the file on the window). The summary shows links, joints and root.
   Plain URDF only: run `xacro` first if you have a xacro (the helper tells you).
2. **YAML files (optional)**: Add as many as you like; each is classified by what is inside
   (see "YAML files"). "Save overrides template..." writes a commented example to start from.
3. **Names and spawn pose**: package, model and world name, spawn x/y/z/yaw. Empty fields use the
   default shown in grey. Values typed here win over every YAML file.
4. **Detected (tick what to add)**: what the helper suggests, grouped as Sensors, Drive, Joints,
   Odometry, ros2_control. Click an item to read why it is suggested. Items marked "in the URDF"
   are already there and only get bridged; "will be added" goes into the *copy* of your URDF.
   Nothing is added unless it is ticked.

**Right, top:** the **bridged topics** as a table: ROS topic, direction (← Gazebo to ROS,
→ ROS to Gazebo, ↔ both), Gazebo topic, what it carries, both message types, and where the
entry came from (generated or which YAML file). Hover a row for details.

**Right, bottom:** tabs with the generated files (**Bridge YAML**, **Launch**, **World**,
**URDF changes** = only the lines added, **URDF** = the full copy, **Package files**), and the
**Run** tab. Below: the problems found (errors, warnings, information), then Copy and Save.
Saving is disabled while there are errors.

*File menu:* Open URDF (Ctrl+O), Add YAML (Ctrl+Shift+O), Reload (F5: reads the files again and
keeps your ticks), Save package (Ctrl+S), Save bridge YAML (Ctrl+Shift+S).

### The Run tab

1. **Workspace**: your colcon workspace (for example `~/ros2_ws`). Leave it empty when the package
   is already built and sourced in the terminal you started the window from.
2. **Build** writes the package to `<workspace>/src/<package>` and runs
   `colcon build --packages-select <package>`.
3. **Launch** runs `ros2 launch <package> sim.launch.py` (after sourcing the workspace). Untick
   "Gazebo window" for a headless run (`gui:=false`). **Stop** sends Ctrl-C to the whole launch.
4. **Check topics** (or tick "Check after N s" to do it by itself once Gazebo had time to start):
   - `ros2 topic list -t`: is the topic there, with the right message type?
   - Gazebo to ROS topics: `ros2 topic hz` listens for a few seconds; messages must arrive.
   - ROS to Gazebo topics: someone must be subscribed (`ros2 topic info`), then a harmless
     all-zero message is published while `gz topic -e -n 1` listens on the Gazebo side.
   - Every row ends as **Pass**, **Fail** (with the reason) or **Skipped** (no safe test message
     for that type, or cancelled).
   "Show the same commands for a terminal" lists what the buttons run, so you can do it by hand.

If you change the plan while Gazebo is running, the table keeps describing what was launched and
says so; stop and launch again to use the new plan.

## What is detected

| Item id | What | How |
|---|---|---|
| `sensor:<name>` | camera, IMU, lidar, GPS/GNSS | `<gazebo><sensor>` blocks already in the URDF, or links named like them (`camera`, `imu`, `lidar`/`laser`/`scan`, `gps`/`gnss`/`navsat`) |
| `sensortopic:<name>` | a `<topic>` for an existing sensor that has none | Gazebo names it after how sdformat lumps links, which you cannot predict, so one is written |
| `drive:diff` | DiffDrive (several wheels per side = skid steering) | continuous/revolute joints with a horizontal axis whose name or link says wheel/tire, or that carry a cylinder or sphere; left/right by REP-103 (+y is left) |
| `drive:mecanum` | MecanumDrive | exactly four wheels, all with rollers (links with a name containing `roller`) |
| `drive:ackermann` | AckermannSteering | two vertical steering joints above the wheels; driven wheels are the unsteered ones |
| `joint_states` | JointStatePublisher → `/joint_states` | any movable joint |
| `jointpos:<joint>` | JointPositionController (command with a `Float64`) | movable joints that are not wheels |
| `odometry` | OdometryPublisher | only offered when there is no drive |
| `ros2control:plugin`, `ros2control:hardware` | `gz_ros2_control` plugin / hardware plugin | when the URDF has a `<ros2_control>` section |

Measured from the URDF's forward kinematics: wheel separation, wheel base, kingpin width,
steering limit (from the joint limits) and wheel radius (how far the surface of the wheel's
cylinders and spheres, and of what is fixed to it such as mecanum rollers, reaches from the
axle). Warnings: wheel axle not +y, mixed mecanum and ordinary wheels, wheels of different radii.

## Topic names

Gazebo topics are scoped to the model; ROS topics are the plain names you would expect.

| ROS | Gazebo | Type |
|---|---|---|
| `/clock` (always) | `/clock` | `rosgraph_msgs/Clock` |
| `/cmd_vel` | `/model/<model>/cmd_vel` | `geometry_msgs/Twist` |
| `/odom` | `/model/<model>/odometry` | `nav_msgs/Odometry` |
| `/tf` | `/model/<model>/tf` | `tf2_msgs/TFMessage` |
| `/joint_states` | `/model/<model>/joint_state` | `sensor_msgs/JointState` |
| `/joint/<joint>/cmd_pos` | `/model/<model>/joint/<joint>/cmd_pos` | `std_msgs/Float64` |
| `/<sensor>/image_raw`, `/<sensor>/camera_info` | `/model/<model>/sensor/<sensor>/image`, `.../camera_info` | `Image`, `CameraInfo` |
| `/<sensor>/data` (IMU) | `/model/<model>/sensor/<sensor>/imu` | `sensor_msgs/Imu` |
| `/<sensor>/scan`, `/<sensor>/points` (lidar) | `.../scan`, `.../scan/points` | `LaserScan`, `PointCloud2` |
| `/<sensor>/fix` (GPS) | `.../navsat` | `sensor_msgs/NavSatFix` |

Sensors that already have a `<topic>` in the URDF keep their Gazebo topic. The ROS name of any
of them can be changed with `ros_topic:` in an overrides file. Camera entries set the header's
`frame_id` to the optical frame; the other sensors to their link.

## YAML files

Each file may be one or more of these kinds, told apart by content:

* **ros2_control controllers**: a `controller_manager` section. The helper checks it against the
  URDF (unknown joint names with a "did you mean", command and state interfaces, update rate vs
  the physics step), copies the file into the package and starts the controllers with spawners.
  In this mode only `/clock` and the sensors go through the bridge; commands and joint states
  come from the controllers.
* **Existing ros_gz_bridge config**: a list of `topic_name` / `ros_topic_name`+`gz_topic_name`,
  `ros_type_name`, `gz_type_name`, `direction`, ... Entries are validated the way `ros_gz_bridge`
  would read them (it silently skips bad ones). Valid entries are added to the bridge and
  replace a generated entry for the same ROS topic.
* **Overrides**: this helper's own settings. "Save overrides template..." writes one with every
  key and a comment; unknown keys are reported with a suggestion. Keys: `package_name`,
  `model_name`, `use_sim_time`, `world` (`name`, `step_size`, `gravity`, `spherical_coordinates`),
  `spawn` (`x y z roll pitch yaw`), `sensors.<name>` (`enabled`, `ros_topic`, `frame_id`,
  `update_rate`, `width`, `height`, `horizontal_fov`, `samples`, `range_min`, `range_max`),
  `drive` (`enabled`, `wheel_separation`, `wheel_radius`, `ros_cmd_vel_topic`, `ros_odom_topic`,
  `max_linear_velocity`, `max_angular_velocity`), `joints` (`state_publisher`, `position_control`,
  `p_gain`, `i_gain`, `d_gain`), `bridge` (extra entries), `exclude` (ROS topics to leave out).

Later files win key by key. A spawn height is computed from the lowest shape of the URDF unless
`spawn.z` is given.

## The generated package

```
<package>/
  package.xml  CMakeLists.txt  README.md
  launch/sim.launch.py            Gazebo + robot_state_publisher + spawn + parameter_bridge
  config/<package>_bridge.yaml    the bridge entries, each with a comment saying what it carries
  worlds/<world>.sdf              physics, the systems the sensors need, ground, light
  urdf/<model>.urdf               your URDF + the added <gazebo> blocks (marked with a banner)
```

Your original is recoverable by removing the marked block (and the `<topic>` lines it lists); the
tests check that the copy is the original plus insertions only. `package://` mesh paths work: the
launch file adds each mesh package's parent folder to `GZ_SIM_RESOURCE_PATH`. Fill in the
maintainer line in `package.xml` before you publish it.

## Command line

```
gz_bridge_helper_cli robot.urdf [more.yaml ...] [--out DIR] [--list] [--only id,id] [--without id,id]
```
`--list` prints the detected items (ticked ones marked `[x]`), the bridge as
`ROS <topic> <-/->/<-> GZ <topic>` and the problems; `--out DIR` writes `DIR/<package>`.
Exit code 1 if there are errors.

## Tests

`ctest --test-dir build` (or `colcon test`) runs:

* `core`: about 1,180 checks of parsing, classification, detection, planning, the emitters and the
  logic that reads `ros2 topic` output, on hand-written URDFs and on every robot that
  `urdf_generator` can make (12 of them).
* `gui`: drives the real window offscreen, including Build, Launch, Stop and Check topics against
  stand-ins for `ros2`, `gz` and `colcon` (`test/mock_bin`), with real processes and process groups.
* `generated_packages`: every generated package is checked with `xmllint`, `check_urdf`, PyYAML
  and a Python compile, and the patched URDF is diffed against the original.
* `drive_numbers_cross_check`: wheel separation, base, kingpin width, steering limit and radius
  are recomputed from the URDF with `yourdfpy` and compared with what is in the plugins.

The last two are only registered when `python3-yaml` / `yourdfpy` are installed.

## Known limits

* **Targets ROS 2 Jazzy with Gazebo Harmonic** (`ros_gz`, `gz-sim8`, plugin names `gz-sim-*-system`).
  The names, parameters and message-type pairs were taken from the `ros_gz`, `gz-sim`, `sdformat`
  and `gz_ros2_control` sources. The sandbox the tool was written in has neither ROS nor
  Gazebo, so the end-to-end run (a real simulation) is the first thing to try on your machine;
  the Run tab is there to tell you quickly what is not right.
* Camera, IMU, lidar (`gpu_lidar`), GPS, magnetometer, air pressure and altimeter sensors are
  bridged. Depth/RGB-D cameras and other types are reported and left to your own bridge YAML.
* With "Gazebo window" unticked, Gazebo runs server-only (`-s`). Cameras and lidars still render
  there, which on a machine without a display or GPU may need `--headless-rendering` in the
  `gz_args` of `launch/sim.launch.py`.
* The PID gains for joint position control (`10, 0, 0.5`) are starting values; tune them.
* Mecanum drive is set up from the plugin's point of view: Gazebo's rollers are not simulated
  as real contact, so expect idealised sideways motion.
* The URDF must be plain URDF (no xacro), with one tree of links.

Apache-2.0.
