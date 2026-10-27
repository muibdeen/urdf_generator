# urdf_generator

A Qt6 GUI and ROS 2 (Jazzy) colcon package that asks for the dimensions of a robot and
generates both a **xacro** file and a flat **URDF** for it. Visuals are plain URDF
primitives (boxes, cylinders, spheres). Every link also gets a matching collision shape
and an inertia computed from its shape and mass, so the model works in simulators too.

## Supported robots and what you are asked for

| Robot | Inputs |
|---|---|
| Hexapod | coxa / femur / tibia lengths, distance of the six leg mounts from the centre, body height, link thickness, masses, joint limits |
| Arm linkage system | number of links; for each link: revolute or sliding, axis (X/Y/Z), **Cartesian (x, y, z) or Polar (r, θ, z)** offset from the previous frame, limits, mass; tool-frame offset |
| Differential drive | chassis shape (square or circular) and size, wheel radius / width, wheel position **x y z** from the centre, optional front/rear ball casters, **Standard or Mecanum for each wheel** |
| 4-wheeled | chassis, wheel radius / width, position x y z of the front-left wheel (the other three mirror it), **Standard or Mecanum for each wheel** |
| Ackermann drive | chassis, wheel radius / width, wheelbase, track width, steering limit; rear wheels driven, front wheels steered, **Standard or Mecanum for each wheel** |
| Quadruped | body size, hip distance from the centre (x and y), link lengths of the 3-joint legs |
| Biped | torso size, hip offset from the centre, thigh / shank lengths, foot size |

**Sensors:** every robot type can also carry sensors (see "Sensors" below).

**Mecanum wheels:** on the three wheeled robots the "Wheel type" group has a Standard / Mecanum
choice for every wheel (and "All standard" / "All mecanum" buttons), so you can mix them, plus the
number of rollers (3 to 16, default 8). A mecanum wheel is a hub with barrel-shaped rollers fixed
round its rim, each roller turned 45° to the axle and lying in the tangent plane of the rim. The
hub keeps the name of the plain wheel (`front_left_wheel_link` / `front_left_wheel_joint`), so
swapping a wheel does not move it or change its joint. The rollers are extra links named
`front_left_roller_1_link` ... with fixed joints (`..._roller_1_joint`) and take 40 % of the wheel
mass between them. Their size follows the wheel width (roller radius = width / 4), and the xacro
has the derived properties (`roller_radius`, `roller_length`, `hub_radius`, ...) for tweaking.

The slant of the rollers is chosen from where the wheel sits, so that the rollers touching the
ground point towards the middle of the robot (the layout that mecanum drive kinematics assume):
front-left and rear-right wheels one way, front-right and rear-left the other. In the xacro this
is the `hand` argument of the `mecanum_wheel` macro (+1 or -1); flip it if you mount a wheel the
other way round. For the differential drive the wheel counts as a front wheel when its x is 0 or
more and as a rear wheel when it is negative. Seen from above you see the top of each wheel,
whose rollers slant the opposite way to the ones on the ground.

Legs have three joints each (hexapod/quadruped: coxa, femur, tibia; biped: hip, knee, ankle).
The GUI shows a **3D view** of the robot that updates as you type, the xacro and URDF text,
a tree of the links and joints, and lists any problems (missing mass, lower limit above upper
limit, ...). Saving is disabled while there are errors.

3D view controls: drag to orbit, right-drag (or Shift-drag) to pan, mouse wheel to zoom,
double-click to reset. The buttons above it jump to an isometric, front, side or top view, and
"Link frames" draws the x (red), y (green) and z (blue) axes of every link as thin lines;
"Sensor axes" (on by default) draws the frame of every sensor, see below. The grid
lies on the lowest point of the robot (the ground, for the wheeled robots) and its red and green
lines are the world x and y axes. The camera stays put while you edit, so you can see a
dimension change against the grid; it only zooms out by itself if the robot grows out of view
and you have not moved the camera. The view is rendered in software, so it needs no OpenGL
and also works over SSH or in a VM.

**Joint sliders:** the panel beside the 3D view has a slider and a number box for every movable
joint (degrees for revolute and continuous joints, metres for sliding ones), so you can pose
the robot and check that legs, arms and steering move the way you meant. Each slider covers
the joint's limits, and the joints of one leg, wheel or arm are grouped. "Reset pose" puts every
joint back at zero. The pose is kept while you edit dimensions, so you can lengthen a link and
watch the posed robot change, and is forgotten when you switch robot type. It is for looking
only: the generated xacro and URDF never depend on it. Wheels are plain cylinders, so a
turning wheel is only visible with "Link frames" on. If the robot swings out of view the
camera backs away; it never zooms back in or re-aims on its own, and never touches a view you
framed yourself. Untick "Joint sliders" above the view to hide the panel.

## Sensors

Every robot page ends with a **Sensors** box. Pick a type (camera, IMU, LiDAR, GPS or odometry)
and press **Add sensor**; each robot type keeps its own list, and a robot can have any number of
sensors. For each sensor you choose:

- **Name** (letters, digits and `_`). The generated link is `<name>_link` and its joint
  `<name>_joint`. A new sensor is called `camera`, `camera_2`, ... and is renamed along when you
  change its type, unless you gave it a name of your own.
- **Mounted on**: any link of the robot (mecanum rollers are left out; mount on the wheel hub).
  The sensor is attached at the **root** of that link, which is the origin of the link's frame,
  where its joint is.
- **Distance**: a slider and a number box that move the sensor **along the link**, from the root.
  The direction is the link's own axis: a rod's axis, the longest side of a box, `z` for a ball.
  It points from the root towards the body, so a leg link runs from its joint to its tip, and the
  range is the length of the link. A body that is centred on its root (a wheel, a chassis) can
  be slid both ways. A bare frame has no length, so the slider is disabled there. If you shorten
  a link below the distance you asked for, the sensor stays at the end of the link and goes back
  to your distance when the link is long enough again.
- **Offset** x / y / z: an extra shift on top of the distance, in the frame of the link.
- **Rotation** roll / pitch / yaw in degrees (URDF `rpy`), relative to the link's frame. At zero
  the sensor's x axis points along the link's x axis.

Choosing a different link puts the sensor at the root of the new link (distance and offsets back
to zero) and keeps its rotation. A new sensor starts on top of the main body (`base_link`).

**What is generated.** A sensor is a small link with a fixed joint to the link it is mounted on,
with visual, collision and inertial like every other link: a box for a camera and an IMU, a
cylinder for a LiDAR, a flat cylinder for a GPS and a thin disc for odometry, each in its own
colour. Its frame follows REP 103: x forward, y left, z up. A camera also gets a
`<name>_optical_frame` (z forward, x right, y down, the convention of image pipelines) as a
child. In the xacro the placement is a group of properties per sensor
(`sensor_<name>_distance`, `_offset_x/y/z`, `_roll`, `_pitch`, `_yaw`), so you can move a
sensor by editing a number; the plain URDF has the resolved values.

**The sensor axes in the 3D view.** The frame of each sensor is drawn as three **thick,
outlined arrows with arrow heads** and `x` `y` `z` letters, a dot at its origin, a name tag in
the colour of the sensor type, and a dashed line back to the root of the link it is mounted on
(a ring marks the root). Link frames are only thin plain lines, and the "Link frames" overlay
does not draw the sensor frames, so the two cannot be mistaken for each other. The arrows turn
with the sensor and move with the joint sliders. Untick "Sensor axes" to hide them.

## Build

On Ubuntu 24.04 with ROS 2 Jazzy:

```bash
cd ~/ros2_ws/src
cp -r /path/to/urdf_generator .
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y     # pulls in qt6-base-dev, xacro, rviz2, ...
colcon build --packages-select urdf_generator
source install/setup.bash
```

Without ROS (just the generator and GUI): `sudo apt install qt6-base-dev cmake g++`, then
`cmake -S . -B build && cmake --build build`.

## Use

```bash
ros2 run urdf_generator urdf_generator_gui                 # the GUI
ros2 launch urdf_generator view_robot.launch.py model:=$HOME/hexapod.xacro   # view in RViz
ros2 run urdf_generator urdf_generator_cli all /tmp/robots # default model of every type, no GUI
```

`view_robot.launch.py` accepts a `.xacro` or a `.urdf`, starts `robot_state_publisher`, RViz and
`joint_state_publisher_gui` (sliders for every joint; `gui:=false` for none).

**Using the xacro:** open it and change the `<xacro:property>` values at the top to resize the
robot, then `xacro hexapod.xacro > hexapod.urdf`. Legs and wheels are `<xacro:macro>`s that are
instantiated at the bottom of the file. The `.urdf` the GUI writes is already resolved and
loads directly.

## Conventions

- Metres, kilograms, seconds. The GUI takes angles in degrees; files contain radians.
- +x forward, +y left, +z up. `base_link` is at the centre of the body or chassis.
- Wheeled robots also have a `base_footprint` frame on the ground; the model is placed so the
  wheels (and casters) touch z = 0.
- Wheels roll about their local y axis; a mecanum wheel's hub does the same and its rollers are fixed to it.
- Hexapod: `leg1` points along +x and legs go counter-clockwise every 60°. With all joints at
  zero the coxa and femur stick out horizontally and the tibia hangs straight down.
- Quadruped and biped: with all joints at zero the legs hang straight down. The biped knee
  bends backwards from 0 up to its limit.
- Arm: `link_1 … link_N` with `joint_1 … joint_N`, and a `tool0` frame at the end. Each link is
  drawn as a rod from its own joint to the next joint (or to the tool frame). Polar θ is measured
  from the previous frame's +x axis about +z.
- Inertia uses the solid-body formulas for the link's primitive shape, about the shape's centre.
- Sensors: x forward, y left, z up (REP 103). A camera's `_optical_frame` is z forward, x right, y down.
  The "root" of a link is the origin of its frame; "distance" is measured from it along the link.

## Tests

```bash
colcon test --packages-select urdf_generator && colcon test-result --verbose
```

- `core`: unit tests of the generator (numbers, inertia, every robot, validation) and of the
  3D scene and renderer (mesh orientation, forward kinematics, joint poses and limits, camera,
  z-buffer, clipping).
- `gui`: drives the real window without a display (checks the GUI defaults equal the library
  defaults and exercises arm links, Cartesian/Polar conversion, chassis shape, errors, saving,
  and the 3D view: live updates, mouse orbit / pan / zoom / reset, presets, frames, joint sliders).
- `xacro_matches_urdf*`: expands every generated xacro with the real `xacro` tool and checks it
  matches the flat URDF exactly (36 models, including unusual settings and sensors).
- The mecanum wheels are covered by `core` (roller positions, 45° slant, tangent plane, handedness per
  wheel, ground contact for 3 to 16 rollers, masses, rigid under steering and wheel turning) and by
  `gui` (the choices, the exact text, errors); mixed-wheel robots are part of the xacro / URDF /
  `check_urdf` / `scene_matches_urdf_fk` checks below.
- Sensors are covered by `core` (the axis and range of every kind of link, placement maths, the
  optical frame, following the joints, xacro properties, materials, bad / repeated / clashing
  names, unknown links, a sensor on every link of every robot) and by `gui` (adding and removing,
  the link list and slider ranges, remembering a distance, a link that disappears, per-page
  lists, the overlay and its switch). Seven models with sensors on legs, tilted and sliding arm
  links, steered and mecanum wheels and a biped take part in the xacro / URDF / `check_urdf` /
  `scene_matches_urdf_fk` checks below.
- `scene_matches_urdf_fk`: when `yourdfpy` is installed, loads every generated URDF with it and
  checks that the 3D view's link poses and overall bounds agree, both with every joint at zero
  and with every joint moved to a different value.
- `check_urdf`: runs urdfdom's `check_urdf` on every generated URDF when it is installed.

## Not included

- No `ros2_control` or Gazebo tags, no joint damping or friction. Add them to the xacro by hand.
- Sensors are frames with a small marker body. There is no `<gazebo>` sensor block, no noise or
  rate settings, no camera intrinsics, and odometry is only a frame (nothing publishes it), so a
  simulator will not produce data from them until you add the sensor plugins to the xacro.
  The marker sizes are fixed.
- Mecanum rollers are fixed to the hub: they give the wheel its shape and mass but cannot spin
  freely, so a physics simulator will not make the robot slide sideways by itself. Drive a mecanum
  robot with a mecanum controller or a planar-motion plugin acting on the four wheel joints. The
  roller count is part of the generated structure, not an xacro property: change it in the GUI.
  The wheels touch the ground plane to within a millimetre (one roller touches it exactly).
- The 3D view shows the robot's shape only: no collision shapes, no dynamics, and no check for
  self-collision when you pose it. Use the launch file to look at the robot in RViz.

## License

Apache-2.0 (see `LICENSE`).
