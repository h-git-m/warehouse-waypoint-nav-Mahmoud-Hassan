# Autonomous Warehouse Waypoint Delivery Robot

A TurtleBot3 Burger autonomously maps a warehouse environment, localizes itself with AMCL, and executes a
named-waypoint delivery mission using Nav2 — starting and ending at a Charging Station, visiting a Loading
Station (with a mandatory 30-second dwell), a Storage Area, and a Shipping Station. All waypoints are
visualized live in RViz as color-coded markers (blue = inactive, green = active navigation goal).

---

## 1. Project Overview and Mission

**Scenario:** Build an autonomous TurtleBot3 warehouse robot that maps and localizes itself inside a
simulated warehouse, then navigates through a sequence of named real-world locations.

**Required mission:**

1. Start at the **Charging Station (Home)**
2. Navigate to the **Loading Station**
3. Wait exactly **30 seconds**
4. Navigate to the **Storage Area**
5. Navigate to the **Shipping Station**
6. Return to the **Charging Station**
7. Wait for each Nav2 result before sending the next goal
8. Stop the mission and report the location if any goal fails

The full pipeline covers: simulation setup → SLAM mapping → AMCL localization → Nav2 navigation →
autonomous multi-waypoint mission execution with live RViz marker feedback.

---

## 2. Repository and Package Structure

```
warehouse-waypoint-nav-Mahmoud-Hassan/
├── robot_navigation/
│   ├── config/
│   │   ├── amcl.yaml
│   │   ├── planner_server.yaml
│   │   ├── controller_server.yaml
│   │   ├── behavior_server.yaml
│   │   └── bt_navigator.yaml
│   ├── launch/
│   │   └── nav2_bringup.launch.py
│   ├── map/
│   │   ├── warehouse_world_map.pgm
│   │   └── warehouse_world_map.yaml
│   ├── scripts/
│   │   └── send_goal.py
│   ├── rviz/
│   │   ├── AMCL_warehouse.rviz
│   │   └── Nav2.rviz
│   ├── CMakeLists.txt
│   └── package.xml
├── warehouse_waypoints/
│   ├── resource/
│   │   └── warehouse_waypoints
│   ├── warehouse_waypoints/
│   │   ├── __init__.py
│   │   └── waypoint_mission.py
│   ├── launch/
│   │   └── waypoint_mission.launch.py
│   ├── rviz/
│   │   └── Nav2_waypoints.rviz
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── warehouse_world/
│   ├── config/
│   │   └── gz_bridge_ros.yaml
│   ├── image/
│   │   └── warehouse_world.png
│   ├── launch/
│   │   └── warehouse_storage_launch.launch.py
│   ├── maps/
│   │   ├── warehouse_world_map.pgm
│   │   └── warehouse_world_map.yaml
│   ├── posegraph/
│   │   ├── warehouse_world.data
│   │   └── warehouse_world.posegraph
│   ├── worlds/
│   │   └── warehouse_storage.sdf
│   ├── models/
│   ├── rviz/
│   │   └── warehouse.rviz
│   ├── CMakeLists.txt
│   └── package.xml
├── images/
│   └── (mapping, localization, navigation, and waypoint marker screenshots)
└── README.md
```

**Package roles:**

| Package | Purpose |
|---|---|
| `robot_navigation` | AMCL + full Nav2 bringup (planner, controller, behavior server, BT navigator), saved maps, and RViz configs |
| `warehouse_waypoints` | Waypoint mission node: sends the ordered sequence of Nav2 goals, handles the 30s dwell, and publishes the `/waypoint_markers` MarkerArray |
| `warehouse_world` | Custom Environment built by Etgah. Turtlebot3_burger urdf and gazebo plugins are integrated into this environment to fullfill task requirements. `warehouse_storage_launch.launch.py` is used to launch the environment with turtlebot3_burger at the origin |
---

## 3. Workspace Build Instructions

```bash
cd ~/workspaces/turtlebot_ws/src

# Clone this repository package (warehouse_world)
git clone https://github.com/ETGAH/warehouse_world.git

cd ~/workspaces/turtlebot_ws or cd ..
colcon build
source install/setup.bash
```

Add `source ~/workspaces/turtlebot_ws/install/setup.bash` to your `.bashrc` (or re-source it in every new
terminal you open for this project).

---

## 4. Launching TurtleBot3 Inside the Warehouse World

After cloning the `warehouse_world` package, TurtleBot3 must be spawned into it. Since `ros2 launch`
needs both the robot's description and its Gazebo plugin definitions, these were located first:

```bash
# Find turtlebot3_description and its URDF
ros2 pkg prefix turtlebot3_description
find $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description -iname "*burger*"
# → /opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf
```

This URDF path is passed to a `robot_state_publisher` node inside the launch file, so RViz/TF have a
description to work from.

```bash
# Find turtlebot3_gazebo and its burger-related files
ros2 pkg prefix turtlebot3_gazebo
find $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo -iname "*burger*"
find /root/workspaces/turtlebot_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger -type f
cat /root/workspaces/turtlebot_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

This `model.sdf` file is the one Gazebo actually spawns — it includes the `DiffDrive` and
`JointStatePublisher` plugins, and is used in the `-file` argument of the spawn node.
(The plain URDF from `turtlebot3_description` has no `<gazebo>`/`<plugin>` tags at all, so spawning from it
alone produces a robot with no working motion or joint states.)

The `JointStatePublisher` plugin's explicit topic override was checked:

```bash
grep -B3 -A15 "JointStatePublisher\|joint-state-publisher" \
  /root/workspaces/turtlebot_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

This confirmed the robot publishes joint states on `joint_states` (not the default long scoped topic path),
so the `gz_bridge_ros` config was updated accordingly. Other required bridge entries — `/cmd_vel`, `/odom`,
`/tf`, and `/scan` — were added the same way.

Once the launch file and bridge config were updated:

```bash
cd ~/workspaces/turtlebot_ws
colcon build --packages-select warehouse_world
source install/setup.bash
ros2 launch warehouse_world warehouse_storage_launch.launch.py
```

Alternatively, run the launch file directly inside gazebo by starting your workspace first and then selecting the gazebo launch file created for this workspace.

This launch file:
- Starts Gazebo
- Loads the warehouse world
- Starts `robot_state_publisher`
- Creates the ROS–Gazebo bridge for all required topics
- Spawns the robot into the simulation

---

## 5. Mapping the Warehouse with SLAM Toolbox

After adding the `slam_toolbox` mapping/localization package to `src/`, building, and sourcing:

```bash
ros2 launch slam_toolbox_demo slam_toolbox_online_async.launch.py
```

With the warehouse world already running, in a separate terminal:

```bash
rviz2
```

Add **RobotModel**, **LaserScan**, **Map**, and **TF** displays, with **Fixed Frame = `map`**. Teleoperate
the robot through every accessible aisle, wall, and corner until the map is fully generated with no large
unexplored gaps or duplicated walls.

**Sensor range tuning:** the TurtleBot3 Burger's default LiDAR range (3.5 m) was insufficient for reliable
scan matching in the warehouse's large, mostly open aisles — long, unterminated laser rays produced
starburst-shaped gaps at the edge of sensor range. The range was increased in both the SLAM config and the
robot's actual sensor definition:

```yaml
# slam_toolbox_online_async.yaml
max_laser_range: 12.0        # was 3.5
loop_search_maximum_distance: 8.0   # was 3.0 — see "Problems Encountered" below
```

```bash
# Update the LiDAR's simulated max range in the source model.sdf (not install/, so it survives rebuilds)
find /root/workspaces/turtlebot_ws/src -iname "model.sdf" | grep turtlebot3_burger
sed -i "0,/<max>3.5<\/max>/s//<max>12.0<\/max>/" \
  /root/workspaces/turtlebot_ws/src/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf

cd ~/workspaces/turtlebot_ws
colcon build --packages-select turtlebot3_gazebo
source install/setup.bash
```

---

## 6. Saving the Warehouse Map

Before closing any running terminals:

```bash
mkdir -p ~/workspaces/turtlebot_ws/src/robot_navigation/map
cd ~/workspaces/turtlebot_ws/src/robot_navigation/map
ros2 run nav2_map_server map_saver_cli -f warehouse_world_map
```

This produces `warehouse_world_map.pgm` and `warehouse_world_map.yaml`. The `.pgm` can be visually
inspected with a PGM viewer extension in VS Code.

**Reloading the saved map to verify it independently of SLAM:**

```bash
cd ~/workspaces/turtlebot_ws/src/robot_navigation/map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=warehouse_world_map.yaml
```

In another terminal:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

In RViz, add a **Map** display and — under its **QoS Settings** — set **Durability Policy** to
**Transient Local**, otherwise the map will not appear.

---

## 7. Launching and Testing AMCL

The map path was updated to point at the saved warehouse map, and `laser_max_range` in the AMCL config was
updated to match the sensor change made earlier:

```yaml
# amcl.yaml
laser_max_range: 12.0   # was 3.5
```

```bash
ros2 launch robot_navigation amcl.launch.py
rviz2
```

Add the following displays with **Fixed Frame = `map`**:

| Display | Topic | Extra settings |
|---|---|---|
| RobotModel | `/robot_description` | — |
| LaserScan | `/scan` | — |
| Map | `/map` | Durability Policy = **Transient Local** |
| TF | — | — |
| ParticleCloud | `/particle_cloud` | Reliability Policy = **Best Effort** |

> This configuration is saved and can be loaded directly from `rviz/AMCL.rviz`.
>
> **Known issue:** if the map fails to load after applying this configuration, kill the AMCL launch
> terminal and re-run it.

**Setting the initial pose:** either pre-defined in `config/amcl.yaml`, or set interactively in RViz with
**2D Pose Estimate**, clicking at the location and heading matching the robot's actual Gazebo pose.

**Validation:**

```bash
ros2 topic list | grep 'amcl'
ros2 topic echo /amcl_pose --once      # confirm the pose updates as the robot moves
ros2 run tf2_ros tf2_echo map odom
```

Example healthy output:

```
At time 633.200000000
- Translation: [-0.116, -0.039, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.021, 1.000]
- Rotation: in RPY (degree) [0.000, -0.000, 2.400]
```

A small, stable correction like this confirms AMCL is actively publishing the `map → odom` transform and
that `/amcl_pose` tracks the robot correctly while driving.

---

## 8. Launching the Complete Nav2 System

The map path was updated in `nav2_bringup.launch.py`. Two costmap/planner parameters were also tuned for
this warehouse layout:

```yaml
# planner_server.yaml
inflation_radius: 0.2   # was 0.5 — larger inflation closed off narrow gaps and produced "no path found"

# controller_server.yaml
local_costmap:
  width: 6    # was 3
  height: 6   # was 3
```

With TurtleBot3 already running in `warehouse_world`:

```bash
ros2 launch robot_navigation nav2_bringup.launch.py
```

Confirm every lifecycle node is active:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
# each should report: active [3]
```

```bash
rviz2
```

Add all the AMCL displays above, plus:

| Display | Topic | Extra settings |
|---|---|---|
| — | `/global_costmap/costmap/map` | Color Scheme = **costmap** |
| — | `/local_costmap/costmap/map` | Color Scheme = **costmap** |
| — | `/plan/path` | Color = **orange** |
| — | `/local_plan/path` | — |

> This configuration is saved and can be loaded directly from `rviz/NAV2.rviz`.

Set the initial pose (as in Section 7), confirm the LiDAR aligns with the map walls, then test navigation:

**Via RViz:** click **2D Goal Pose**, click an empty spot on the map, drag to set final orientation, and
release. The robot plans a path and navigates to the goal.

**Via terminal:**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Via Python action client:**
```bash
ros2 run robot_navigation send_goal.py   # sends x=1.5, y=0.5, yaw=0.0
```

`/cmd_vel` was confirmed to use `geometry_msgs/msg/Twist`, matching what the DiffDrive plugin expects.

---

## 9. Waypoint Names, Positions, and Orientations

Each waypoint's position and orientation was recorded in the `map` frame by driving the robot to the
target location under AMCL localization, then reading:

```bash
ros2 topic echo /amcl_pose --once
```

| Name | x | y | z | qz | qw | Notes |
|---|---|---|---|---|---|---|
| **Home** (Charging Station) | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 | Mission start and final destination |
| **Loading Station** | 7.782053842839245 | 7.420772652392393 | 0.0 | 0.0 | 1.0 | Wait here 30 seconds |
| **Storage Area** | 20.000934662165207 | -2.415622611057596 | 0.0 | 0.0 | 1.0 | Second navigation goal |
| **Shipping Station** | -5.930928476565102 | -5.406362903539103 | 0.0 | 0.0 | 1.0 | Third navigation goal |


---

## 10. Mission Route

```
Charging Station  →  Loading Station  →  Wait 30 sec  →  Storage Area  →  Shipping Station  →  Return to Charging
    (Home)                                                                                          (Home)
```

Each goal begins only after the previous goal succeeds. The mission starts only once the robot is
localized at the Charging Station.

---

## 11. RViz Waypoint-Marker Behavior and Running the Waypoints Package

Implemented in the `warehouse_waypoints` package (`waypoint_mission.py`):

- All four waypoints are published as a `visualization_msgs/msg/MarkerArray` on `/waypoint_markers`,
  each placed at its stored pose in the `map` frame with a text label showing the station name.
- **Blue** = inactive waypoint.
- **Green** = the currently active Nav2 navigation goal.
- The full `MarkerArray` is republished every time the active goal changes, so only one waypoint is ever
  shown green at a time.
- On startup, all four markers are published immediately (blue), before the mission begins — the mission
  itself only starts once `start` is typed into the terminal running the node.

---

With `warehouse_world` already running (Section 4), bring up Nav2 (Section 8) and then the waypoint mission node in
two separate terminals:

```bash
ros2 launch robot_navigation nav2_bringup.launch.py
ros2 run warehouse_waypoints waypoint_mission
```

**RViz setup for the waypoint mission:**

In addition to the standard Nav2 RViz configuration (Section 8), add a **MarkerArray** display (by
display type) and set its topic to `/waypoint_markers`. Then set the robot's **2D Pose Estimate** to match
its actual Gazebo pose.

Alternatively, simply open the saved `rviz/NAV2_waypoint.rviz` configuration (which already includes the
`/waypoint_markers` display) and just set **2D Pose Estimate**.

Once everything is set — Nav2 fully active, RViz showing the map/costmaps/markers, and the initial pose
set — type `start` in the terminal running `ros2 run warehouse_waypoints waypoint_mission` to begin the
autonomous mission.

---

## 12. Required Terminal Output

**Waypoint mission node output (actual captured run):**

```
ros2 run warehouse_waypoints waypoint_mission
[INFO] [1785563764.429697221] [waypoint_mission]: Published waypoint markers (active goal: None)
Waypoint markers published on /waypoint_markers.
Type 'start' and press Enter to begin the mission.
start
[INFO] [1785563902.331685644] [waypoint_mission]: Waiting for Nav2 action server...
[INFO] [1785563902.332488199] [waypoint_mission]: Nav2 action server available. Starting mission.
[INFO] [1785563902.333505077] [waypoint_mission]: Navigating to: Loading Station
[INFO] [1785563902.335416009] [waypoint_mission]: Published waypoint markers (active goal: loading)
[INFO] [1785563984.327935467] [waypoint_mission]: Reached: Loading Station
[INFO] [1785563984.328406703] [waypoint_mission]: Waiting 30 seconds at Loading Station...
[INFO] [1785564014.330474806] [waypoint_mission]: Wait complete. Continuing mission.
[INFO] [1785564014.331144588] [waypoint_mission]: Navigating to: Storage Area
[INFO] [1785564014.332697153] [waypoint_mission]: Published waypoint markers (active goal: storage)
[INFO] [1785564132.713734052] [waypoint_mission]: Reached: Storage Area
[INFO] [1785564132.714343757] [waypoint_mission]: Navigating to: Shipping Station
[INFO] [1785564132.715740727] [waypoint_mission]: Published waypoint markers (active goal: shipping)
[INFO] [1785564337.404270177] [waypoint_mission]: Reached: Shipping Station
[INFO] [1785564337.405169457] [waypoint_mission]: Navigating to: Charging Station (Home)
[INFO] [1785564337.406706574] [waypoint_mission]: Published waypoint markers (active goal: home)
[INFO] [1785564399.583684045] [waypoint_mission]: Reached: Charging Station (Home)
[INFO] [1785564399.584132621] [waypoint_mission]: Mission complete. Robot is home.
[INFO] [1785564399.585531961] [waypoint_mission]: Published waypoint markers (active goal: None)
```

Timing check against the required mission: Loading Station was reached at `1785563984.33`, and the wait
ended at `1785564014.33` — exactly **30.0 seconds** later, confirming the dwell requirement was met
precisely.

**Nav2 stack output for a single goal (e.g. sent while the full Nav2 stack is launched), showing the
BT navigator handing off to the controller server and successfully reaching the goal:**

```
[bt_navigator-6] [INFO] [1785562234.878260488] [bt_navigator]: Begin navigating from current location (-0.05, -0.02) to (-2.75, -2.81)
[controller_server-4] [INFO] [1785562234.911849014] [controller_server]: Received a goal, begin computing control effort.
[controller_server-4] [WARN] [1785562234.914662485] [controller_server]: No goal checker was specified in parameter 'current_goal_checker'. Server will use only plugin loaded goal_checker . This warning will appear once.
[controller_server-4] [WARN] [1785562234.915175850] [controller_server]: No progress checker was specified in parameter 'current_progress_checker'. Server will use only plugin loaded progress_checker . This warning will appear once.
[controller_server-4] [INFO] [1785562236.016767240] [controller_server]: Passing new path to controller.
[controller_server-4] [INFO] [1785562237.016783511] [controller_server]: Passing new path to controller.
....
[controller_server-4] [INFO] [1785562268.916841593] [controller_server]: Passing new path to controller.
[controller_server-4] [INFO] [1785562269.916780461] [controller_server]: Passing new path to controller.
[controller_server-4] [INFO] [1785562271.016774739] [controller_server]: Passing new path to controller.
[controller_server-4] [INFO] [1785562271.030400274] [controller_server]: Reached the goal!
[bt_navigator-6] [INFO] [1785562271.058557668] [bt_navigator]: Goal succeeded
```

---

## 13. Problems Encountered and Their Solutions

| Problem | Root Cause | Solution |
|---|---|---|
| Robot spawned but never moved and had no wheel joint states | Spawned from `turtlebot3_description`'s plain URDF, which has no `<gazebo>`/`<plugin>` tags at all — no DiffDrive, no JointStatePublisher | Switched the spawn node to use `-file` pointing at `turtlebot3_gazebo`'s `model.sdf`, which includes the required plugins |
| `wheel_left_link`/`wheel_right_link` missing from the TF tree; RViz RobotModel showed "No transform from wheel_left_link to base_link" | The bridge's `/joint_states` `gz_topic_name` pointed at the default scoped topic (`/world/.../model/.../joint_state`), but the plugin's explicit `<topic>joint_states</topic>` override meant it actually published on a different, unscoped topic with zero publishers on the old path | Verified the real topic with `gz topic -l`, then corrected `gz_topic_name` in the bridge config to match exactly |
| Robot's wheels spun in place briefly, then suddenly lurched into motion at mission start | Robot spawned with a small gap above the floor, so wheels spun with zero traction before gravity brought it into ground contact | Spawned with a small deliberate `-z` offset above the floor to allow a controlled settle, rather than exact `z=0.0` |
| Revisiting a previously-mapped area showed walls "shifted" from their earlier scan position | `loop_search_maximum_distance` (default 3.0 m) was smaller than the raw odometry drift accumulated over a long run, so SLAM Toolbox couldn't find loop-closure matches to correct it | Increased `loop_search_maximum_distance` to 8.0; verified `map → odom` correction magnitude dropped significantly on repeat test |
| Parts of the generated map stayed incomplete, with radiating "starburst" gaps in large open aisles | Default LiDAR max range (3.5 m) was too short to reach walls across wide warehouse aisles | Increased the LiDAR's `<range><max>` in `model.sdf`, and matched `max_laser_range` in both `slam_toolbox` and `amcl.yaml` to 12.0 |
| Nav2 planner reported "no path found" through some warehouse aisles | `inflation_radius` (0.5) was inflating obstacles enough to fully close otherwise-navigable gaps | Reduced `inflation_radius` to 0.2 in `planner_server.yaml`, and enlarged the local costmap footprint to 6×6 in `controller_server.yaml` for more planning headroom |


---

## 14. Screenshots

All screenshots referenced below are stored in the `images/` folder at the repository root.
 
### Simulation Setup
 
![TurtleBot3 Burger spawned in the warehouse world](images/turtlebot3_burger_warehouse_world.png)
*TurtleBot3 Burger spawned and running inside `warehouse_world`.*
 
### SLAM Toolbox Mapping
 
![SLAM Toolbox actively building the map](images/SLAM_in_action.png)
*SLAM Toolbox actively building the warehouse map while teleoperating the robot.*
 
![Scan and odometry topics feeding SLAM](images/SLAM_scan_odom_topics.png)
*`/scan` and `/odom` topics confirmed as active inputs to SLAM Toolbox.*
 
![Saved map inspected in a PGM viewer](images/PGM_viewer.png)
*The saved `warehouse_world_map.pgm` inspected visually with a PGM viewer.*
 
![Saved map reloaded and displayed in RViz](images/load_map_RViz.png)
*The saved map reloaded via `map_server` and displayed correctly in RViz (Transient Local QoS).*
 
### AMCL Localization
 
![Setting the robot's initial pose in RViz](images/AMCL_display_initial_pose.png)
*Setting the robot's correct initial pose in RViz using 2D Pose Estimate.*
 
![Initial AMCL pose estimate](images/AMCL_pose_initial.png)
*Initial `/amcl_pose` reading immediately after setting the initial pose.*
 
![AMCL pose updating as the robot moves](images/AMCL_pose_update.png)
*`/amcl_pose` updating as the robot moves, confirming localization is live.*
 
![LiDAR scan aligned with map walls](images/AMCL_tree_laser_align.png)
*LiDAR scan cleanly aligned with the map's walls/obstacles.*
 
![Particle cloud before convergence](images/AMCL_particle_cloud_0.png)
*Particle cloud immediately after initial pose estimate — still spread out.*
 
![Particle cloud after convergence](images/AMCL_particle_cloud_1.png)
*Particle cloud tightly converged around the robot after driving around.*
 
![TF tree showing map to odom to base_footprint](images/TF_tree.png)
*TF tree confirming the `map → odom → base_footprint` chain is complete and available.*
 
### Nav2 Navigation
 
![Nav2 lifecycle nodes status check](images/NAV2_servers_status_check.png)
*All Nav2 lifecycle nodes (`map_server`, `amcl`, `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`) confirmed active.*
 
![RViz displays required for Nav2](images/NAV2_RViz_req.png)
*Required RViz displays configured: map, costmaps, TF, particle cloud, and paths.*
 
![Sending a manual 2D Goal Pose](images/NAV2_2D_goal.png)
*A manual 2D Goal Pose sent in RViz to test navigation before automation.*
 
![Robot's planned path to the goal](images/NAV2_robot_path_plan.png)
*Global plan computed and displayed from the robot's current pose to the goal.*
 
![Goal reached confirmation](images/NAV2_goal_achieved.png)
*Nav2 confirming the goal was reached successfully.*
 
![Confirming cmd_vel message type](images/NAV2_confirm_cmd_vel_msg_type.png)
*Confirming `/cmd_vel` uses `geometry_msgs/msg/Twist` as required.*
 
### Autonomous Waypoint Mission
 
![Waypoint markers topic in RViz](images/waypoint_marker_topic.png)
*The `/waypoint_markers` MarkerArray topic added and displaying all four named waypoints in RViz.*
 
![Navigating to the Loading Station](images/waypoint_loading.png)
*Active navigation goal (green marker) set to the Loading Station.*
 
![Waiting 30 seconds at the Loading Station](images/waypoint_loading_30s.png)
*Robot holding position at the Loading Station during the required 30-second wait.*
 
![Navigating to the Storage Area](images/waypoint_storage.png)
*Active navigation goal (green marker) set to the Storage Area.*
 
![Storage Area goal achieved](images/waypoint_storage_goal_achieved.png)
*Confirmation that the Storage Area goal was successfully reached.*
 
![Navigating to the Shipping Station](images/waypoint_shipping_station.png)
*Active navigation goal (green marker) set to the Shipping Station.*
 
![Returning to the Charging Station](images/waypoint_home_return.png)
*Active navigation goal (green marker) set back to the Charging Station (Home) for the final leg.*
 
![Home return goal achieved — mission complete](images/waypoint_home_return_goal_succeeded.png)
*Confirmation that the robot successfully returned to the Charging Station, completing the full mission.*
 
---

---

## 16. Demonstration Video

<!-- Link to the full narrated video here -->
[Full narrated demonstration video](#)
