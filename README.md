# wormhole_map_switcher

# 🗺️ Multi-Map Navigation System (ROS 2 + TurtleBot3)

## 📋 Overview

This package enables a TurtleBot3 robot to:
- Save named poses in different maps.
- Define **"wormholes"**—navigation links between maps.
- Automatically transition between maps and continue navigating to goals.
- Execute navigation goals based on map and pose names via a custom action server.

---

## ✅ Prerequisites

### Software Requirements
- ROS 2 Humble or later
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_navigation2`, `turtlebot3_gazebo`)
- Gazebo simulator
- SQLite3
- Custom message/service/action definitions

### TurtleBot3 Setup
1. TurtleBot3 working in simulation (Gazebo).
2. Nav2 stack launched and configured.
3. At least **two maps** created and saved as `.yaml` + `.pgm`.
4. One map must be currently **loaded with the Nav2 stack running**.

---

## 📁 Package Components

### 1. **Pose Saver**
- **Node:** `SavePoseNode`
- **Service:** `/save_pose`
- **Purpose:** Save the current AMCL pose under a `point_name` for a given map.
- **DB:** `wormholes.db` via `PoseSaverDB`

### 2. **Wormhole Saver**
- **Node:** `WormholeSaver`
- **Service:** `/save_wormhole`
- **Purpose:** Save an inter-map transition point (wormhole).
- **DB:** `wormholes.db` via `WormholeDB`

### 3. **MultiMap Action Server**
- **Node:** `MultiMapActionServer`
- **Action:** `/navigate_to_named_pose`
- **Purpose:**
  - Accept goal as (map_name, pose_name)
  - Navigate via wormhole if map change needed
  - Load new map and re-localize
  - Navigate to target pose

---

## 🔧 Usage Instructions

### 1. Launch Simulation

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/path/to/room1.yaml
```
### 2. Save Poses
Run pose saver node:
```
ros2 run multi_map_nav save_pose_node
```
Save pose using service:

ros2 service call /save_pose custom_msg/srv/SavePose "{map_name: 'room1', point_name: 'dining_table'}"

### 3. Save Wormholes
Run wormhole saver node:
```
ros2 run multi_map_nav wormhole_saver
```
Save a wormhole from current map to another:
```
ros2 service call /save_wormhole custom_msg/srv/SaveWormhole "{current_map: 'room1', target_map: 'room2'}"
```
Ensure the robot is at the wormhole position.

### 4. Launch Multi-Map Action Server
```
ros2 run multi_map_nav multi_map_action_server --ros-args -p current_map:=room1
```
### 5. Send Navigation Goal
```
ros2 action send_goal /navigate_to_named_pose custom_msg/action/NavigateToNamedPose \
"{map_name: 'room2', pose_name: 'charging_station'}"
```
## 🧠 Internals
### 📌 Map Transition Logic

If target_map != current_map:

    Navigate to wormhole pose from current → target.

    Load the new map using /map_server/load_map.

    Re-localize using reverse wormhole from target → current.

    Spin in place to help AMCL converge.

    Navigate to goal pose.

### 📌 Orientation (Yaw) Extraction
```
theta = atan2(
  2.0 * (w * z + x * y),
  1.0 - 2.0 * (y * y + z * z)
)
```
### 🗄️ Database

SQLite3 DB used: wormholes.db
Tables:

    saved_poses(map_name, point_name, x, y, theta)

    wormholes(from_map, to_map, x, y, theta)

### Inspect DB:

sqlite3 wormholes.db

### 🔁 Workflow Example

    Launch sim in room1.

    Save named pose(s).

    Move robot to wormhole location.

    Save wormhole: room1 → room2.

    Switch sim to room2, save reverse wormhole.

    Save target pose in room2.

    Launch action server.

    Send goal to any named pose in any map.

### 🧪 Testing Tips

    Wait for /amcl_pose before saving poses.

    Manually test /map_server/load_map before using auto transition.

    Use RViz to verify pose and navigation.