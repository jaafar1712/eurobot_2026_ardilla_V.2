# Eurobot 2026 — Mechanuts V.2

Autonomous ROS2 robot stack for the [Eurobot 2026](https://www.eurobot.org/) competition, developed by the MAM (Master's in Advanced Mechatronics) team. The robot autonomously detects, collects, and delivers colored crates to pantry drop-off zones within a 100-second match, using SLAM, Nav2 navigation, ArUco-based perception, and a contact-aware gripper.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Packages](#packages)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building](#building)
- [Running the System](#running-the-system)
- [Configuration](#configuration)
- [Branching Strategy](#branching-strategy)
- [Team](#team)

---

## Overview

The robot operates as a fully autonomous agent during Eurobot matches:

1. **Searches** for colored crates using an RGB camera and ArUco markers
2. **Navigates** toward the nearest crate using direct velocity control
3. **Aligns** the open gripper around the crate with ±4 cm precision
4. **Grips** the crate using contact-sensor feedback
5. **Delivers** the crate to a detected pantry drop-off point via Nav2
6. **Returns** to the starting nest when time is running low

The full mission runs on a 9-state finite state machine inside `task_manager.py`.

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  Task Manager                   │
│  INITIALIZING → SEARCHING → NAVIGATING_TO_CRATE │
│  → ALIGNING → GRIPPING → NAVIGATING_TO_DROPOFF  │
│  → RELEASING → RETURNING_TO_NEST → FINISHED     │
└────────┬──────────────┬──────────────┬──────────┘
         │              │              │
   ┌─────▼──────┐ ┌─────▼──────┐ ┌───▼────────────┐
   │ Perception │ │ Navigation │ │  Manipulation  │
   │ ArUco +    │ │ SLAM+Nav2  │ │ Gripper State  │
   │ Color HSV  │ │ DWA Planner│ │ Machine        │
   └────────────┘ └────────────┘ └────────────────┘
```

**ROS2 Topics (key):**

| Topic | Type | Description |
|---|---|---|
| `/crate/detections` | `CrateDetectionArray` | Detected crates with distance & angle |
| `/pantry/detections` | `PantryDetectionArray` | Detected drop-off zones |
| `/gripper/command` | `String` | `open` / `close` commands |
| `/gripper/state` | `String` | Current gripper state |
| `/cmd_vel` | `Twist` | Robot velocity commands |
| `/task_manager/state` | `String` | Current mission state |
| `/crate/debug` | `Image` | ArUco debug visualization |

---

## Packages

### `eurobot_perception`
Vision-based crate and pantry detection.

- **ArUco detection** (`aruco_crate_perception.py`) — production-quality node with temporal filtering, outlier rejection, and adaptive confidence scoring. Uses `DICT_4X4_50` with marker IDs: `36` (blue), `47` (yellow), `41` (empty).
- **Color detection** (`color_identifier.py`) — HSV range-based fallback for blue/yellow crates.
- **Pantry detection** (`pantry_detection.py`) — locates pantry drop-off zones from camera data.

### `eurobot_navigation`
SLAM, autonomous path planning, and mission orchestration.

- **SLAM** (`slam_node.py`) — online async SLAM via `slam_toolbox`.
- **Nav2 integration** (`nav2_global_planner.py`, `nav2_local_planner.py`) — global path planning with DWA local planner.
- **Task Manager** (`task_manager.py`) — 9-state FSM with match timer and nest return countdown.
- **Scan Frame Fixer** (`scan_frame_fixer.py`) — converts LIDAR scan frames for SLAM compatibility.
- **Teleop** (`teleop_keyboard.py`) — keyboard teleoperation for manual control and testing.

### `eurobot_manipulation`
Gripper control with contact feedback.

- **Gripper Controller** (`gripper_controller.py`) — 20 Hz state machine with states: `initializing → idle → opening → closing → gripping`. Reads contact sensors on both fingers and stops closing when contact is detected.

### `eurobot_interfaces`
Custom ROS2 message definitions.

- `CrateDetection.msg` — single crate: color, distance, angle, confidence
- `CrateDetectionArray.msg` — array of detections
- `PantryDetection.msg` / `PantryDetectionArray.msg` — pantry drop-off data

### `mam_eurobot_2026`
Main launch files, URDF robot model, Gazebo world, and RViz config.

- `full_system.launch.py` — launches everything with staggered timing
- `arena.launch.py` — launches Gazebo arena only
- `simple_robot.urdf` — robot model with differential drive, camera, LIDAR, and parallel gripper
- `arena_world.sdf` — Gazebo Ignition arena with crates and pantry shelves

---

## Prerequisites

- **OS:** Ubuntu 22.04 (recommended) / Windows with WSL2
- **ROS2:** Humble Hawksbill
- **Simulator:** Gazebo Ignition (Fortress)
- **Python:** 3.10+

ROS2 dependencies:
```
nav2_bringup
slam_toolbox
ros_gz_bridge
ros_gz_sim
robot_state_publisher
controller_manager
ros2_control
image_view
rviz2
cv_bridge
opencv-python
```

---

## Installation

```bash
# Clone the repository
git clone https://github.com/jaafar1712/eurobot_2026_ardilla_V.2.git
cd eurobot_2026_ardilla_V.2

# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Source your ROS2 installation
source /opt/ros/humble/setup.bash
```

---

## Building

```bash
colcon build --symlink-install
source install/setup.bash
```

To build a single package:
```bash
colcon build --packages-select eurobot_navigation
```

---

## Running the System

### Full System (Simulation + Navigation + Perception)

```bash
ros2 launch mam_eurobot_2026 full_system.launch.py
```

**Startup sequence (automatic):**

| Time | Component |
|------|-----------|
| t=0s | Gazebo, Robot State Publisher, RViz, ROS-IGN bridges |
| t=5s | Joint state broadcaster, gripper controller, scan frame fixer, ArUco perception |
| t=6s | ArUco debug image view |
| t=7s | SLAM (slam_toolbox) |
| t=10s | Nav2 (autonomous navigation) |
| t=13s | Teleop keyboard (xterm window) |

### Individual Components

```bash
# Perception only
ros2 launch eurobot_perception aruco_crate_perception.launch.py

# SLAM only
ros2 launch eurobot_navigation online_async_launch.py

# Navigation only
ros2 launch eurobot_navigation navigation.launch.py

# Arena (Gazebo simulation)
ros2 launch mam_eurobot_2026 arena.launch.py
```

### Manual Teleoperation

```bash
ros2 run eurobot_navigation teleop_keyboard
```

### Gripper Manual Control

```bash
# Open gripper
ros2 topic pub /gripper/command std_msgs/String "data: 'open'" --once

# Close gripper
ros2 topic pub /gripper/command std_msgs/String "data: 'close'" --once
```

---

## Configuration

### Task Manager Parameters

Located in `eurobot_navigation/config/`:

| Parameter | Default | Description |
|---|---|---|
| `team_color` | `yellow` | Target crate color (`yellow` or `blue`) |
| `match_duration` | `100.0` | Match length in seconds |
| `nest_return_time` | `10.0` | Seconds before end to trigger nest return |
| `max_crates_to_collect` | `6` | Maximum crates per match |
| `alignment_distance` | `0.50` | Approach distance for gripper alignment (m) |
| `alignment_tolerance` | `0.04` | Fine alignment tolerance ±4 cm |
| `angle_tolerance` | `8.0` | Angular alignment tolerance (degrees) |

### Nav2 Parameters

`eurobot_navigation/config/nav2_params.yaml` — DWA local planner, costmap inflation, and recovery behaviors.

### ArUco Marker IDs

| Marker ID | Crate Color |
|---|---|
| 36 | Blue |
| 47 | Yellow |
| 41 | Empty |

---

## Branching Strategy

```
main          ← stable, tested, protected
└── dev       ← integration branch (default)
    ├── feature/your-feature
    └── fix/your-fix
```

- All work is done on `feature/` or `fix/` branches off `dev`
- Open a Pull Request to merge into `dev`
- `dev` is merged into `main` after testing on the physical robot
- Direct pushes to `main` and `dev` are protected — PRs required

---

## Team

**MAM — Master's in Advanced Mechatronics | Eurobot 2026**

> Built with ROS2 Humble · Gazebo Ignition · Nav2 · slam_toolbox · OpenCV
