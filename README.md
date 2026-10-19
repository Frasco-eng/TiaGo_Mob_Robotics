# TIAGo Autonomous Exploration and Manipulation

Project for the **Autonomous and Mobile Robotics** course, University of Bologna, A.Y. 2025/2026.

This repository is a ROS 2 Humble workspace that gives a simulated PAL **TIAGo** robot three autonomous skills, run in sequence:

1. **Exploration**: build a 2D occupancy grid map of an unknown environment with SLAM.
2. **Navigation**: localize inside the known map, then find and record the poses of the ArUco-marked pick and place stations.
3. **Manipulation**: pick the ArUco cubes from the pick station and place them on the place station.

**Presentation:** [`presentation/main.pdf`](presentation/main.pdf) (LaTeX sources in [`presentation/`](presentation/))

---

## Architecture

The system follows a **Policy vs. Actuator** design:

- **Coordinators (policy)**: one high-level node per task. Each runs the task's state machine and decides what happens next.
- **Generalized actuators**: reusable skills that know nothing about the task, such as `map_navigator`, `head_motion`, `localization` and `tuck_arm`.
- **Handshake protocol**: nodes coordinate only through defined topics, services and actions. The custom messages and services live in the `tiago_exam_interfaces` package.

### Workspace structure

The project's own code is in these packages:

| Package | Description |
|---|---|
| [`src/tiago_exam`](src/tiago_exam) | Main `ament_python` package: launch files, maps, RViz configs and all task nodes |
| [`src/tiago_exam_interfaces`](src/tiago_exam_interfaces) | Custom messages and services (`MoveHead`, `SaveMarker`, `ManipulatorCommand`, `StartNavigation`, ...) |
| [`src/tiago_exam_worlds`](src/tiago_exam_worlds) | Gazebo worlds and models (tables, ArUco markers and cubes) for the exam scenario |

```
tiago_exam/
├── launch/            # task1, task2, task3 and environment launch files
├── maps/              # saved occupancy grid maps
├── data/              # marker_locations.csv (output of Task 2, input of Task 3)
├── config/            # RViz configurations
└── tiago_exam/
    ├── map_navigator.py, head_motion.py, localization.py, tuck_arm.py, utils.py
    ├── task1/         # map_saver_watcher
    ├── task2/         # coordinator_task2, naive_explorer, aruco_tracker, aruco_noter
    └── task3/         # coordinator_task3, manipulator
```

The other packages in `src/` are third-party dependencies, some of them modified for this project: the PAL TIAGo stack, `m-explore-ros2`, `IFRA_LinkAttacher`, `pymoveit2` and others.

---

## Tasks

### Task 1: Exploration
- Autonomous **frontier-based exploration** with a modified `m-explore` (`explore_lite`) node. It prefers frontiers that are **close** to the robot and border **large unexplored areas**.
- Before moving, the arm is **tucked** so the robot's footprint stays small.
- SLAM and Nav2 parameters are tuned for narrow passages and for false LIDAR obstacles caused by shadows.
- When no frontiers are left, the robot returns **home** and `map_saver_watcher` saves the map to `maps/`.

### Task 2: Navigation and marker search
- **Global localization** with AMCL: the particles are spread over the whole map, the robot spins 360°, and it waits until the pose covariance drops below a threshold.
- **Goal sampling**: `cv2.distanceTransform` on the map produces a whitelist of safe free cells, and goals are sampled from it at random.
- **Finite state machine** in the coordinator: *Exploring → Tracking → Inspecting → Homing*. It uses a `MultiThreadedExecutor` and reentrant callback groups, so an ArUco detection can interrupt navigation right away.
- Target markers: **ID 26** (pick station) and **ID 238** (place station). Their poses are written to `data/marker_locations.csv`.

### Task 3: Pick and place
- The coordinator reads `marker_locations.csv`, navigates to each station and sends `PICK_<id>` / `PLACE_<id>` commands to the `manipulator` node (MoveIt 2).
- At the station, the head tilts down to detect the cube and the last valid marker transform is cached. The grasp is collinear and antipodal for the 7 cm cubes (IDs 63 and 582).
- The table is added to MoveIt as a **collision object**. The arm retracts before the torso lowers.
- **Relative pose replication**: the base-to-table distance recorded while grasping is reused for placing, with a final linear correction to keep the target reachable.
- The **Link Attacher** plugin holds the cube in the gripper during transport, so it does not slip because of unreliable contact physics in Gazebo.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11 and RViz2
- Nav2, SLAM Toolbox, MoveIt 2, `aruco_ros`

## Installation

```bash
git clone https://github.com/Frasco-eng/TiaGo_Mob_Robotics.git
cd TiaGo_Mob_Robotics
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage

Run the tasks in order. Each task uses what the previous one produced.

```bash
# Task 1: explore and build the map
ros2 launch tiago_exam task1.launch.py

# Task 2: localize and find the station markers
ros2 launch tiago_exam task2.launch.py

# Task 3: pick and place the cubes
ros2 launch tiago_exam task3.launch.py
```

Optional launch arguments: `world_name:=<world>` (default `group29`) and `map_name:=<map>` (default `env_map`).

---

## Authors

- **Francesco Cenni** (francesco.cenni3@studio.unibo.it)
- **Giulio Mazzaroppi** (giulio.mazzaroppi@studio.unibo.it)
- **Giacomo Pezzolati** (giacomo.pezzolati@studio.unibo.it)

University of Bologna, Autonomous and Mobile Robotics, A.Y. 2025/2026

## Copyright

© 2026 Francesco Cenni, Giulio Mazzaroppi, Giacomo Pezzolati. **All rights reserved.**

The original code in `tiago_exam`, `tiago_exam_interfaces` and `tiago_exam_worlds`, and the presentation, may not be copied, modified or redistributed without the authors' written permission.

Third-party packages in `src/` (PAL Robotics packages, `m-explore-ros2`, `IFRA_LinkAttacher`, `pymoveit2`, ...) keep their original licenses, which are included in each package's directory.
