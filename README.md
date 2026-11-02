[README.md](https://github.com/user-attachments/files/34275310/README.md)
# pick_place_demo: Automated Pick-and-Place with MoveIt 2

A ROS 2 (Humble) C++ node that makes a 7-DoF Franka Panda arm repeatedly pick up a box and place it at a
second station, then bring it back on the next cycle. Motion planning uses MoveIt 2; the robot runs on
`ros2_control` mock hardware and is visualised in RViz. Every stage is checked, and the run ends with a
success rate and mean cycle time.

## Functional requirements

| ID | Requirement | How it is met |
|---|---|---|
| FR-1 | Pick a 4 cm cube from station A (0.5, -0.2) and place it at station B (0.5, 0.2), and back | `PickPlace::cycle()`, direction alternates each cycle |
| FR-2 | Approach, lift and retreat must be straight vertical moves, so the gripper does not sweep into the object | `computeCartesianPath` with 5 mm steps; a stage fails if under 99% of the path is achievable |
| FR-3 | Free-space moves between stations must be collision-free | OMPL planning through `move_group`, object present in the planning scene |
| FR-4 | While held, the object must move with the gripper and be included in collision checking | `attachObject` to `panda_hand`, with the fingers and hand as allowed touch links |
| FR-5 | A failed stage must not leave the system stuck | `recover()`: release, return to `ready`, respawn the object where the next cycle expects it |
| FR-6 | Each run must report per-stage status, per-cycle duration, success rate and mean cycle time | Stage logging plus summary at the end of `main()` |
| FR-7 | The number of cycles must be configurable without recompiling | `cycles` launch argument / ROS parameter |

## Cycle

```
open gripper -> move above object -> descend (Cartesian) -> attach + close gripper
-> lift (Cartesian) -> move above target -> descend (Cartesian) -> open + detach -> retreat (Cartesian)
```

## Results

| Run | Cycles | Success | Mean cycle time | Range |
|---|---|---|---|---|
| Mock hardware, RViz, 0.5 velocity/acceleration scaling | 4 | 4/4 (100%) | 9.53 s | 7.76 to 11.23 s |

Cycle time varies because OMPL is a sampling-based planner, so each free-space path differs.

## Run it

Tested on Ubuntu 22.04 with ROS 2 Humble (inside distrobox on Fedora).

```bash
sudo apt install ros-humble-desktop ros-humble-moveit ros-humble-moveit-resources-panda-moveit-config \
                 ros-humble-ros2-control ros-humble-ros2-controllers
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone <this repo> pick_place_demo
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# terminal 1: robot, controllers, move_group, RViz
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
# terminal 2
ros2 launch pick_place_demo pick_place.launch.py cycles:=10
```

`demo.launch.py` does not publish the SRDF on a topic, so `pick_place.launch.py` passes the URDF, SRDF and
kinematics configuration to the node as parameters.

## Layout

```
src/pick_place.cpp          node: scene setup, cycle stages, recovery, metrics
launch/pick_place.launch.py loads the Panda MoveIt config into the node
```

## Next steps

- Run in Gazebo with physics, using a grasp plugin instead of attaching the object kinematically
- Detect the object position from a simulated depth camera instead of hard-coding it
- Port to MoveIt Task Constructor for multi-stage grasp planning
