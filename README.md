# Autonomous Valet Parking System

A simulated robot car that drives itself to a parking spot. It runs in a virtual parking lot, builds a map of the lot, finds its own path, and parks in a chosen slot.

The project uses ROS 2 Jazzy and Gazebo Harmonic.

## What it does

- Shows a robot car in a simulated parking lot (Gazebo).
- Builds a map of the lot using the robot's sensors (SLAM).
- Plans a path and drives the robot along it (Nav2).
- Follows a simple list of steps to complete the parking task (Behavior Tree).
- Shows the robot, map, and path in RViz.

## Project parts

All parts are in the `src` folder.

| Package | What it is for |
|---|---|
| `avps_description` | The robot model (URDF file). |
| `avps_gazebo` | The parking lot world and the Gazebo launch file. |
| `avps_slam` | Map building settings and launch file. |
| `avps_navigation` | Path planning settings and launch file. |
| `avps_controller` | Steering code that makes the robot follow the path (pure pursuit). |
| `avps_behavior` | The parking task steps. It sends the robot to the parking zone and then to the slot. |
| `avps_bringup` | Starts everything together and holds the RViz view. |

## How the parking task works

The task is written in `src/avps_behavior/behavior_trees/avps_parking.xml`.

1. Drive to the parking zone entrance.
2. Drive to the parking slot (Slot A3).

If a step fails, the task starts again from the beginning.

## What you need

- Ubuntu with ROS 2 Jazzy
- Gazebo Harmonic
- Nav2
- SLAM Toolbox
- BehaviorTree.CPP
- RViz2

## How to build

```bash
cd Autonomous_Valet_Parking_System-main
colcon build
source install/setup.bash
```

## How to run

Start the full system:

```bash
ros2 launch avps_bringup avps_full.launch.py
```

This opens Gazebo, starts the map building, and opens RViz.

You can also start each part alone:

```bash
ros2 launch avps_gazebo gazebo.launch.py
ros2 launch avps_slam slam.launch.py
ros2 launch avps_navigation navigation.launch.py
ros2 launch avps_description display.launch.py
```

## Notes

- Settings for SLAM and navigation are in the `config` folders of `avps_slam` and `avps_navigation`.
