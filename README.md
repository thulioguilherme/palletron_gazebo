# Palletron Gazebo

This repository provides the ROS 2 package to simulate the Palletron robot on Gazebo.

## System requirements

This package was built and tested on:
  * Ubuntu 22.04.3 LTS (Jammy Jellyfish)
  * ROS Humble Hawksbill
  * Gazebo Classic simulator

## Installation

Follow the instructions:
* On your workspace, clone this repository.
```bash
cd ~/<your_workspace>/src
git clone https://github.com/thulioguilherme/palletron_gazebo.git
```
* Install all the dependencies using `rosdep`.
```bash
cd ~/<your_workspace>
rosdep install --from-paths src --rosdistro humble --ignore-src -r -y
```
* Build it using `colcon`.
```bash
colcon build
```

## Usage

Start any Gazebo world of your preference, only ensuring that its launcher loads the `libgazebo_ros_factory.so` plugin.

### Spawn a single robot

Use the `spawn_robot` launcher to spawn a single robot.
  ```bash
  ros2 launch palletron_gazebo spawn_robot.launch.py
  ```

  > ***Tip**: Add the `-s` option after the launcher name to see all the arguments.*

### Spawn more than one robot

We support multi-robot spawns through the `spawn_multi_robot` launcher.
  ```bash
  ros2 launch palletron_gazebo spawn_multi_robots.launch.py
  ```