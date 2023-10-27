# The Palletron Gazebo repository

This repository provides the ROS package to simulate the Palletron robot on Gazebo.

## Getting started

* On your workspace, clone this repository.
  ```bash
  cd ~/<your_workspace>/src
  git clone https://github.com/thulioguilherme/palletron_gazebo.git
  ```
* Install dependencies.
  ```bash
  cd ~/<your_workspace>
  rosdep install --from-paths src --ignore-src -r -y
  ```
* Build it!
  ```bash
  colcon build
  ```
* Source the `setup.bash` from the `install` folder and spawn the robot.
  ```bash
  source install/setup.bash
  ros2 launch palletron_gazebo spawn_robot.launch.py
  ```