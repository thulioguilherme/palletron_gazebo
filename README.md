# The Palletron Gazebo repository

This repository provides the ROS package to simulate the Palletron robot on Gazebo.

## Getting started

* On your workspace, clone the [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) from the `ros2` branch.
  ```
  cd ~/<your_workspace>/src
  git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
  git checkout ros2
  ```
* Clone this repository.
  ```
  cd ~/<your_workspace>/src
  git clone https://github.com/thulioguilherme/palletron_gazebo.git
  ```
* Install dependencies.
  ```
  cd ~/<your_workspace>
  rosdep install --from-paths src --ignore-src -r -y
  ```
* Build it!
  ```
  colcon build
  ```
* Source the `setup.bash` from the `install` folder and launch a simulation.
  ```
  source install/setup.bash
  ros2 launch palletron_gazebo palletron_simulation.launch.py
  ```