#!/bin/sh

cp -r /home/user/simulation_ws/src/barista/barista_description /usr/share/gazebo/models/
cp -r /home/user/simulation_ws/src/barista/kobuki_description /usr/share/gazebo/models/
cp -r /home/user/simulation_ws/src/barista/turtlebot_description /usr/share/gazebo/models/

cp -r /home/user/simulation_ws/src/spawn_robot_tools/spawn_robot_tools_pkg/models/* /usr/share/gazebo/models/
cp -r /home/user/simulation_ws/src/barista_systems/barista_worlds/models/simple10x10 /usr/share/gazebo/models/