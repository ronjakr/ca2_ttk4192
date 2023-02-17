# ca2_ttk4192

This repository contains the ROS (Robot Operating System) package used in the second obligatory assignment in the course TTK4192 at the Norwegian University of Science and Technology (NTNU). It is based on ROS Noetic; it might work with other ROS versions, but no guarantees. 

Install the package by entering the following commands (assuming your catkin workspace folder is named catkin_ws):
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/sbremman/ca2_ttk4192
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

To check if it works correctly, you can start a Gazebo simulation of the turtlebot in a maze with the following command:
```
$ roslaunch ca2_ttk4192 complicated_maze.launch
```

To see a visualization of the turtlebot and its environment in RViz, open \underline{another terminal window} and run: 

```
  $ export TURTLEBOT3_MODEL=waffle_pi
  $ roslaunch ca2_ttk4192 collision_detector.launch map_file:=complicated
```

When you are done, close the simulation and RViz by pressing CTRL+C in their respective terminals.
