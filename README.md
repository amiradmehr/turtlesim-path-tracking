# Project 1 - Task 2

## Description

In this project the turtlesim is subjected to follow a M like path. The path is extracted from the image svg and the xml file and is stored in `edges.csv` file. The path is then followed by the turtlebot using the `turtlesim` package and my package named `prj1`. The python codes are in `script` folder.

## code explanation

The code is divided into 3 files:
1. `track_path.py` is the main file that is used to initialize the node and follow the path.
2. `turtle_controller.py` is the file that contains the class `TurtleController` which has the methods to control the turtlebot like `go_to_goal`, `spawn_turtle`. It also contains the Class `PID` which is used to control the **linear and angular velocity** of the turtlebot using the PID controller.
3. `extract_path.py` is the file that is used to extract the path from the `edges.csv` file and output the path in the form of a numpy array of points.

## How to run the code

1. copy the package to `catkin_ws/src` folder
```bash
cp -r prj1 ~/catkin_ws/src
```

2. Run the following commands in the terminal
```bash
cd ~/catkin_ws
catkin_make
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscd prj1
```

3. Run the following command to start the turtlesim
```bash
roscore
rosrun turtlesim turtlesim_node
```

4. Run the following command to start the trajectory following
```bash
roscd prj1
rosrun prj1 track_path.py
```
