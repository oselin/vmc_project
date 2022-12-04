# Vector Field Histogram algorightm implementation

Simple simulation of a turtle bot moving within a maze. Algorithm developed using ROS2 and Python3

Inspired by the paper __A Constrained VFH Algorithm for Motion Planning of Autonomous Vehicles__


## Installation of the ROS package

1 - Move to catkin workspace folder
```bash
cd ~/catkin_ws/src/
```

2 - Clone the repository
```bash
git clone git@github.com:oselin/vmc_project.git
```

3 - Copy the track file into the Gazebo folder
```bash
cp -r vmc_project/models ~/.gazebo
```

3 - Recompile CMakeLists file and source the folder
```bash
catkin_make
source ~/.bashrc
```

## Run the program

First of all, launch Gazebo with the chosen track
```bash
roslaunch vmc_project project_world.launch
```

Then, activate the VFH algorithm by running the script
```bash
rosrun vmc_project racer.py
```

If you want to see in real time how data is manipulated, run
```bash
rosrun vmc_project plotter.py
```
or
```bash
rosrun vmc_project plotter2.py
```

## Difference between Plotter and Plotter2
The designed node publishes two types of data
1 - velocity commands to the robot
2 - manipulated data, or VF histograms + occupancy map

Since the latter is a very big message (almost 5000 float32 values), to increase the perfomances the following workaround has been designed.

`plotter.py` builds the graphs basing on the complex and heavy message

`plotter2.py` reads LiDAR values directly from the `/scan ` topic and performes the same data manipulation steps as racer.py, to avoid a useless publishing on a custom topic


## Known issues in the software
Since this data manipulation is quite complex, the computer cannot keep up with both manipulating data, publishing commands and manging real time plots.

This slows down all the computations, leading to a failure of the algorithm. __Workaround__: A cool way to see real time plots is to first record the LiDARS values with
```bash
rosbag record /scan
```

And then simulating data publishing, but this time running only the plotter. Therefore
```bash
rosbag play <your-recording>
```

And on a new terminal
```bash
rosrun vmc_project plotter2.py
```