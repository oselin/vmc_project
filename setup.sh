# Copy package in Catkin workspace
cp -r ../VMC-project ~/catkin_ws/src/

# Copy environment in Gazebo
cp -r models ~/.gazebo

cd ~/catkin_ws && catkin_make

source ~/.bashrc
