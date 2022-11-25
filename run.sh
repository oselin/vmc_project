source ~/.bashrc

# Launch gazebo with the right world
roslaunch vmc_project project_world.launch &
# Run the program
rosrun vmc_project racer.py