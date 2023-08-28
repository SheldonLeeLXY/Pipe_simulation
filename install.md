- install ubuntu 18
- install ros melody http://wiki.ros.org/melodic/Installation/Ubuntu

- catkin_make
- sudo apt install ros-melodic-velocity-controllers

- sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers

- chmod +x ./run_cmd.sh

- chmod +x mybot_velocity_controller.py

- echo "source /home/sheldon/mybot_ws_camera/devel/setup.bash" >> ~/.bashrc
- source ~/.bashrc

- ./run_cmd.sh
- roslaunch mybot_gazebo t4_worldV.launch
- rosrun active_vision record_images.py

- conda deactivate

# remember to check relative path