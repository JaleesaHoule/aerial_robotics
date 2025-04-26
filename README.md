
```
Jaleesa Houle
CS 691: Aerial Robotics 
Spring 2025
Project goal: write a ROS Node to land minihawk via Gazebo simulation

```


## Project Implementation 

### goal: land the drone on building using given error measurements between drone and Apriltag

#### Manual implementation for drone flight:
```
[Launch ROS Gazebo SIM in new terminal]:
cd $HOME/aerial_robotics_ws && source devel/setup.bash
roslaunch robowork_minihawk_gazebo minihawk_playpen.launch

[Launch Ardupilot Gazebo SITL in new terminal]:
cd $HOME/aerial_robotics_ws/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-minihawk --model gazebo-quadplane-tilttri --console  # --map

[Launch Rviz in new terminal]:
rviz -d $HOME/aerial_robotics_ws/src/aerial_robotics/robowork_minihawk_launch/config/minihawk_SIM.rviz

### MAVProxy-based commanding ###

### Note: this step is not necessary if completed previously:
[Load sample mission waypoints in Gazebo SITL terminal]
wp load ../src/aerial_robotics/robowork_minihawk_gazebo/resources/waypoints.txt

```
```
[Launch ROS node in new terminal]:
ROS_NAMESPACE="minihawk_SIM" roslaunch robowork_minihawk_launch vehicle1_apm_SIM.launch

[Invoke flight commands in new terminal]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'AUTO'"
rosservice call /minihawk_SIM/mavros/cmd/arming True   ###Required if the mission hasn't started yet### 

[Observe ROS topic output of apriltag_ros node in new terminal]:
rostopic echo /minihawk_SIM/MH_usb_camera_link_optical/tag_detections
```

## Automated implementation of the above steps: 

```
cd $HOME/aerial_robotics_ws/robowork_minihawk_landingmission && chmod +x execute_mission.sh

./execute_mission.sh

```

### Initiate custom Ros node for control and landing protocol ###

```
rosrun robowork_minihawk_landingmission mission.py

```

Note: make sure to run ``` source devel/setup.bash``` & ``` catkin make ``` when first downloading this repo
