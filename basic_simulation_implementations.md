
[Observe ROS topic output of apriltag_ros node in new terminal 3 (relative pose in camera frame coordinates with respect to detected apriltag marker)]:
rostopic echo /minihawk_SIM/MH_usb_camera_link_optical/tag_detections

### Try switching between some Quadplane modes and control the aircraft motion ###

# https://ardupilot.org/plane/docs/qloiter-mode.html#qloiter-mode
[Publish ROS topic in new terminal 3 (this is required to virtually center the rc sticks, 4 first channels are 0:roll(-left,+right), 1:pitch(-up,+down), 2:throttle(-down,+up), 3:yaw(-left,+right))]:
rostopic pub -r 10 /minihawk_SIM/mavros/rc/override  mavros_msgs/OverrideRCIn "channels: [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"
[Invoke ROS service in terminal 2]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'QLOITER'"

# https://ardupilot.org/plane/docs/qhover-mode.html
[Publish ROS topic in new terminal 3 (this is required to virtually trim the rc sticks)]:
rostopic pub -r 10 /minihawk_SIM/mavros/override  mavros_msgs/OverrideRCIn "channels: [1464, 1598, 1466, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"
[Invoke ROS service in terminal 2]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'QHOVER'"

# https://ardupilot.org/plane/docs/qland-mode.html
[Invoke ROS service in terminal 2]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'QLAND'"
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

### MAVROS-based commanding ###

[Launch ROS node in new terminal]:
ROS_NAMESPACE="minihawk_SIM" roslaunch robowork_minihawk_launch vehicle1_apm_SIM.launch

[Invoke flight commands in new terminal]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'AUTO'"
rosservice call /minihawk_SIM/mavros/cmd/arming True   ###Required if the mission hasn't started yet### 

[Observe ROS topic output of apriltag_ros node in new terminal]:
rostopic echo /minihawk_SIM/MH_usb_camera_link_optical/tag_detections



