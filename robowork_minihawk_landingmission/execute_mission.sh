#!/bin/bash


get_new_terminal(){ 
local command=$1  
gnome-terminal --tab -- bash -c "$command; exec bash"
sleep 10
}

echo "launching flight simulation"

get_new_terminal "cd $HOME/aerial_robotics_ws && source devel/setup.bash && roslaunch robowork_minihawk_gazebo minihawk_playpen.launch"


get_new_terminal "cd $HOME/aerial_robotics_ws/ardupilot && ./Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-minihawk --model gazebo-quadplane-tilttri --console  # --map"


get_new_terminal "rviz -d $HOME/aerial_robotics_ws/src/aerial_robotics/robowork_minihawk_launch/config/minihawk_SIM.rviz"


get_new_terminal "ROS_NAMESPACE="minihawk_SIM" roslaunch robowork_minihawk_launch vehicle1_apm_SIM.launch"

get_new_terminal "rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'AUTO'" "

get_new_terminal "rosservice call /minihawk_SIM/mavros/cmd/arming True"


get_new_terminal "rostopic echo /minihawk_SIM/MH_usb_camera_link_optical/tag_detections"


echo "launch complete"

