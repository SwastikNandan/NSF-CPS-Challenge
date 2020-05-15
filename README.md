# Scripts to control drone in PX4 Gazebo environment

Run `2020CPSChallenge.py` to code to attach and detach an object with a PX4 drone.
This script will also set mode to Offboard and arm the vehicle.

This script publishes to attach and detach commands, which require the following repository code to be setup and running.
https://github.com/Open-UAV/gazebo_ros_link_attacher 



Run `testOffboardPositionControlUnitDiamond.py` to move the drone in a diamond pattern.
This script will need you manually set the mode to offboard and arm the vehicle.
Commands to set mode and arm the vehicle.
```
rosservice call /mavros/set_mode "base_mode: 0 custom_mode: 'OFFBOARD'"
rosservice call /mavros/cmd/arming "value: true"
```
 	
  

Authors:

Jnaneshwar Das

Harish Anand
