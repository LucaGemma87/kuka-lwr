KUKA LWR
========

ROS/hydro metapackage that contains ROS related code to emulate the controllers included in the KUKA LWR 4+

Overview
--------


Build
-----

* 

Launch the simulation environment in terminal 1:

`roslaunch lwr_gazebo lwr_on_box.launch`

Launch the controllers in terminal 2:

`roslaunch lwr_ros_control `

You have to be sure that in lwr_control.launch is active the controller that you are looking for

Send commands to any joint in terminal 3 (you need to load/start the position controllers in the lwr_control.launch file):

`rostopic pub -1  /JointImpedanceControl/command_configuration std_msgs/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`
