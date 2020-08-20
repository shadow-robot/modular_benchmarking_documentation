# Integrate a robot from a launch file
This page contains all information required to integrate a robot directly from an existing launch file.

## Must have list
To follow this tutorial, make sure you have the following components:
- ROS package containing the launch file
- URDF file of the robot

## Before starting the framework
1. Open a new terminal (ctrl+shift+t), and clone the ROS package containing the launch file in `/home/user/projects/shadow_robot/base/src`
2. Run `cd projects/shadow_robot/base/; catkin_make; source devel/setup.bash`

## Tutorial
1. Start the framework: `roslaunch modular_framework_api start_framework.launch`
2. Specify the URDF file of the robot to the framework
3. Set the composition of your robots (how many arms, hands, sensors)
4. Specify the custom launch file to the corresponding field in the GUI. You should now see one editor allowing you to provide further options regarding the launch file. The expected format of each option is `<arg name="<arg_name>" value="<value>"/>`.
5. If the launch file already runs MoveIt!, you don't need to set the MoveIt! configuration package. However you can still set the MoveIt! planners in the arm/hand tab.
6. If the launch file contains external components not run by MoveIt! (such as controllers. kinematics or motion planner) you need to [specify them](???) to the framework.
At this point you should see the `Launch robot` button being enabled. If you don't have anything else to configure (sensor, joint states, etc.) you can hit it and one window should appear.
You should be able to see the robot that you want to control in a RViz window.
