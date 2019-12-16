# Configuring the framework

The Modular Benchmarking Framework allows a wide range of configuration at different levels: the *config files*, the *launch file* and the *task_constructor*.

## Config files

All the config files are stored in the **config** folder of the *modular_benchmarking_framework_api* package. They gather all the differents components that you might want to change in order to run your robot, without requiring any code. We will provide a detailed description of each of them.

### Hardware connection

This YAML file must contain all the *hardware_interface* that ROS should be aware of when using the framework on the physical robot. If you are not familiar with this concept, you can read more about it on this [tutorial](https://github.com/ros-controls/ros_control/wiki/hardware_interface). The first parameter in the YAML file, **robot_hardware** should contain the list of all the *combined_hardware_interface* that are going to be defined in the same file. Each of the specific hardware interface must be derived from **CombinedRobotHW**. A tutorial about why and how to create a CombinedRobotHW can be found [here](https://github.com/ros-controls/ros_control/wiki/Writing-CombinedRobotHW). </br>
The YAML file must then specificy for each hardware interface the different parameters that must be loaded on the ROS param server in order to run the hardware. The parameters are implementation-dependent. **The framework provides natively a compatible hardware interface for Universal Robot arms**. The source code can be found on [github](https://github.com/shadow-robot/sr_ur_arm/blob/kinetic-devel/sr_ur_robot_hw/src/sr_ur_robot_hw.cpp). For our implementation, the following parameters must be filled:
* ```robot_id```: Allow to load a robot which name consists of a prefix (usually given in the URDF file)
* ```robot_ip_address```: Robot's IP address
* ```control_pc_ip_address```: IP address of the machine that is controlling the robot arm
* ```speed_scale```: Robot arm speed scale [0.0 - 1.0]
* ```payload_mass_kg```: Manipulator weight (initial payload of the robot)
* ```payload_center_of_mass_m```: Estimated manipulator's payload centre of inertia
* ```topic_to_wait_for```: Name of the topic published by the hardware interface to notify that the controllers can be loaded
* ```robot_program_path```: Path to the directory of Universal Robot's program used to communicate with the arm.

Obviously, some parameters (even if renamed) must absolutely be loaded, such as the robot and pc ip addresses. In any case, please make sure to specify the ```type``` field if you are using another robot arm or if you prefer using your own implementation. An example of hardware connection config file for a UR5 only can be found [here](https://github.com/shadow-robot/modular_benchmarking_framework/modular_benchmarking_framework_api/config/connection_hardware.yaml).

### Generative methods parameters

Whatever the method used to generate grasps, joint states, trajectories or poses, they rely on some parameters provided by the user. If you are using the framework for benchmarking, you may not want to modify the same information across the different files but instead centralize them in a single YAML file loaded on the ROS param server and access them through the ROS API. This file can be filled with anything the user needs to load and access in different nodes without hardcoding it and that is susceptible to change. </br>
For instance, the one we provide contains information about the name of the origin frame and the values specifying a cropping area. You can also add, for example, the path to a neural network that needs to be evaluated.

### Manipulator controller

This YAML file specifies how to control the manipulator that you are using either in simulation or on the physical robot. Let's differentiate the case in which the manipulator you are using is *fully integrated* to ROS (meaning that it can be controlled using *ros_controllers* and is plugged in the framework via a *hardware_interface*) and in which the manipulator is controlled directly through some drivers without a hardware interface.

#### The manipulator is not fully integrated to ROS
In this case, in order to control it through the framework, the only thing you need is to write a ROS action server in a node. A tutorial about actions is available [here](http://wiki.ros.org/actionlib) and an example of action server for the EZGripper can be found [here](https://github.com/bdenoun/EZGripper/blob/master/ezgripper_driver/nodes/ezgripper_controller.py). The latter is a ROS action server designed for grasping, but you can easily develop one dedicated to dexterous manipulation. <br/>
The last step is to specify ```package_action_server``` (name of the package in which the server to run is stored), ```node_action_server_name``` (name of the node running the action server) and ```action_server_type``` (type of the node to launch). These three parameters correspond to the different flags you can find when you include a **node** in a launch file. The parameter ```action_server_name``` indicates the name given to the ROS action server, and can be used in any node.
If you are working in **simulation** and your manipulator can be controlled using Moveit, you can use the generic [Moveit-based action server](https://github.com/shadow-robot/modular_benchmarking_framework/modular_benchmarking_framework_core/nodes/???) that we provide as an example of grasp controller. <

#### The manipulator is fully integrated to ROS
TBC


Anyway we strongly advised to get a prefix for manipulator, and of course add anything to the YAML file if required.
