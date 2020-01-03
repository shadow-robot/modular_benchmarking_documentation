# Configuring the framework
The Modular Benchmarking Framework allows a wide range of configuration at different levels: the **config files**, the **task_constructor** and the **launch file**. So all you need to modify is located in the **API** package.<br/>
*The framework should natively be configured to run the EZGripper in simulation without any modification.*

## Config files
All the config files are stored in the **config** folder of the *modular_framework_api* package. They gather all the differents components that you might want to change in order to run your robot, without requiring any code. We will provide a detailed description of each of them.

### Hardware connection
This YAML file must contain all the *hardware_interface* that ROS should be aware of when using the framework on the physical robot. If you are not familiar with this concept, you can read more about it on this [tutorial](https://github.com/ros-controls/ros_control/wiki/hardware_interface). The first parameter in the YAML file, **robot_hardware** should contain the list of all the *robot hardware* that are going to be defined in the same file. Each of the specific hardware interface must be derived from `RobotHW`. The framework will then automatically create a combined hardware interface.</br>
The YAML file must then specificy for each hardware interface the different parameters that must be loaded on the ROS param server in order to run the hardware. The parameters are implementation-dependent. **The framework provides natively a compatible hardware interface for Universal Robot arms**. The source code can be found on [github](https://github.com/shadow-robot/sr_ur_arm/blob/kinetic-devel/sr_ur_robot_hw/src/sr_ur_robot_hw.cpp). For our implementation, the following parameters must be filled:
* ```robot_id```: Allow to load a robot which name consists of a prefix (usually given in the URDF file)
* ```robot_ip_address```: Robot's IP address
* ```control_pc_ip_address```: IP address of the machine that is controlling the robot arm
* ```speed_scale```: Robot arm speed scale [0.0 - 1.0]
* ```payload_mass_kg```: Manipulator weight (initial payload of the robot)
* ```payload_center_of_mass_m```: Estimated manipulator's payload centre of inertia
* ```topic_to_wait_for```: Name of the topic published by the hardware interface to notify that the controllers can be loaded
* ```robot_program_path```: Path to the directory of Universal Robot's program used to communicate with the arm.


Obviously, some parameters (even if renamed) must absolutely be loaded, such as the robot and pc ip addresses. In any case, please make sure to specify the ```type``` field if you are using another robot arm or if you prefer using your own implementation. An example of hardware connection config file for a UR5 only can be found [here](https://github.com/shadow-robot/modular_benchmarking_framework/blob/kinetic-devel/modular_framework_api/config/connection_hardware.yaml).

### Generative methods parameters
Whatever the method used to generate grasps, joint states, trajectories or poses, they rely on some parameters provided by the user. If you are using the framework for benchmarking, you may not want to modify the same information across the different files but instead centralize them in a single YAML file loaded on the ROS param server and access them through the ROS API. This file can be filled with anything the user needs to load and access in different nodes without hardcoding it and that is susceptible to change. </br>
For instance, the one we provide contains information about the name of the origin frame and the values specifying a cropping area. You can also add, for example, the path to a neural network that needs to be evaluated.

### Manipulator controller
This YAML file specifies how to control the manipulator that you are using either in simulation or on the physical robot. Let's differentiate the case in which the manipulator you are using is *fully integrated* to ROS (meaning that it can be controlled using *ros_controllers* and is plugged in the framework via a *hardware_interface*) and in which the manipulator is controlled directly through some drivers without a hardware interface.

#### The manipulator is not fully integrated to ROS
In this case, in order to control it through the framework, the only thing you need is to write a ROS action server in a node. A tutorial about actions is available [here](http://wiki.ros.org/actionlib) and an example of action server for the EZGripper can be found [here](https://github.com/bdenoun/EZGripper/blob/master/ezgripper_driver/nodes/ezgripper_controller.py). The latter is a ROS action server designed for grasping, but you can easily develop one dedicated to dexterous manipulation. <br/>
The last step is to specify ```package_manipulator_action_server``` (name of the package in which the server to run is stored), ```manipulator_controller_node_name``` (name of the node running the action server) and ```manipulator_controller_action_server_type``` (type of the node to launch). These three parameters correspond to the different flags you can find when you include a **node** in a launch file. The parameter ```manipulator_controller_action_server_name``` indicates the name given to the ROS action server, and can be used in any node requiring to control the manipulator.

#### The manipulator is fully integrated to ROS
In this case it means that you are free to choose between several options. The first one is to use one of the controller provided by ROS by declaring it in the *controller file* (you can find more information [here](./robot_package.md)). If you want something a bit more specific, you can create your own ROS controller derived from the class ```controller_interface::ControllerBase``` and create a plugin so you can use it by declaring it in the *controller file*. An example of home made controller and how to create a plugin can be found [here](https://github.com/shadow-robot/sr_core/tree/kinetic-devel/sr_mechanism_controllers). If you are opting for the latter option, please **make sure that** ```package_manipulator_action_server``` **is an empty string**, and you do not need to fill the two other fields since the framework will know through the *controller file* which controller to load and execute.


If you are working in **simulation** and/or your manipulator can be controlled using Moveit, you can use our generic [Moveit-based action server](https://github.com/shadow-robot/modular_benchmarking_framework/blob/kinetic-devel/modular_framework_core/nodes/moveit_grasp_action_server.cpp) that we provide as an example of grasp controller.<br/>
If your controller requires more parameters, you can of course add them to this config files (such as ```max_torque``` for instance).

### Connection to an external manipulator
This config file gathers all parameters required to connect a manipulator that is **not** fully integrated in ROS and/or is not using an ethernet connection. If the manipulator only has an USB interface and you do not want to change the port address everytime in the code, you can store all these information on the ROS param server. The [file](https://github.com/shadow-robot/modular_benchmarking_framework/modular_framework_api/config/external_manipulator_connection.yaml) natively provided gives an example of the information needed to communicate with the EZGripper.

### Motion Planner config
*If you are interested in planner benchmarking, you might be interested in [this](https://planners-benchmarking.readthedocs.io/en/latest/)*. <br/> <br/>
This config file allows to configure the motion planner that is going to be used when planning for a specific group during your task. For now, you can choose any motion planner among this [list](http://ompl.kavrakilab.org/planners.html). All of the parameters of this configuration file **must be filled**! .
* The string corresponding to ```planner_name``` must exactly match the name of the planner defined in your [moveit package config](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html). If you are using what we provide, the name should match what is declared in this [file](https://github.com/ARQ-CRISP/arq_ur5_ezgripper_moveit_config/blob/master/config/ompl_planning.yaml).
* ```robot_speed_factor``` allows to scale the speed of the robot when executing a trajectory (value between 0 and 1)
* ```arm_group_name``` specifies the group for which all the other parameters are going to be set. The group name must match the group defined in the [srdf file](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html)! An example of a srdf file with some groups defined can be found [here](https://github.com/ARQ-CRISP/arq_ur5_ezgripper_moveit_config/blob/master/config/arq_ur5_with_ezgripper.srdf).
* ```number_plan_attempt``` is the number of times the plan is computed from scratch before the shortest solution is returned.
* ```planning_max_time``` specifies the maximum amount of time before timing out when planning.


 **/!\** Please note that **for now** the planner **must** be part of the **OMPL** planner manager and is going to be used by Moveit. We are working on creating an interface so that home made motion planners can be integrated **/!\**

 ### Known joint states
 Most of the time, even when running highly automated methods, we know some *waypoints* of the robot (e.g starting joint state of the robot). For instance when cleaning a surface by picking objects, they are usually all dropped at the same pre-defined location. Instead of hardcoding them in the code, you can gather them in the config file called **named_joint_states.yaml**. They must be defined with the following rule:
 ```yaml
waypoint1_name:
  joint_name_1: <value_11>
  joint_name_2: <value_21>
  ...
  joint_name_n: <value_n1>

waypoint2_name:
  joint_name_1: <value_12>
  joint_name_2: <value_22>
  ...
  joint_name_n: <value_n2>

...
 ```
 Just make sure that the given names in this YAML file are **unique** (i.e not giving twice the same name to two joint states). You can however define joint states for **different groups** in the same file. The joint states defined here can also be used to automatically create trajectories (please see [here](#known-trajectories)).

 ### Known trajectories
In some cases, part of the task performed by the robot is known in advance and we are expecting the robot to always repeat a given trajectory. For instance, after grasping an object at an unknown pose, go to pose A through pose B and C. For such cases, you can predefined some trajectories based on the joint states that are defined in **named_joint_states.yaml**. The framework can then retrieveand execute this known trajectory by its name. The different trajectories can be configured to match a given timeline and must be declared following this rule:
```yaml
trajectory1_name:
  - {name: <first_point_name>, interpolate_time: <value_1>, pause_time: <pause_time_1>}
  - {name: <second_point_name>, interpolate_time: <value_2>, pause_time: <pause_time_2>}
  ...
  - {name: <last_point_name>, interpolate_time: <last_value>, pause_time: <last_pause_time>}

trajectory2_name:
  - {name: <point_1_name>, interpolate_time: <value_1>, pause_time: <pause_time_1>}
  - {name: <point_2_name>, interpolate_time: <value_2>, pause_time: <pause_time_2>}
  ...
  - {name: <point_n_name>, interpolate_time: <value_n>, pause_time: <pause_time_n>}

...
```
The parameters ```interpolate_time``` and ```pause_time``` are in seconds and allow you to control the *flow* of the trajectory. The first one constrains the trajectory to be at the given waypoint after ```interpolate_time``` from previous waypoint. The other parameter allows to pause the robot in a specific pause before going to the next waypoint. We believe these two parameters are enough to create a wide range of behaviours with respect to speed and so on. An example of trajectory can be found [here](https://github.com/shadow-robot/modular_benchmarking_framework/modular_framework_api/config/named_trajectories.yaml).

### Sensor plugins
If you are using a specific sensor which outputs should be considered when performing motion planning, you can develop your own plugin so that Moveit integrates the sensor's values in the planning process. The documentation avout how to create your plugin can be found [here](https://moveit.ros.org/documentation/plugins/). If you are using point clouds or depth maps, you can use already existing [plugins](http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/perception_configuration.html). If you are developing your own plugin for a given sensor, make sure to add it within the ```sensors``` field according to the format of the provided example. You can also add all the different parameters required to run the different plugins in this file (loaded in the namespace ```/move_group/<your_param>```).

### Sensors config
Adding sensors to the environment requires to declare them in the [tf2 tree](http://wiki.ros.org/tf2). Instead of creating a node for each sensor in which we have to modify the different parameters everytime the scene changes a bit, you can provide the different values in this config file and the different transforms will be published for you. The only thing you need to do is to declare each sensor with the following format:
```yaml
sensor_name_1:
  frame_id: <frame_id_1>
  origin_frame_id: <origin_1>
  frame_x: <x_coordinate_1>
  frame_y: <y_coordinate_1>
  frame_z: <z_coordinate_1>
  frame_roll: <roll_1>
  frame_pitch: <pitch_1>
  frame_yaw: <yaw_1>

sensor_name_2:
  frame_id: <frame_id_2>
  origin_frame_id: <origin_2>
  frame_x: <x_coordinate_2>
  frame_y: <y_coordinate_2>
  frame_z: <z_coordinate_2>
  frame_roll: <roll_2>
  frame_pitch: <pitch_2>
  frame_yaw: <yaw_2>

...
```
Please note that if any of the fields is missing, the frame will not be added and the corresponding transform won't be published. You can also add all the parameters that you want to use in other nodes and you do not want to hardcode since they might change, such as the name of the topics on which data is published. **You can add sensors that are attached to moving links as well.** For example, if your add to this [file](https://github.com/shadow-robot/modular_benchmarking_framework/modular_framework_api/config/sensors_config.yaml) a new sensor of which origin is ```eg_manipulator```, with any coordinates, you will se that when the robot is moving the frame is changing as well. Which allows for instance to integrate wrist mounted cameras or tactile sensors easily. The following YAML file depicts a setup with a dynamic and a static camera:
```yaml
fixed_kinect:
  frame_id: "simulated_kinect2"
  origin_frame_id: "world"
  frame_x: -0.1645
  frame_y: 0.375
  frame_z: 2.72
  frame_roll: 3.14159265359
  frame_pitch: 0
  frame_yaw: -1.57079632679

wrist_mounted_realsense:
  frame_id: "simulated_realsense"
  origin_frame_id: "eg_manipulator"
  frame_x: 0.
  frame_y: -0.075
  frame_z: -0.165
  frame_roll: 0.436332313
  frame_pitch: 0
  frame_yaw: 0
```
The output is something like [this](???)

## Task Constructor
The different files contained in the folder *task_constructor_script* describe through a simple and intuitive YAML interface the task the robot should perform. Given some *states* representing atomic actions, we believe it possible to create very complex and non linear behaviours that can be used to solve real world problems. A modular way to connect these states to create use cases, while controlling the behaviour when facing an issue, is using state machines. For this purpose we rely on the [smach](http://wiki.ros.org/smach) library. <br/>
We created a layer that simplifies the construction of the state machines, so that users without any knowledge regarding state machines, smach or even ROS can program the robot to perform some specified tasks. We provide a set of states allowing to perform most of the generic tasks related to manipulation, but that can be used for other use cases. The user only needs to link the states together through in a YAML file.<br/>
The YAML file must be structured the following way:
```yaml
name: <state_machine_name>
source: <template_filename>
node_name: <name_of_node_running_state_machine>
outcomes: <list_of_state_machine_outcomes>
states:
  - <state1>:
      source: <state_filename>
      <option_1_state1>: <value_option_1_state1>
      <option_2_state1>: <value_option_2_state1>
      ...
      <option_n_state1>: <value_option_n_state1>
      transitions: {<state_outcome1>: <state2>, <state_outcome2>: <a_state_or_state_machine_outcome>}
  - <state2>:
      source: <state_filename>
      <option_1_state2>: <value_option_1_state2>
      <option_2_state2>: <value_option_2_state2>
      ...
      <option_n_state2>: <value_option_n_state2>
      transitions: {<state_outcome3>: <state_machine_1>, <state_outcome4>: <a_state_or_state_machine_outcome>}
  - <state_machine_1>:
      source: <template_filename>
      <option_1_state_machine_1>: <value_option_1_state_machine_1>
      <option_2_state_machine_1>: <value_option_2_state_machine_1>
      ...
      <option_n_state_machine_1>: <value_option_n_state_machine_1>
      transitions: {<state_outcome3>: <another_state>,  <state_outcome3>: <a_state_or_state_machine_outcome>}
      states:
        - <state11>:
            source: <state_filename>
            <option_1_state11>: <value_option_1_state11>
            <option_2_state11>: <value_option_2_state11>
            ...
            <option_n_state11>: <value_option_n_state11>
        ...
        - <state_n1>:
            source: <state_filename>
            <option_1_state_n1>: <value_option_1_state_n1>
            <option_2_state_n1>: <value_option_2_state_n1>
            ...
            <option_n_state_n1>: <value_option_n_state_n1>
  ...
  - <staten>:
      source: <state_filename>
      <option_1_state1>: <value_option_1_state1>
      <option_2_state1>: <value_option_2_state1>
      ...
      <option_n_state1>: <value_option_n_state1>
      transitions: {<state_outcome_n>: <a_state_or_state_machine_outcome>, <state_outcome_n+1>: <a_state_or_state_machine_outcome>}
```
This interface allows to easily create a wide range of use cases and complex closed-loop behaviours just by changing the transitions between the states. You can nest state machines inside others in order to condense more complex behaviours. We provide two task constructor scripts as examples. One is defining a [pick and hold on a physical robot](https://github.com/shadow-robot/modular_benchmarking_framework/modular_framework_api/task_constructor_scripts/pick_and_hold.yaml), the other one can be used in [simulation for picking objects](https://github.com/shadow-robot/modular_benchmarking_framework/modular_framework_api/task_constructor_scripts/simulation_pick.yaml) based on a given Grasp Pose Detection method. <br/>
**Note that `<state_machine_name>` must match with the corresponding argument in the launch file.** <br/>
A more in-depth tutorial about the [task constructor](./4_task_constructor.md) and the provided states can be found [here](./provided_states.md).

## Launch file
The launch file contains other important information that are required to run the robot. We are going to review all the different arguments that you can set to make the framework fit what you need.
* ```simulation```: If set to ```false```, automatically launches the framework to communicate with the physical robot. Default is ```true``` (and run Gazebo 9).
* ```description_package```: Name of the ROS package containing *scenes*, *worlds* and *models* you may want to use to set up your environment. You can find out more about what is a description package [here](./description_package.md).
* ```robot_package```: Name of the ROS package containing information about the robot such as the *urdf file* and the *controller file*. You can find out more about what is a robot package [here](./robot_package.md).
* ```robot_urdf_file```: Name of the urdf file containing the description of the **whole** robot. It must contain both the arm(s) and the manipulator(s). The file should be contained in the ```robot_package```.
* ```world_file```: Name of the Gazebo **.world** file describing the robot's setup for simulation. An explanation about how to create such files can be found [here](./description_package.md).
* ```scene_file```: Name of the file containing all the information about the collisions so Moveit can plan according to the different obstacles. An explanation about how to create such files can be found [here](./description_package.md).
* ```urdf_args```: Optional arguments that you may need to pass along with the urdf file (for instance to set the position of the robot). **If not needed, leave empty!**
* ```moveit_config_package```: Name of the moveit config package required to operate the robot with Moveit. A tutorial about how to create one using the assistant is available [here](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html). If you don't manage to properly create it you can follow this [tutorial](./creating_moveit_config.md).
* ```controller_file```: Name of the file containing the different controllers that must be loaded by ROS. The file should be contained in the ```robot_package```.
* ```simulation_starting_pose```: **Used only in simulation** and defines the values of each joints in Gazebo when starting the robot.
* ```manipulator_prefix```: String that is contained in all the manipulator's link. For instance it can be `rh`.
* ```states_directory```: Path of the directory containing the states that you want to use to create your state machine.
* ```templates_directory```: Path of the directory containing the templates used to generate the different state machines you might need
* ```state_machine_to_load```: Specify the name of the state machine to run (should end with *.py*). Leave empty if you want to generate a new one.
* ```task_constructor_script```: Name of the yaml script to use for generating the state machine that is going to be used. **Not used** if `state_machine_to_load` is not empty.
* ```generated_state_machine_name```: Name of the file containing the generated state machine (should end with *.py*). **Not used** if `state_machine_to_load` is not empty.


Modifying the parameters in the configuration file and in the launch file allows to set up the robot (and its environment) so they can be used by the framework. Having new states and new *task constructor scripts* allow to create different use cases in a modular way without requiring an extensive knowledge about state machines. With all these options, we believe that you can configure the framework to fit your needs. If not, you can have a closer look to the [core](???) and try to modify it.
