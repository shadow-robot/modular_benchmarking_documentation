# Robot package
This short tutorial explains what is a robot package, why we need it and what it should contain. It also give an overview about what is a *controller file* and how to modify them.

## What is it?
A robot package contains all the information that is vital to control a robot. It **must** contain urdf and controller files describing the robot and how to control it. The different files should be organized in two different folders:
```
- controllers
- urdf
```

## Controller file
A *controller file* contains all the information about how ROS should operate the different robots it recognizes. It means that only controllers for physical robots that can be controlled with ROS controllers should be defined there. When running the simulation mode, Moveit *assumes* that the robot can be controlled that way, which means that you need to add the controllers to the file. <br/>
Each controller should be defined in the same file and each must follow this template:
```yaml
<controller_name>:
  type: "<controller_type>"
  joints:
    - <joint_name_1>
    - ...
    - <joint_name_n>
  constraints:
    goal_time: <goal_time>
    stopped_velocity_tolerance: <velocity_tolerance>
    <joint_name_1>: {trajectory: <value>, goal: <value>}
    <joint_name_2>: {trajectory: <value>, goal: <value>}
    ...
    <joint_name_m>: {trajectory: <value>, goal: <value>}
  stop_trajectory_duration: <trajectory_duration>
  state_publish_rate: <publish_rate>
  action_monitor_rate: <monitor_rate>
  allow_partial_joints_goal: <boolean>
```
If you create your own controller (derived from `controller_interface::ControllerBase`) declared in a plugin, you must define it in this file as well. Please note that some of the options, such as *constraints*, *stop_trajectory_duration* or others may not be relevant. <br/>
Whether in simulation or when operating the physical robot, the framework is relying on Moveit, so if you are using native ROS controllers, please make sure that they are also included (**with the same name**) in the `controllers.yaml` file of your moveit config package. If you are using your own controller (derived from `controller_interface::ControllerBase`) you **don't** need to include it in the moveit config package. But if the physical manipulator you want to run does not have a hardware interface, then must not add anything in this file or in the moveit config package.
