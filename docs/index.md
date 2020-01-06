# The Modular Benchmarking Framework Documentation
This is the starting point for the Modular Benchmarking Framework Documentation.

## Motivation
Robotic manipulation is a high level and complex task which combines a range of different components. Although one might want to compare the overall system, it becomes hard to dissociate and evaluate each component of a manipulation pipeline, and therefore to obtain a fair comparison between different specific solutions. Hence, we propose the Modular Benchmarking Framework, a ROS-based framework that enables to: 1) easily benchmark different solutions with minimal coding and integration effort and 2) separately compare each component of a pipeline, which is essential to drive progress in this area. <br/>
Robot arms with a ROS interface and manipulators can be integrated without too much effort, allowing to save effort and time that could be spent on developing algorithms to solve real world problems. This framework also allows easy integration of a wide range of methods for manipulation without caring about which part is robot-specific, allowing researchers that are relatively new to robotics to easily experiment their methods.

## Contents
* [Installing the framework](user_guide/1_installing_the_framework.md)
* [Configuring the framework](user_guide/2_configuring_the_framework.md)
* [Integrating hardware to the framework](user_guide/3_integrating_robot.md)
* [Using the task constructor](user_guide/4_task_constructor.md)
* [Framework's core](user_guide/5_framework_core.md)
* [Contributing](user_guide/6_contributing.md)
