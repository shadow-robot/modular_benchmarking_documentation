# The framework's core
The framework's core [package](https://github.com/shadow-robot/modular_benchmarking_framework/tree/kinetic-devel/modular_framework_core) gathers all the scripts, nodes, utils and messages available and linking all the parts of the framework. If you want to change any functionality, this is where you should start investigating.

## Framework's managers
Regardless of the problem we want to solve, the ROS messages that are being used are most of the time similar and are derived from standard messages. When creating complex behaviours, it is common to have a lot of different topics created to communicate such messages generated in several nodes. In order to make these communications easier, we implemented six managers that allow dealing with the different messages for you. You can learn more about them, and how to create you own [here](./5_1_managers.md).

## Framework's states
In order to easily create some complex tasks, we implemented six states that allow to build the most common tasks in robot manipulation. Of course, we could not think about all the different use-cases and needs. That is why you can find [here](./5_2_states.md) the details of our states as well as a tutorial to create your own.

## Framework's simulation mode
Working in simulation is more and more popular to speed up data collection of the robot's interaction with its environment. In order to do so, we propose a simple command line based interface in order to dynamically interact with the simulated world and place or remove objects at the user's will. You can read more about it [here](./5_3_simulation_mode.md).

## Framework's messages
We provide some messages that try to unify the different representations of grasping. We believe that some of their content can also be used for different tasks. In order to interact easily with widely used ROS messages, we also provide some methods in C++ and python. You can find out more about our messages or the different tools [here](./5_4_messages.md)
