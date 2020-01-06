# Integrating a robot to the framework
As stated in the [previous section](./2_configuring_the_framework.md), the framework natively supports Universal Robot arms and is provided with all the packages required to operate (both in simulation and on physical setup) an EZgripper attached to an UR5 arm.<br/>
It is nevertheless likely that you are working with a different hardware. We are hence going to detail how to integrate different kind of hardware.

## Common steps
The first step is to create an urdf (or xacro) file in which both the arm(s) and the manipulator(s) are defined such as [here](https://github.com/ARQ-CRISP/ARQ_common_packages/blob/master/arq_robots/urdf/arq_ur5_with_ezgripper.urdf.xacro) or [here](https://github.com/shadow-robot/sr_interface/blob/kinetic-devel/sr_multi_description/urdf/right_srhand_ur10.urdf.xacro). <br/>
The second step is to create a *moveit_config* package for the robot. You can either use the [Moveit! Setup Assistant](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) or follow this step by step [tutorial](./creating_moveit_config).

## Using your robot in simulation
Now that you have your moveit config package ready, the only remaining step is to make it compatible with Gazebo.<br/>
In order to use ros controllers with your robot, you need to add some elements to your urdf file. Gazebo uses a specific element that links actuators to joints, called `<transmission>`. Each of these elements must contain **at least**:
* `<joint name= >`: which corresponds to the name of a defined joint in your urdf file that you want Gazebo to be able to actuate.
* `<type>transmission_interface/SimpleTransmission</type>`: which specifies the type of transmission. More information [here](https://wiki.ros.org/urdf/XML/Transmission).
* `<hardwareInterface>` that should be present in both `<joint>` and `<actuator>` tags. It states *gazebo_ros_control* plugin what hardware interface to load (position, velocity or effort interfaces).

The last step is to add the *gazebo_ros_control* plugin to the urdf file. The plugin should look like this:
```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <!-- Optional - Default being the value of robot name in the urdf/sdf file-->
    <robotNamespace>my_name_space</robotNamespace>
    <!-- Optional - The period of the controller update (in seconds), default is Gazebo's period -->
    <controlPeriod>my_value</controlPeriod>
    <!-- Optional - The location of the robot_description (URDF) on the parameter server, default being '/robot_description' -->
    <robotParam>my_param_value</robotParam>
    <!-- Optional - The pluginlib name of a custom robot sim interface to be used (see below for more details), default being 'gazebo_ros_control/DefaultRobotHWSim' -->
    <robotSimType>my_value</robotSimType>
  </plugin>
</gazebo>
```
If you want to use your own controller, you can follow [this tutorial](http://gazebosim.org/tutorials/?tut=ros_control) and change the proper parameters. Now that your robot can be (in simulation at least) actuated using ROS controllers, you need to specify them both in the `controllers.yaml` file of your moveit config package and in the *controller file*. <br/>
Now you should be able to control, in simulation, your robot with Moveit and see that the motions planned and executed in Rviz match with one happens in the Gazebo window launched when the framework is being run in simulation mode.

## How to know if my manipulator is *fully integrated to ROS*?
If you are using a manipulator that can be actuated using [ROS controllers](http://wiki.ros.org/ros_controllers), then it means that your manipulator is fully integrated to ROS. It also means that your robot has a [hardware interface](http://wiki.ros.org/hardware_interface). If you have a doubt about that, try to find out if you have any file containing the definition of classed derived from `hardware_interface::RobotHW`. If you do, you can directly go to [this subsection](#using-a-physical-manipulator-that-does-have-a-ros-hardware-interface), otherwise please read the following subsection.

## Using a physical manipulator that does **not** have a ROS hardware interface
If you are using a manipulator that cannot be operated using ROS controllers, no worries you can still integrate your hardware and make it work with the framework. <br/>
It is highly likely that a ROS node launched in one of your launch files is performing the control and send the information directly to the robot's drivers. If you have developed it you know where it is, otherwise you just need to find it and isolate it. In order to integrate it to the framework, you need to transform it to an [action server](http://wiki.ros.org/actionlib/Tutorials). You will then be able to control it as you wish by sending action requests that you can pre-empt and get feedback if anything goes wrong. You can find an example [here](https://github.com/bdenoun/EZGripper/blob/master/ezgripper_driver/nodes/ezgripper_controller.py).<br/>
Once you have the action server that is able to control the manipulator, change the different options in `manipulator_controller_parameters.yaml` so your node can be launched (**do not forget to** `chmod +x` **if it's a python file**). Now you can create a second *controller file* in which you will only have the ROS controller defined for the robot arm (like [this](https://github.com/ARQ-CRISP/ARQ_common_packages/blob/master/arq_robots/controllers/ur5_ezgripper_position_controllers.yaml)). In `start_framework.launch`, change the name of the controller file to be used to point to the one you just created. In order to avoid having bothersome warning and error messages (everything should still work though), do not forget to **comment out** the reference to the controller for your manipulator in `controllers.yaml` of your moveit config package. <br/>
And here you are, you should now be able to control the manipulator via the framework. In order to test it, you can create a minimalist [action client](http://wiki.ros.org/actionlib/Tutorials) and after launching the framework, run the client in order to send any command to the manipulator. It should now move.

## Using a physical manipulator that does have a ROS hardware interface
If you already have a ROS hardware interface for your manipulator, you might need to slightly change it. The only constraints are that the given hardware interface must have a **parameter-less** constructor and must have an `init` method with the following signature: `init(ros::NodeHandle &n, ros::NodeHandle &robot_hw_nh)`. You can find an example of hardware interface [here](https://github.com/shadow-robot/sr_ur_arm/blob/kinetic-devel/sr_ur_robot_hw/src/sr_ur_robot_hw.cpp). <br/>
You can now use the standard ROS controllers on the manipulator. If you opt for this solution, do not forget to specify it in the *controller file* and to possibly change the nature of the controller in `controllers.yaml` of your moveit config package. You should now be able to control the physical robot with Moveit. The action server node provided in the framework can be used (but you can use your own if you wish). <br/>
If you want to use a more convoluted controller for specific use cases, you can still create it. The class must be derived from ```controller_interface::ControllerBase``` and contain an action server. You can then create a plugin so the controller can be recognized by ROS and use it freely. An example is given [here](http://wiki.ros.org/ros_control/Tutorials/Creating%20a%20controller). Once again, define it in the *controller file* and comment out any reference to the manipulator in `controllers.yaml` of your moveit config package. Once the framework launched, you should be able to control the manipulator through a very minimalist [action client](http://wiki.ros.org/actionlib/Tutorials).
