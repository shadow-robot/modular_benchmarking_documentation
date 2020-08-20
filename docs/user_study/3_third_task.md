# Task 3: Simplified pick and place

The purpose of this task is to make you use another robot and to integrate through another mean. Here we are going to use an already existing launch file that will run all the required components to control a different robot arm and the Shadow Dexterous Hand. Once integrated, create a state machine to fake a pick and place. The robot should go from its current pose to a pose A to pick the object and should drop it at pose B. Pose A and B can either be joint states or poses (to be selected from provided list).

**Using a launch file, run the Shadow Dexterous Hand mounted on a UR10 and add the necessary states to the previous task to fake a pick and place.**

**Note:**
- The launch file already runs MoveIt!
- The launch file should be run with both options "scene" and "start_home" set to "true".

## Related documentation
* [Integrating a robot from a launch file](./integrate_from_launchfile.md)
* [Adding poses](./adding_poses.md)

## Resources
* [URDF file](https://github.com/shadow-robot/sr_interface/blob/kinetic-devel/sr_multi_description/urdf/right_srhand_ur10.urdf.xacro)
* [launch file](https://github.com/shadow-robot/sr_interface/blob/kinetic-devel/sr_robot_launch/launch/sr_right_ur10arm_hand.launch)
* [List of valid joint states](???)
* [List of valid poses](???)
