# Framework's messages and action
In addition to the classic ROS messages and actions, the framework supports some home made messages and action that can be used in your scripts. We also provide some handful methods that allow to create some messages easily in order to avoid boilerplate. All the messages can be found in this [folder](???) and the action can be found [here](???).

## TorqueIntensity
This message contains the information required to *squeeze*. As a matter of fact, it exists some two-steps controller for grasping. The first one is to use position controller (for instance) to actuate the joints to a specific state. Once the latter is reached, torque is applied to some joints. The name of such joints must be specified in `joint_names`. Since each joint can apply a different percentage of `max_torque`, the value (between -1 and 1) corresponding to each joint must be specified in `torque_intensity`. A negative value would mean motion in the outward direction for instance.

## ManipulatorState
This message fully describes the state of the manipulator. It contains a field `posture` which is a [JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) describing the internal state of the manipulator. It also contains the field `pose` which describes with a [PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) the pose of the manipulator. <br/>
We believe that these information allow to specify the state of the manipulator regardless of the application.

## StandardisedGrasp
The definition of a grasp differs greatly from one work to another. We propose here a quite generic definition of a grasp. Since you can use only some of the fields contained in this message, we believe that this message can be compatible with most of the different works about grasping. <br/>
In our definition, a grasp can be characterised by:
* `grasp_id`: String that can be used to name the given grasp. Especially useful when stored in a database.
* `hand_id`: String that specifies for which manipulator the grasp has been created. Useful when storing messages in a database.
* `object_id`: String specifying for which object the grasp has been generated. Especially useful when grasping known  or familiar objects. Can also be used to do a query in a database.
* `pregrasp`: ManipulatorState message that contains how should be the manipulator before grasping. Can be filled with an empty ManipulatorState message.
* `grasp`: ManipulatorState message containing how the manipulator should be for grasping an object. Can be filled with an empty ManipulatorState message.
* `postgrasp`: ManipulatorState message containing how the manipulator should be for after grasping an object. It can be seen as the *release* state. Can be filled with an empty ManipulatorState message.
* `torque_intensity`: TorqueIntensity message that contains how the manipulator should behave when a two-steps controller is used. Can be filled with an empty TorqueIntensity message.
* `grasp_quality`: Float estimating the probability of success of the given grasp. This value highly depends on the task carried out, and to the metric being used. These choices is left to you. Can be `0`.

As you can see, most of the methods generating grasps provide at least some of the information contained in this message. Thats is why with some simple modification we can easily make it compatible with most of the methods.

## GraspCommand

## Helpers
Here are all the functions that can be used both in python and C++
