# Provided states
In order to create a range of applications, we provide six states that can be used natively when installing the framework. We tried to make them as generic as possible so you don't need to implement yours, but if for some reasons it happens, you can get inspiration from those.

## Common features
We have tried to make our states as parametrisable as possible so they can be used in a lot of different use cases. In the [task constructor](./4_task_constructor.md) you can directly, for all our states, set the following arguments:
* `outcomes`: List of string that describes the outcome of the state. Default is `["success", "fail"]`.
* `input_keys`: List of string gathering the name of the different fields of the userdata that is going to be used **as input**.
* `output_keys`: List of string gathering the name of the different fields of the userdata that is going to be used **as output**.
* `io_keys`: List of string gathering the name of the different fields of the userdata that is going to be used **both as input and output**.

Depending on the purpose of the state the default value of such fields will differ so you don't have to modify them except if you need to. Depending on the functionality of the state, other parameters may need to be configured in the task constructor.

## Waiting for a signal
While building a task, it is quite interesting to be able to *control* the flow of the state machine. For instance, when benchmarking, we might want to trigger a generative method manually everytime and the state machine to wait for this method to be launched before proceeding. That is why we provide a configurable blocking state that allows you to *control* the state machine flow. In the task constructor you can set:
* `topic_name`: String stating the name of the topic containing a boolean message
* `timeout`: Optional integer stating how long the state should block (in seconds). Default is `None`, meaning that it will be blocking until it receives a message on the given topic.

This state will listen to the topic `topic_name` that must contain a [Boolean message](http://docs.ros.org/melodic/api/std_msgs/html/msg/Bool.html) and will return the *positive* outcome if and only if the content of this message is `True`. You can also specify how long this state should be blocking. You can find more details about the implementation [here](???).

## Allowing collisions for the manipulator
While carrying out a manipulation related task, we want to be able to control the collisions between the manipulator and the environment. For instance we usually don't want the robot hand to collide with the table but we want the robot to be able to collide with a given object to manipulate it. If you are using Moveit to plan and control your robot hand (whether in simulation or in real world), we provide a state that allows collisions between the different links of the manipulator and/or collisions between the manipulator and some objects. In order to make the most of it using the [task constructor](./4_task_constructor.md), you can change these parameters:
* `allow`: Boolean specifying whether the state should allow or disallow collision check for the hand
* `collision_type`: Optional string telling what kind of collision should be allowed. It must be either `""`, `self-collision` or `object-collision`. Default is `""` which is equivalent to a simple getter of the [ACM](http://docs.ros.org/kinetic/api/moveit_core/html/classcollision__detection_1_1AllowedCollisionMatrix.html).
* `objects`: Optional list of objects we want to allow the manipulator to collide with. The given objects **must** be part of the scene and be part of the *models* folder of the [description package](./description_package.md). Default is `[]`, implying that all objects added to the original scene will be considered. The code of the state can be found [here](???)

**Please note that you need two distinct states, one for allowing and the other one for disallowing collisions**.

## Selecting messages to store in the userdata
Some methods generate a set of solutions to a given problem. But considering only the best one (according to some metric) might be problematic. As a matter of fact, if this solution is not feasible, then this would fail the state machine and terminate the task. Instead of having to start over and over, we propose a state that allows to select some messages stored in one of our [managers](./managers.md) and select them one after the other. That way, everytime the state is executed, another message can be selected and integrated in the userdata. You can hence create a *loop* to this state that will keep updating the userdata with a message until the task has been successful. In the task constructor you can set this state using the following parameters:
* `message_type`: String specifying the type of message to select. Can be `joint_state`, `trajectory`, `plan`, `pose`, `grasp`, `pregrasp` or `postgrasp`.
* `output_keys`: List that should contain `selected_<message_type>` in order to store the selected message in the userdata.
* `message_names`: Optional list containing the names of the messages to retrieve and select one after the other. Default is `None`, meaning that the latest [anonymous](./messages.md) `<message_type>` will be selected.

You can find more information about the state in this [file](???).

## Executing a grasp
Once a grasp has been generated by a given method, we want at some point to actuate the manipulator to execute the predicted grasp. For this purpose we provide a hardware-agnostic state that, given a [StandardisedGrasp message](???) will execute it using the action server that has been loaded by the framework. You can find more information about the action server [here](./3_integrating_robot.md). **This state is just taking care of actuating the manipulator for grasping**. <br/>
The message containing the grasp definition can be provided to the state by two means. The first one is to store the grasp to execute in the `userdata` under the name `selected_grasp`. As a matter of fact, if the input keys of this state contain such keyword, then this grasp will be executed. Otherwise you can directly specify the name of the grasp that must be retrieved from the [standardised grasp manager](???), and it will be executed. In the task constructor, you can specify:
* `grasp_type`: The [manipulator state](???) to be executed, can be `pregrasp`, `grasp` or `postgrasp`.
* `grasp_name`: Optional string that states which grasp message should be retrived from the manager. Default is `""`, meaning that it will be the latest [anonymous grasp](./messages.md) that will be retrieved.
* `input_keys`: Optional list that can contain `selected_grasp` in order to specify that the state must execute the grasp saved in the userdata. If `selected_grasp` is part of the input keys then our state will use it by default.

The two different ways to provide grasp messages and the trick about anonymous grasps should allow you to cope with a range of applications involving grasping. You can find our implementation of the state [here](???).

## Planning a motion for the robot arm
Most of the time, the robot arm's environment is quite complex, which forces us to add an extra step before moving the arm: planning the motion. We provide therefore a generic state allowing to plan a motion for the robot arm from a starting to a target state using Moveit. Once again, you can provide our state the starting and target robot state by two means. You can either retrieve them using the [provided managers](./managers.md) or through the state machine's userdata. The different options that you can set from the task constructor are:
* `target_state_type`: String specifying the target state's type. Can be `pose`, `joint_state`, `grasp`, `pregrasp` or `postgrasp`.
* `target_state_name`: Optional string containing the target state's name. Used to retrieve it using the corresponding manager. Default is `""` meaning that the latest anonymous `target_state_type` will be retrieved.
* `plan_name`: Optional string containing the name that will be given to the computed plan. Can be empy to make it [anonymous](./messages.md). Default is `""` meaning that it will make it anonymous.
* `starting_state_type`: Optional string specifying the starting state's type. Can be `""`, `pose`, `joint_state`, `grasp`, `pregrasp` or `postgrasp`. Default is `""` meaning that the current robot's state will be used as starting state.
* `starting_state_name`: Optional string containing the starting state's name. Used to retrieve it using the corresponding manager. Default is `""` meaning that the latest anonymous `starting_state_type` will be retrieved.
* `input_keys`: Optional list that can contain `selected_<type>` with `<type>` being the different types supported in order to specify that the state must

   **/!\ Look at the code with different combinations and conditions!???**

## Executing a plan for the robot arm
One obvious action that must be pplan_nameerformed to solve any manipulation problem is moving the robot arm. Once the motion plan has been computed, this state can either execute it if it's part of the userdata or retrieve it from the [moveit plan manager](???). You can set this state in the task constructor using the following parameters:
* `plan_name`: Optional string containing the name of the plan to retrieve from the manager. Default is `""` meaning that the latest anonymous plan will be picked.
* `input_keys`: Optional list that can contain `selected_plan` in order to specify that the state must execute the plan saved in the userdata. If `selected_plan` is part of the input keys then our state will use it by default.

**/!\ Give example of possible combinations???**
The code of this state can be found [here](???).
