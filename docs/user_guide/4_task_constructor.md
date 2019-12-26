# Using the task constructor

One of the goal of the Modular Benchmarking Framework is to be able to separate each component used for solving a problem related to robotic manipulation. This way, we can easily re-use any component that seems powerful in a given context to another and study how good or bad it can be transfered or combined with other components. In order to do so, we need to be able to easily create a wide range of tasks the robot should execute independently of what hardware it is or which method is being used. Then, we can thoroughly compare the different metrics obtained during the experiments and conclude that, for instance, grasp synthesis method A is better than method B given a specific motion planner, controller and so on for a specific task. <br/>

## Principle
We believe that most of the tasks related to manipulation can be represented as a (complex) combination of *atomic* actions, such as *planning*, *predicting a pose*, *generating a trajectory* and so on. Considering such *simple* actions and linking them together has several benefits. The first one is to be able to create more complex and task-oriented behaviours through the links between the actions. The other one is that we can link each component of a method to one or several actions and therefore allow to analyze the *real* performance of each component individually and test different combinations. Considering each action to be a *state*, then a task can be defined as a *[state machine](https://en.wikipedia.org/wiki/Finite-state_machine)*.

## Creating a state machine
We propose an intuitive way to create state machines, which does not require any knowledge about ROS or how to implement state machines using [smach](http://wiki.ros.org/smach). The only thing you need to do is to describe in a YAML file which states (or state machines) you want to use and how to link them, that's all. The YAML file should be store inside the folder `task_constructor_scripts` of the *modular_framework_api* package.

### Format of the YAML files
Each task constructor script (YAML file) must be structured the following way:
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
Let's go over the above template and explain the different parts.

#### Script header
The header **must** be composed of:
* `name`: Name you want to give to the file containing the generated state machine
* `source`: Name of the *template file* that is going to be used to generate the state machine. It **must** be contained in the `templates_directory` arguments passed in the launch file.
* `node_name`: Name you want to give to the node that is going to run the generated state machine
* `outcomes`: List of different outcomes of the root state machine. For instance `[success, failure]`.
* `states`: Defines all the states or state machines that you are going to need to create your task.

#### Using a state
Each state you want to use should be defined as follow:
```yaml
- <state_name>:
    source: <state_filename>
    <state_option_1>: <state_option_1_value>
    <state_option_2>: <state_option_2_value>
    ...
    <state_option_n>: <state_option_n_value>
    outcomes: <list_of_outcomes>
    transitions: {<outcome_name>: <state_or_outcome>, <outcome_name>: <state_or_outcome>}
```
The can give any name to your states, we just advise you that they are meaningful in order to get a hang of what the task is about just by looking at the name of the differents states.
* `source` **must** be the name of the python file in which the state is implemented. The file **must** be located in the `states_directory` argument of the launch file.
* As long as the states are properly formated (see [here](???)) you can also define the different options direclty in the YAML file.
* `outcomes` define the different outcomes of the states. It must be a list, such as `[success, failure]`.
* `transitions` **must** be specified as they are defining how to link the different states (or state machines) together. For each potential outcome of the state, specifies what should be done next. If you want the state called `DummyState` to follow your state if the latter outputs `success` then this field should be `{success: DummyState}`.

#### Nesting a state machine
Some components of a behaviour can themselve be designed as whole state machines. We can imagine a small state machine that plan and move the robot arm to its initial pose. <br/>
You can directly create such nested state machines inside the task constructor script using the following template:
```yaml
states:
  - ...
  - ...
  - <state_machine_name>:
      source: <template_filename>
      <state_machine_option_1>: <state_machine_option_1_value>
      <state_machine_option_2>: <state_machine_option_2_value>
      ...
      <state_machine_option_n>: <state_machine_option_n_value>
      transitions: {<outcome_name>: <state_or_outcome>, <outcome_name>: <state_or_outcome>}
      states:
        - <state_machine_state_1_name>:
            source: <state_filename>
            <state_1_option_1>: <state_1_option_1_value>
            <state_1_option_2>: <state_1_option_2_value>
            ...
            <state_1_option_n>: <state_1_option_n_value>
        ...
        - <state_machine_state_m_name>:
            source: <state_filename>
            <state_m_option_1>: <state_m_option_1_value>
            <state_m_option_2>: <state_m_option_2_value>
            ...
            <state_m_option_n>: <state_m_option_m_value>
```
You can of course create several nested state machines in the same root state machine or created recursively nested state machines. The only restriction is that the source file of the nested state machines **must** be *template files* such as the files you can find [here](???), and you **must** define the transitions in order to know what to do when the nested state machine is done. Please note that we already provide template files allowing to create a state machine **fully compatible** with the framework, as well as a [concurrent state machine](http://wiki.ros.org/smach/Tutorials/Concurrence%20container). In our implementation, one of the option for the state machine is to specify the information it should have access to (userdata). When using nested state machine, we often want to kinf od *inherit* the same userdata as the parent. You can do it with `userdata: self.userdata`.

#### Miscellanous
Although we tried our best to simplify the creation of state machines by preventing users to dive into different tutorials and face the numerous boilerplate that would come with it, defining a complex state machine might be a bit painful. That is why when [creating new states](???) (or using the one we [already provide](???)) you can define some default values for the options that are unlikely to be changed and not specify them in the task constructor script. <br/>
When creating state machines (especially nested ones), it might be a bit painful to copy/paste some parts such as the input/output keys that might be the same for several states. To simplify this it is possible, in a state or a state machine, to create a `params` field which **must** be a dictionary in which you can store values and reuse them in the script. An example of `params` could be `params: {outcomes: [success, fail]}`. If this field has been added to a state machine then all the children states can use `outcomes: params.outcomes`. Please note that the same principle can be applied within a given state in order to shorten the definition of a state and make (once you get used to it) the task constructor script more compact and readable.

### Examples
In this subsection we are going to give some examples of state machines based on the [provided states](???) and the [framework template](???). The point is not to have meaningful examples in term of task but rather to demonstrate how to use the task constructor.

#### Making a task constructor script more compact
Let's start with the following task constructor script:
```yaml
name: just_planning
source: framework_state_machine
node_name: just_planning_sm
outcomes: [sm_successful, sm_failed]
states:
  - PlanMotion:
      source: state_plan
      target_state_type: joint_state
      target_state_name: stability_pose
      plan_name: current_to_stability
      starting_state_type: ""
      starting_state_name: ""
      outcomes: [success_to_plan, fail_to_plan]
      input_keys: []
      output_keys: []
      io_keys: [arm_commander]
      transitions: {success: sm_successful, fail: sm_failed}
```
If you are launching the framework using this [config file](???) then you should have a state machine that plans from the current robot pose to the joint state named `stability_pose`. The state named `PlanMotion` showed above contains all the possible parameters that cen be changed in our [implementation](???). Taking the most of the default values defined in the state, we could simply shorten it to
```yaml
name: just_planning
source: framework_state_machine
node_name: just_planning_sm
outcomes: [sm_successful, sm_failed]
states:
  - PlanMotion:
      source: state_plan
      target_state_type: joint_state
      target_state_name: stability_pose
      plan_name: current_to_stability
      outcomes: [success_to_plan, fail_to_plan]
      transitions: {success_to_plan: sm_successful, fail_to_plan: sm_failed}
```
Now, using the `params` trick you could have exactly the same behaviour created from
```yaml
name: just_planning
source: framework_state_machine
node_name: just_planning_sm
outcomes: [sm_successful, sm_failed]
states:
  - PlanMotion:
      source: state_plan
      params: {target_state_type: joint_state, target_state_name: stability_pose, plan_name: current_to_stability, outcomes: [success_to_plan, fail_to_plan]}
      transitions: {success_to_plan: sm_successful, fail_to_plan: sm_failed}
```
Here, we are using `params` within a state, and since **its keys are matching the names of the state's options** then we don't need to specify them afterwards. We will provide another example of how to use the `params` trick.

#### Using a concurrent state machine
Let's say that we want the robot to go to a first known pose and then to another one. You have a lot of different ways to do it, such as creating a trajectory in the framework and execute it, for the sake of this tutorial, we are going to design it as a state machine.
```yaml
name: mock_trajectory
source: framework_state_machine
node_name: mock_trajectory_sm
outcomes: [sm_successful, sm_failed]
states:
  - ConcurrentPlan:
      source: concurrent_state_machine
      params: {state_outcomes: [success, fail], state_io_keys: [arm_commander], target_type: joint_state}
      name: simple_concurrent_planning
      outcomes: [sucess_plans, fail_plans]
      userdata: self.userdata
      default_outcome: fail
      outcome_map: {success_plans: {PlanInitPose: success, PlanStabPose: success}}
      transitions: {success_plans: MoveInit, fail_plans: sm_failed}
      states:
        - PlanStabPose:
            source: state_plan
            target_state_type: params.target_type
            target_state_name: initial_pose
            plan_name: current_to_init
            io_keys: params.state_io_keys
        - PlanStabPose:
            source: state_plan
            target_state_type: params.target_type
            target_state_name: stability_pose
            plan_name: init_to_stability
            starting_state_type: params.target_type
            starting_state_name: initial_pose
            outcomes: params.state_outcomes
            io_keys: params.state_io_keys
  - MoveInit:
      source: state_move
      params: {outcomes: [success, fail], io_keys: [arm_commander]}
      plan_name: current_to_init
      transitions: {success: MoveStab, fail: sm_failed}
  - MoveStab:
      source: state_move
      params: {outcomes: [success, fail], io_keys: [arm_commander]}
      plan_name: init_to_stability
      transitions: {success: sm_successful, fail: sm_failed}
```
When using this script with the framework, you should see your robot going successively to `initial_pose` and then to `stability_pose`. Since we know beforehand the starting and ending pose of the robot, it is possible to plan for both motions at the same time, saving execution time. For this purpose we have to use a [concurrent state machine](http://wiki.ros.org/smach/Tutorials/Concurrence%20container). As you can see, the differents states executed within the latter requires as an input key `robot_commander`, so we are *inheriting* the userdata from the root state machine. You can also see that `params` here is being used in order to declare in a nested state machine the different parameters that can be used in **all** of its children states.

#### How to use `params`
The two different examples that we have seen so far show some examples about how to use `params`. However, there are some constraints about how to use this trick. For instance, you **cannot** *add* `params` from a parent and the state itself. As a matter of fact, the `params` declared within a state would **overwrite** the parent's `params`, so keep this in mind. This is also true for the nested state machines. If you have have defined a `params` in the header of the root state machine, everything will be overwritten for all the states contained in the named nested state machine. You can see that as the equivalent of *scope* variables.

#### Real-world state machines
We provide two task constructor scripts that should natively work with the framework. One is made especially for picking an object in [simulation](???) and the other is about [pick and place](???) on a physical robot not fully integrated to ROS. Both use-case rely at some point on a grasp-pose detection method that will determine where and how the manipulator should be in order to grasp the object. As explained in the introduction of this tutorial, the state machine itself is almost method-independent so that you can just change the method for benchmarking for instance.

## Using a state machine