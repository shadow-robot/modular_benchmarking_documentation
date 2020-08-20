# State machines

The GRIP framework allows for visual programming of robotic tasks for grasping and manipulation. GRIP relies on hierarchical state machines to model a robot's behaviour.
A state machine is a model that depending on the outcome can have different transitions and thus different sequences of actions depending on the situation.

The design of behaviours in GRIP requires two components: state machine containers and states. State machine containers provide different execution semantics (sequential, concurrent, etc.) of the task. States are atomic components that are going to carry out specific computations. Once a state is finished it will transition to another one. In order to avoid to implement this in a textual way, we implemented a GUI allowing to define state machines through simple drag-and-drop. When starting the framework, the base state machine is already created. You can then nest state machines and/or add states to create the behavior. Each state and state machine is further configurable depending on what it is for.
