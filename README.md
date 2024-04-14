# goal_state_publisher
Ros package GoalStatePublisher for Master Thesis

This is a helper package to publish commands from/to moveit and from/to the robot controller. It controls repulsive force fields and feeds them to the controller in the force_control package and iuncludes the demo.

Force Fields:
-rosrun goal_state_publisher moveit_collision_checker (will filter BBOXES colliding with robot)
-rosrun goal_state_publisher force_field (publishes force field to controller)
-when node is ready publish a planning scene manually from rviz or via custom node to the force_field node, then all bounding boxes and potential fields are updated at       high frequency (20Hz, can be set via argc)

demo:
-rosrun goal_state_publisher demo
This starts the demo. You have different options and a text prompt will pop up in the terminal. You can choose from one of three pre-programmed tasks (or doing all of them in the corresponding order 1-3). There is also the option to activate/deactivate the stiffness of the robot (free-float) and to move it to a position. For moving to positions you will be prompted to enter the reference pose.

Note:
-  for force control tasks, we only have task 3 at the moment. The force control is handled inside of the controller and the force magnitude is fixed at 10N in negative z-direction. All commands pointing to any other behaviour in the code will do nothing!
_________________________________________________________________________________________________________________
Update Dec. 2023.
-Added Action Primitives
-Added action_primitive_tester for testing, impedance_setter_node for communictation with controller
-Added custom messages for Impedance updates and Action Primitives to communicate with the controller (see: custom_msgs pacakge)
____________________
Action Primitives:
The base class is declared in ActionPrimitive.h. AtomicTasks.h inherits from ActionPrimitive and there all the
Action Primitive tasks and their execution are defined (Get Me, Follow Me, Hold This, Take this, Avoid Me).
The Impedance Parameters for each task are set fixed in AtomicTasks.cpp in the constructors corresponding to the task.
____________________
ImpedanceParameterController:
This class organizes the action primitives and controller communication on a high level. It receives callbacks, such as the hand position
and updates or executes action primitives and their goal parameters accordingly.
______________________
action_primitive_tester/impedance_setter_node nodes
-run as "rosrun goal_state_publisher action_primitive_tester/impedance_setter_node"
in action_primitive_tester you can choose which action primitive to execute (there are some dummy values written to the task).
Impedance_setter_node receives the requested task and handles the execution via an ImpedanceParameterController instance. 
Each time a new action primitive is called (the impedance_setter_node just waits for callbacks), the corresponding impedance parameters are communicated 
via custom_msgs/ImpedanceParamMsg to the controller. The task is transferred from the tester node (mimicking the hololens) via action_primitive_message.
________________________
To Run full example on gazebo you can run
1) roslaunch franka_gazebo panda.launch controller:=cartesian_impedance_controller (or similar if you have another config - the controller needs to run on the robot)
2) rosrun goal_state_publisher action_primitive_tester 
3) rosrun goal_state_publisher impedance_setter_node

You will be promted by 3) to enter a number - the task will be executed accordingly. Enjoy :D

_________________________________________________________________________________________________________________
Update March 2024.
-Added Moveit Collision checker node, to filter bounding boxes colliding wh the robot