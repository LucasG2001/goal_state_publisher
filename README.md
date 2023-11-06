# goal_state_publisher
Ros package GoalStatePublisher for Master Thesis

This is a helper package to publish commands from/to moveit and from/to the robot controller. It controls repulsive force fields and feeds them to the controller in the force_control package and iuncludes the demo.

Force Fields:
-rosrun goal_state_publisher force_field (publishes force field to controller)
-when node is ready publish a planning scene manually from rviz or via custom node to the force_field node, then all bounding boxes and potential fields are updated at       high frequency (20Hz, can be set via argc)

demo:
-rosrun goal_state_publisher demo
This starts the demo. You have different options and a text prompt will pop up in the terminal. You can choose from one of three pre-programmed tasks (or doing all of them in the corresponding order 1-3). There is also the option to activate/deactivate the stiffness of the robot (free-float) and to move it to a position. For moving to positions you will be prompted to enter the reference pose.

Note:
-  for force control tasks, we only have task 3 at the moment. The force control is handled inside of the controller and the force magnitude is fixed at 10N in negative z-direction. All commands pointing to any other behaviour in the code will do nothing!
