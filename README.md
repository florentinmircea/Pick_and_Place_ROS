# Pick_and_Place_ROS

I have chosen as my robotic platform the CRANE-X7 robot which is of type manipulator.
It features a 7 DOF;this robot is compact, nice styled and it’s ideal for research of
collaborative robots.

The robot is simulated with Gazebo Simulator and controlled using MoveIt Motion
Planning Framework.

I implemented a pick and place task.In Gazebo Simulator I have on the
table 2 wooden cubes which needs to be placed one on top of the other.The 2 cubes
initially are placed at different coordinates. The robot should pick one cube from its
location and place it to a fixed location defined in the script then repeat the task for the
second one.

Links for all the resources used
https://robots.ros.org/crane-x7/
https://github.com/rt-net/crane_x7_ros
