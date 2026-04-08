# RoboticsII_Final_Project

## Setting up communication between robots
1. Ensure robots are connected to the same network
2. [Both], run ```export ROS_DOMAIN_ID=X```, where X is replaced by a number 1-232. It must be the same number on both robots.
3. [Both] Run the docker setup
4. [1] ```ros2 launch yahboomcar_multi X3_bringup_multi_ctrl.launch.xml robot_name:=robot1```
5. [2] ```ros2 launch yahboomcar_multi X3_bringup_multi_ctrl.launch.xml robot_name:=robot2```

You can now be able to verify the connection with ```ros2 topic echo```, which should show a number of topics under /robot1/ and /robot2/
Both robots will subscribe to /cmd_vel. You can drive them both with ```ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"```
