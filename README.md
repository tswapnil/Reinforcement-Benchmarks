# Reinforcement-Benchamrks

. ~/catkin_ws/devel/setup.bash
roslaunch BALL_STOP_gazebo BALL_STOP.launch


rosservice call gazebo/get_model_state '{model_name: unit_sphere_0}'

rosservice call gazebo/get_model_properties '{model_name: unit_sphere_0}'

rosservice call /gazebo/apply_body_wrench '{body_name: "coke_can::link" , wrench: { torque: { x: 0.01, y: 0 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }'

rosservice call /gazebo/clear_body_wrenches '{body_name: "coke_can::link"}'


--------------------------
https://www.generationrobots.com/blog/en/2015/02/robotic-simulation-scenarios-with-gazebo-and-ros/

https://answers.ros.org/question/9201/how-do-i-install-a-missing-ros-package/


