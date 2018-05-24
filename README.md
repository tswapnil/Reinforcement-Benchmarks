# Reinforcement-Benchamrks

. ~/catkin_ws/devel/setup.bash
roslaunch BALL_STOP_gazebo BALL_STOP.launch

roslaunch BALL_STOP_PLEX_gazebo BALL_STOP_PLEX.launch

rostopic echo -n 1 /gazebo/model_states

rostopic pub -l /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: unit_sphere_0, pose: { position: { x: 0, y: 0, z: 0.5 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'

rosservice call gazebo/get_model_state '{model_name: unit_sphere_0}'

rosservice call gazebo/get_model_properties '{model_name: unit_sphere_0}'


rosservice call /gazebo/clear_body_wrenches '{body_name: "coke_can::link"}'


rosservice call /gazebo/apply_body_wrench '{body_name: "unit_sphere_0::link", reference_frame: "unit_sphere_0::link", wrench: { force: { x: 2.0, y: 0, z: 0.0 } }, start_time: 0, duration: -1 }'

--------------------------
Link for creating topics and publisher-subscriber mechanism in C++
http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5


https://www.generationrobots.com/blog/en/2015/02/robotic-simulation-scenarios-with-gazebo-and-ros/

https://answers.ros.org/question/9201/how-do-i-install-a-missing-ros-package/

https://answers.ros.org/question/11047/applying-a-force-to-a-rigid-body/

