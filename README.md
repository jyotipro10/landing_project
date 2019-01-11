# landing_project
after adding node , launch and cmake file:
1. source devel/setup.bash
2. catkin_make --pkg mav_trajectory_generation_ros

In one tab execute the command:
roslaunch rotors_gazebo mav.launch

In another tab run the command:
roslaunch mav_linear_mpc mav_linear_mpc_sim.launch

In the next tab run the following command:
rosrun rviz rviz

Inside rviz,do the following:
1. In the left part of the rviz window, change map field to world.
2. Then add the topic TF under the tab display type.

The in another tab, launch the node with the command:
roslaunch mav_trajectory_generation_ros my_node.launch

To change position:
rostopic pub /firefly/command/pose geometry_msgs/PoseStamped "header:
     seq: 0
     stamp:
       secs: 0
       nsecs: 0
     frame_id: 'world'
   pose:
     position:
       x: 0.0
       y: 0.0
       z: 1.0
     orientation:
       x: 0.0
       y: 0.0
       z: 0.0
       w: 0.0"
       
Take care to first send the drone to higher ground through above command else launching node will not produce any results.
