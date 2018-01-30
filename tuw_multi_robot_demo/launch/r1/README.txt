Lauch files to start pioneer (r1) with the multi robot router

roscore
roslaunch tuw_p3dx gazebo.launch room:=roblab
roslaunch tuw_multi_robot_demo global_router.launch room:=roblab
roslaunch tuw_p3dx navigation_mpn.launch use_localization:=false room:=roblab use_nodelet:=true use_voronoi:=false
roslaunch tuw_p3dx navigation_mpn_rviz.launch