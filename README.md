# tuw\_multi\_robot
This repository includes ros packages to plan routes for multiple robots on a search graph. It creates a graph out of a pixel map and tries to find a path for multiple robots using an extended approach for prioritized planning. The inputs are the odometry messages of the robots, the map and the desired goal poses and the output are multiple synchronized routes given to the single robots. 
# Installation
Have a look at the [INSTALL.md](INSTALL.md) file
# Demos / Tutorials
Have a look at the [tuw_multi_robot_demo/README.md](tuw_multi_robot_demo/README.md) file

# Packages
* tuw\_multi\_robot\_demo
* tuw\_voronoi\_graph
* tuw\_multi\_robot\_router
* tuw\_multi\_robot\_rviz
* tuw\_multi\_robot\_ctrl
* tuw\_multi\_robot\_local\_behavior\_controller

## tuw\_multi\_robot\_demo
Contains launch and config files to run a sample demo. 

```
roslaunch tuw_multi_robot_demo demo.launch room:=cave cfg:=robot_2
```

## tuw\_voronoi\_graph
This package includes a voronoi-graph-generator a dxf-to-graph-node and a segment-to graph node for creating routing graphs for the multi robot router.

The _voronoi-graph-generator-node_ receives a pixel map ([occupancy\_grid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)) and converts it into a voronoi graph describing the original map. This graph is automatically generated or loaded from a cache folder if saved. Additionally the node can load specific graphs saved in a folder.

The _voronoi-dxf-to-graph_ takes a dxf file as input containing the scaled and transformed map and any number of lines arcs and circles. These lines arcs and circles are converted to a graph and saved to a specific location.

![Librecad][tuw_multi_robot/tuw_multi_robot_demo/res/roblab_dxf_graph.png]

The _voronoi-segment-to-graph-node_ takes a segment file with predefined segments as input and converts it to a graph, which is published afterwords.

The _graph-generator-node_ receives a grid\_map like the one published from voronoi-map-generator-node and converts it into a graph message for use in tuw\_multi\_robot\_route\_planner.

## tuw\_multi\_robot\_router
The tuw_multi_robot_router is a Multi Robot Route Planner, which subscribes to all given robots odometry messages, to the map/graph published from the tuw_voronoi_graph package, and to the PoseIdArray msg from the tuw_multi_robot_msgs package.

The MRRP uses a prioritized planning approach to find the robots routes. Additionally, there are a Priority and a Speed Rescheduler as well as a Collision resolver integrated to solve special scenarios non solvable by standard prioritized planning approaches. Since the results generated for these scenarios are dependent the given routes have to be executed synchronized. Therefore, the Router publishes a Route from tuw_multi_robot_msgs containing preconditions, when a robot is allowed to enter a segment. Additionally a unsynchronized nav_msgs::Path is published for every robot. The algorithm is documented in the master thesis [1]. 

## tuw\_multi\_robot\_rviz
Presents rviz plugins to set goal positions for the planner and a tool to visualize generated graphs. 

## tuw\_multi\_robot\_ctrl
A simple multi robot controller using Routes as input, which are used to execute the path synchronized. (Used for testing)

## tuw\_multi\_robot\_local\_behavior\_controller
This package contains a node, which receives the tuw_segment_path msg for a robot and publishes a nav_msgs::path up to the point a robot is allowed to move.

In detail: A tuw_segment_path contains a set of segments, where each of them has preconditions to tell when a robot is allowed to enter a certain segment. The tuw_multi_robot_route_to_path_node subscribes to these messages and checks how many of these preconditions are met and publishes a path from start to the least segment, for which the preconditions are met. This node subscribes to all robots as one node for performance reasons while testing with a large number of robots. 

# dependencies
libdxflib-dev
tuw\_multi\_robot\_msgs

# references
http://wiki.ros.org/tuw_multi_robot

# citations
[1] [Binder, B. (2017). Spatio-Temporal Prioritized Planning (Master thesis), Retrieved from TU Wien Bibliothekssystem (Accession No. AC14520240)](http://repositum.tuwien.ac.at/obvutwhs/content/titleinfo/2400890)
