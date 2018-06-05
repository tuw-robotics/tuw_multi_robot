# tuw\_multi\_robot
This repository includes ros packages to plan routes for multiple robots on a search graph. It creates a graph out of a pixel map and tries to find a path for multiple robots using an extended approach for prioritized planning. The inputs are the odometry messages of the robots, the map and the desired goal poses and the output are multiple synchronized routes given to the single robots. 

#Packages

* tuw\_voronoi\_graph
* tuw\_multi\_robot\_route\_planner
* tuw\_multi\_robot\_rviz
* tuw\_multi\_robot\_control
* tuw\_multi\_robot\_demo

## tuw\_voronoi\_graph
This package includes a voronoi-map-generator, a voronoi-map-saver, a voronoi-map-server and a graph-generator-node. 

The _voronoi-map-generator-node_ receives a pixel map ([occupancy\_grid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)) and converts it into a [grid\_map](http://wiki.ros.org/grid_map) including the original map, the distance transform of the map and the voronoi path of the map.

The _voronoi-map-saver-node_ saves a published grid\_map in the given directory.

The _voronoi-map-generator-node_ publishes a saved grid\_map from a given directory.

The _graph-generator-node_ receives a grid\_map like the one published from voronoi-map-generator-node and converts it into a graph message for use in tuw\_multi\_robot\_route\_planner.


# dependencies
libdxflib-dev
