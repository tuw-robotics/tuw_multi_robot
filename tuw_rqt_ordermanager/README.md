# Demo

```
roslaunch tuw_multi_robot_demo demo.launch room:=cave cfg:=robot_2 &
rosrun tuw_station_manager tuw_station_manager_node.py &
rosrun tuw_order_planner tuw_order_planner &
rqt --standalone tuw_rqt_ordermanager
```

Leftclick on the map to create stations.  
Click 'new order' to add a new order, then click on stations on the map to associate to that station.

Make sure to place stations at reachable locations and not too close to walls.
Click 'Start' to start.

