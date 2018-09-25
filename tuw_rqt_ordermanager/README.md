# Demo

```
roslaunch tuw_multi_robot_demo demo.launch room:=cave cfg:=robot_2 &
rosrun tuw_order_manager tuw_order_manager &
rqt --standalone tuw_rqt_ordermanager
```

Click 'new order' to add new order, then leftclick on the map to set waypoints (set at least two for each order).
Make sure to place waypoints at reachable locations and not too close to walls.
Click 'Start' to start.

