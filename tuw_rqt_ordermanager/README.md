# RQT Ordermanager
This RQT plugin provides a map view where stations and orders can be built.
<img src="res/rqt_ordermanager.png" alt="RQT ordermanager" width="400px" />

# Demo

```
roslaunch tuw_multi_robot_demo demo.launch room:=cave cfg:=robot_2 &
# wait until started
rosrun tuw_station_manager tuw_station_manager_node.py &
rosrun tuw_order_planner tuw_order_planner &
rqt --standalone tuw_rqt_ordermanager
```

Leftclick on the map to create stations.
Click 'new order' to add a new order, then click on stations on the map to add that station to the order.
Stations are always added to the order which is currently selected in the order list on the right.
The first station of an order is marked by a circle around.

Make sure to place stations at reachable locations and not too close to walls.
Click 'Start' to start.

Hold the right mouse button to pan the map view.
Use the scrollwheel to zoom in or out.
A rightclick on the map opens a context menu where you can delete stations.

# Code overview
* GraphicsView: subclass of QGraphicsView with panning, zooming, contextmenu
* Item{Order,Robot,Station}: subclass of QGraphicsItem and QListWidgetItem so we can have the same object in the list and graphics views. Most important is the `paint()` function
* MapTransformation: deal with transformations of coordinates or poses between ROS map coordinates and QT scene coordinates
* \*Dialog: gui code for create/edit dialogs
* RQTOrdermanager: RQT plugin and ROS code

## Threads
Both QT and ROS run in their own thread. 
This means it is not safe to alter QT structures inside a ROS message handler. 
Here, the QT signal/slot mechanism (`connect()`) is used to transfer control from the ROS to the QT thread.
Still some synchronisation of shared structures are necessary, for which mutexes are used.

# Troubleshooting
If the plugin does not load after (re-)compiling, try `rqt --clear-config`.

# Installl

```
sudo apt install ros-melodic-rqt ros-melodic-rqt-common-plugins
```
