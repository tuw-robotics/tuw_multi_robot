# Custom Graph for the Multi Robot Router 

This demo shows how to draw a custom routing graph on a map and how to use it in the MRRP.

## Drawing the Graph

The graph generator requires a `.dxf` drawing of the map and the graph. For this purpose we use
Librecad which is available in Ubuntu, however any other CAD software could be used.
```
sudo apt-get install librecad
```

First, one has to import an existing map (e.g. a `.pgm` image used by the map server) via `File -> Import -> Insert Image`.
It is convenient to add a new layer for the graph such that a separate color can be used. 

When finished drawing, save the `.dxf` file in `tuw_multi_robot_demo/cfg/dxf/$ROOM/$DXF_NAME.dxf`, where `$ROOM` is the 
name of the map (room) on which the graph is drawn and `DXF_NAME` can be arbitrary, however one has to provide this name for
the converter. Afterwards, one has to launch the converter node as follows:
```
roslaunch tuw_multi_robot_demo dxf_to_graph.launch dxf_name:=$DXF_NAME.dxf room:=$ROOM graph_name:=$GRAPH_NAME
```
Here `$GRAPH_NAME` is again an arbitrary name. The generated graph is then stored at the following path: 
`tuw_multi_robot_demo/cfg/graph/$ROOM/cache/$GRAPH_NAME/`.

## Using the Graph

Now to use the drawn graph with the demo launch file, one has to set `load_segments` to true as well as set the `map_name`
parameter to use the custom graph. Here is an example:
```
roslaunch tuw_multi_robot_demo demo.launch room:=$ROOM map_name:=$GRAPH_NAME load_segments:=true
```
