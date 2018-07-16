# tuw\_voronoi\_graph
This package contains two nodes where one node is used to generate and load voronoi_graphs out of a map automatically, and one node is used to create graphs out of pre-drawn segments in .dxf format.

## tuw\_dxf\_to\_graph\_node
is a standard executable which takes a .dxf file as input and generates a graph which is saved to be loaded with the tuw\_voronoi\_graph\_node.

The dxf file must contain the original map.pgm (e.g. used by map server) transformed by scale and offset, to give scale and offset to the application (Theoretically any image with the right scale and offset would work as well). Additionally lines, circles and arcs can be included, which are included to generate the segments. All elements which share an endpoint are connected.

Each resutling segment has a width, given by the commandline argument -w and a lenght, which has at least the length of the commandline argument [-l] and at max the length of [2*(-l) - epsilon].

### Arguments
`-h [ --help ]`

	Display help message

`-i [ --input ] arg (=./segments.dxf)`

    The path to the file

`-o [ --output ] arg (=./graphs/segments)`

	The output directory

`-w [ --width ] arg (=0.600000024)`

    The width of a segments in meters

`-l [ --length ] arg (=1)`

	The length of a segment in meters


## tuw\_voronoi\_graph\_node

Receives a pixel map (occupancy_grid) and converts it into a `tuw_msgs::graph` message spanning the whole free space of the map. Additionally the graph is saved to a given folder. If a map is allready converted to a graph the graph is loaded from memory to save computation time.

### Subscribed Topics

`~map` (`nav_msgs::occupancy_grid`)
    The map used for planning (used for matching odom pose to graph)

### Published Topics

`~segments` (`tuw_multi_robot_msgs::VoronoiGraph`)
    The generated graph

### Parameters

`~map_topic` (`string` default: "/map")

     Sets the topic where the map is read from

`~map_smoothing` (`int` default: "15")

     Sets the parameter to smooth the map making cleaner voronoi graphs. (Opencv sometimes segfaults during this process (change this param if so). Keeping this param at zero never produces any segfault.

`~segments_topic` (`string` default: "/segments")

     The topic where the graph is published.

`~segment_length` (`float` default: "0.9")

     Defines the length of a graph segment.

`~crossing_opimization` (`float` default: "0.2")

     Crossings which have less distance than this value are merged together

`~end_segment_optimization` (`float` default: "0.4")

     End segments (has only one neighbor) which are shorter than this value are removed.

`~graph_path` (`string` default: ".") 

     Defines the path were the generated graphs are saved


`~custom_graph_path` (`string` default: "") 

     If this param is set a custom generated graph is loded from this path


## tuw segment to graph node

Receives a pixel map (occupancy_grid) and converts it into a `tuw_msgs::graph` message spanning the whole free space of the map. Additionally the graph is saved to a given folder. If a map is allready converted to a graph the graph is loaded from memory to save computation time.

### Subscribed Topics

## Published Topics

`~segments` (`tuw_multi_robot_msgs::VoronoiGraph`)

    The generated graph


### Parameters

`~segment_file` (`string` default: "segments.yaml")
    The filepath to the defined segments.

`~segments_topic` (`string` default: "/segments")

     The topic where the graph is published.

`~segment_length` (`float` default: "0.9")

     Defines the length of a graph segment.

### Sample Segments File
start_x:      [      0,        2,   3.2,    5]
start_y:      [   -1.5,     -1.5,    -2,   -4]
end_x:        [      2,      3.2,     5,    3]
end_y:        [   -1.5,       -2,    -4,   -4]
space:        [    1.0,      1.0,   1.0,  1.0]
origin_x:     15
origin_y:     15
resolution:   0.05
