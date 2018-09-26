# Orderplanner

Assigns robots to orders.

## orderplanner
The orderplanner monitors /robot_info to get knowledge of available robots and their positions.  
When an /orders message is received, it starts a solver to find an assignment of robots to orders.  
Unassigned robots are goaled to their current position.  
The assignment is published on the /goals topic.  
From /robot_info the orderplanner knows when a robot arrived at its current destination.  
New goals are emitted such that this robot receives the next station on its order, until all robots have reached the last station of their order.

The orderplanner further publishes /pickup, which is used to notify robots about which order they are picking up.  
/order_position is used to publish the current position of an order, as it gets transported by the robots.

## solver

### abstractsolver
abstractsolver.h aims to provide an interface to plug in different assignment algorithms.  
Currently this is as simple as it gets, for more sophisticated solvers this interface would probably need extension.

### simplesolver
The simplesolver works as follows:

* for all orders:
* * for all robots:
* * * compute linear distance between robot and first station of order
* sort distances ascending
* for all distances, if neither robot nor order is already picked
* * emit the respective tuple (robot, order)

