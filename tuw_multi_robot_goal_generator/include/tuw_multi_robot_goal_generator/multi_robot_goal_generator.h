#ifndef TUW_RANDOM_GOAL_GENERATOR_NODE_H
#define TUW_RANDOM_GOAL_GENERATOR_NODE_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "tuw_geometry/grid_map.h"

/**
 * class to cover the ros communication
 **/
class RadomGoalGeneratorNode  {
public:
    RadomGoalGeneratorNode ( ros::NodeHandle & n ); /// Constructor
    void publish ();      /// publishes the motion commands 
    void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
private:
    ros::NodeHandle n_;         
    ros::NodeHandle n_param_;  
    ros::Subscriber sub_map_; 
    ros::Publisher pub_goals_;
    tuw::GridMap<int8_t> map_;
    
};

#endif // TUW_RANDOM_GOAL_GENERATOR_NODE_H

