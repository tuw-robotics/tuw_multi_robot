#ifndef TUW_RANDOM_GOAL_GENERATOR_NODE_H
#define TUW_RANDOM_GOAL_GENERATOR_NODE_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "tuw_multi_robot_msgs/RobotGoalsArray.h"
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
    ros::Publisher pub_map_goals_;
    tuw::GridMap<int8_t> map_;
    tuw::GridMap<int8_t> map_goals_;
    int max_resample_;                     /// retries/max_resample steps to find a free spot for a goal  [m]
    int nr_of_available_robots_;           /// parameter  count
    double distance_boundary_;             /// parameter  [m]
    double distance_between_robots_;       /// parameter  [m]
    double distance_to_map_border_;        /// parameter  [m]
    std::string robot_name_prefix_;        /// parameter
    std::string frame_id_;                 /// parameter
    tuw_multi_robot_msgs::RobotGoalsArray robot_goals_array_;
    nav_msgs::OccupancyGrid msg_map_goals_;
    nav_msgs::OccupancyGrid::ConstPtr msg_map_;
    
    void updateNrOfRobots(size_t nr_of_robots);
    
};

#endif // TUW_RANDOM_GOAL_GENERATOR_NODE_H

