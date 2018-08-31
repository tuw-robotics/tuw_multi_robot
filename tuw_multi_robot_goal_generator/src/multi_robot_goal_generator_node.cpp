#include "ros/ros.h"
#include "tuw_multi_robot_goal_generator/multi_robot_goal_generator.h"
#include "tuw_multi_robot_msgs/RobotGoalsArray.h"

 


RadomGoalGeneratorNode::RadomGoalGeneratorNode ( ros::NodeHandle & n ){
    sub_map_ = n.subscribe ( "map", 1, &RadomGoalGeneratorNode::callback, this );
    pub_goals_ = n.advertise<tuw_multi_robot_msgs::RobotGoalsArray> ( "goals", 1 );
}

void RadomGoalGeneratorNode::callback ( const nav_msgs::OccupancyGrid::ConstPtr& msg ) {
    map_.init ( msg->info, &msg->data[0] );  
    std::cout << "hihi\n";
}

void RadomGoalGeneratorNode::publish () {
}


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "RandomGoalGenerator" );
    ros::NodeHandle n;
    RadomGoalGeneratorNode node ( n );
    ros::spin();
    return 0;
}
