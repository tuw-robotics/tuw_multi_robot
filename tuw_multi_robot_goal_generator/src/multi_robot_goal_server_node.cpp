#include <ros/ros.h>
#include "tuw_multi_robot_goal_generator/multi_robot_goal_handler.h"


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "GoalServer" );
    ros::NodeHandle n;
    GoalHandlerNode node ( n, GoalHandlerNode::READ);
    node.publish();
    return 0;
}
