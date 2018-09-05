#include "ros/ros.h"
#include <random>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include "tuw_multi_robot_msgs/RobotGoalsArray.h"

/**
 * class to cover the ros communication
 **/
class GoalSaverNode  {
public:
    GoalSaverNode ( ros::NodeHandle & n );
    void callback ( const tuw_multi_robot_msgs::RobotGoalsArray::ConstPtr& msg );
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    ros::Subscriber sub_goals_;
    std::string file_name_;                 /// parameter

    tuw_multi_robot_msgs::RobotGoalsArray robot_goals_array_;

};

GoalSaverNode::GoalSaverNode ( ros::NodeHandle & n )
    : n_ ( n ),  n_param_ ( "~" ) {
    n_param_.param<std::string> ( "file_name", file_name_, "/tmp/goals.cvs" );
    sub_goals_ = n.subscribe ( "goals", 1, &GoalSaverNode::callback, this );
}


void GoalSaverNode::callback ( const tuw_multi_robot_msgs::RobotGoalsArray::ConstPtr& msg ) {
    ROS_INFO ( "GoalSaverNode::callback" );

    std::string frame_id_;                 /// parameter
    std::ofstream file;
    file.open ( file_name_ );
    file << msg->header.frame_id << std::endl;
    file << std::setprecision(std::numeric_limits<float>::digits10 + 1);
    file << std::setw(12) << msg->header.stamp.sec << ", " << std::setw(12) << msg->header.stamp.nsec << std::endl;
    for ( size_t i = 0; i < msg->goals.size(); i++ ) {
        const tuw_multi_robot_msgs::RobotGoals &robot = msg->goals[i];
        file << robot.robot_name << std::endl;
        for( size_t j = 0; j < robot.path_points.size(); j++){
            const geometry_msgs::Pose p = robot.path_points[j];
            file << std::setw(12) << p.position.x << ", " << std::setw(12) << p.position.y << ", " << std::setw(12) << p.position.z << ", ";
            file << std::setw(12) << p.orientation.x << ", " << std::setw(12) << p.orientation.y << ", " << std::setw(12) << p.orientation.z << ", " << std::setw(12) << p.orientation.w << std::endl;
        }

    }
    file.close();
}


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "GoalSaver" );
    ros::NodeHandle n;
    GoalSaverNode node ( n );
    ros::spin();
    return 0;
}
