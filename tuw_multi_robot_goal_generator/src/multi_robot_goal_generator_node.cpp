#include "ros/ros.h"
#include <random>
#include "tuw_geometry/utils.h"
#include "tuw_multi_robot_goal_generator/multi_robot_goal_generator.h"
#include <algorithm>



RadomGoalGeneratorNode::RadomGoalGeneratorNode ( ros::NodeHandle & n )
    :   n_ ( n ),
        n_param_ ( "~" ) {
    int nr_of_robots = 0;
    if ( ! n_param_.getParam ( "nr_of_robots", nr_of_robots ) ) {
        ROS_ERROR ( "nr_of_robots is not set" );
        return;
    }
    n_param_.param<std::string> ( "frame_id", frame_id_, "map" );
    n_param_.param<std::string> ( "robot_name_prefix", robot_name_prefix_, "robot_" );
    n_param_.param<double> ( "distance_boundary", distance_boundary_, 0.5 );
    n_param_.param<int> ( "max_resample", max_resample_, 1000 );

    updateNrOfRobots ( nr_of_robots );
    pub_goals_ = n.advertise<tuw_multi_robot_msgs::RobotGoalsArray> ( "goals", 1, true );
    pub_map_goals_ = n.advertise<nav_msgs::OccupancyGrid> ( "map_goals", 1, true );
    sub_map_ = n.subscribe ( "map", 1, &RadomGoalGeneratorNode::callback, this );
}

void RadomGoalGeneratorNode::updateNrOfRobots ( size_t nr_of_robots ) {
    ROS_INFO ( "nr_of_robots: %i", ( int ) nr_of_robots );
    ROS_INFO ( "using prefix: %s", robot_name_prefix_.c_str() );
    robot_goals_array_.goals.resize ( nr_of_robots );
    for ( size_t i = 0; i < nr_of_robots; i++ ) {
        tuw_multi_robot_msgs::RobotGoals &robot_check_points = robot_goals_array_.goals[i];
        robot_check_points.robot_name = robot_name_prefix_ + std::to_string ( i );
    }
}


void RadomGoalGeneratorNode::callback ( const nav_msgs::OccupancyGrid::ConstPtr& msg ) {

    map_.init ( msg->info, &msg->data[0] );
    msg_map_goals_.header = msg->header;
    msg_map_goals_.info = msg->info;
    msg_map_goals_.data.resize ( msg->data.size() );
    map_goals_.init ( msg_map_goals_.info, &msg_map_goals_.data[0] );
    std::copy(msg->data.begin(), msg->data.end(), msg_map_goals_.data.begin());
    map_goals_.erode(distance_boundary_);
    //std::cout << map_.infoHeader() << std::endl;
    //std::cout << tuw::format(map_.Mw2m()) << std::endl;
    //std::cout << tuw::format(map_.Mm2w()) << std::endl;
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen ( rd() ); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis_x ( map_.min_x(), map_.max_x() );
    std::uniform_real_distribution<> dis_y ( map_.min_y(), map_.max_y() );
    std::uniform_real_distribution<> dis_alpha ( -M_PI, M_PI );
    
    int total_retries = 0;
    int max_retries = 0;
    for ( tuw_multi_robot_msgs::RobotGoals &robot: robot_goals_array_.goals ) {
        tuw::Pose2D pw;
        int retries = 0;
        do {
            pw.set ( dis_x ( gen ), dis_y ( gen ), 0 );
            retries++;
        } while ( (!map_goals_.isFree ( pw.position() )) && (retries < max_resample_));
        total_retries += retries;
        if( retries < max_resample_){
            pw.theta() = dis_alpha(gen);
            //std::cout << pw << std::endl; //Each call to dis(gen) generates a new random double
            robot.path_points.resize ( 1 );
            map_goals_.circle(pw.position(), distance_boundary_, map_goals_.SPACE_OCCUPIED, -1 );
            geometry_msgs::Pose &p = robot.path_points[0];
            pw.copyToROSPose(p);
        } else {
            ROS_WARN ( "Max retries on finding new free space for goals for robot: %s", robot.robot_name.c_str() );
        }
        if(max_retries < retries) max_retries =  retries;
    }
    

    robot_goals_array_.header.frame_id = frame_id_;
    robot_goals_array_.header.stamp = ros::Time::now();
    pub_goals_.publish ( robot_goals_array_ );

    pub_map_goals_.publish ( msg_map_goals_ );
    ROS_INFO ( "Goal msg published: %i max retries and %i total retries on finding free space ", max_retries,  total_retries);




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
