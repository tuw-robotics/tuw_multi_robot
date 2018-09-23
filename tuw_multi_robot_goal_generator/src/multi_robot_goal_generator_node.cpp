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
    
    n_param_.param<int> ( "nr_of_available_robots", nr_of_available_robots_, -1 );
    n_param_.param<std::string> ( "frame_id", frame_id_, "map" );
    n_param_.param<std::string> ( "robot_name_prefix", robot_name_prefix_, "robot_" );
    n_param_.param<double> ( "distance_boundary", distance_boundary_, 0.5 );
    n_param_.param<double> ( "distance_between_robots", distance_between_robots_, 2. );
    n_param_.param<double> ( "distance_to_map_border", distance_to_map_border_, 0.2 );
    n_param_.param<int> ( "max_resample", max_resample_, 1000 );

    updateNrOfRobots ( nr_of_robots );
    pub_goals_ = n.advertise<tuw_multi_robot_msgs::RobotGoalsArray> ( "goals", 1, true );
    pub_map_goals_ = n.advertise<nav_msgs::OccupancyGrid> ( "valid_goal_locations", 1, true );
    sub_map_ = n.subscribe ( "map", 1, &RadomGoalGeneratorNode::callback, this );
}

void RadomGoalGeneratorNode::updateNrOfRobots ( size_t nr_of_robots ) {
    ROS_INFO ( "nr_of_robots: %i", ( int ) nr_of_robots );
    ROS_INFO ( "using prefix: %s", robot_name_prefix_.c_str() );
    robot_goals_array_.robots.clear();
    if((nr_of_available_robots_ > 0) && (nr_of_available_robots_ < nr_of_robots)){
        ROS_ERROR( "nr_of_robots must be equal or less then nr_of_available_robots" );
        return;
    }    
    robot_goals_array_.robots.resize ( nr_of_robots );
    if(nr_of_available_robots_ < 0){
        ROS_INFO( "nr_of_available_robots < 0 and therefore ignored" );
        for ( size_t i = 0; i < nr_of_robots; i++ ) {
            tuw_multi_robot_msgs::RobotGoals &robot_check_points = robot_goals_array_.robots[i];
            robot_check_points.robot_name = robot_name_prefix_ + std::to_string ( i );
        }
    } else {
        std::set<std::string> available_robots;        
        for ( size_t i = 0; i < nr_of_available_robots_; i++ ) {
            available_robots.insert(robot_name_prefix_ + std::to_string ( i ));
        }
        for ( size_t i = 0; i < nr_of_robots; i++ ) {
            tuw_multi_robot_msgs::RobotGoals &robot_check_points = robot_goals_array_.robots[i];
            int offset = rand() % available_robots.size();
            std::set<std::string>::iterator it = available_robots.begin();
            std::advance(it,offset);
            robot_check_points.robot_name = *it;
            available_robots.erase(it);
        }
        
    }
}


void RadomGoalGeneratorNode::callback ( const nav_msgs::OccupancyGrid::ConstPtr& msg ) {
    msg_map_ = msg;
    publish () ;
    ros::shutdown();
}

void RadomGoalGeneratorNode::publish () {
    map_.init ( msg_map_->info, msg_map_->data );
    msg_map_goals_.header = msg_map_->header;
    msg_map_goals_.info = msg_map_->info;
    msg_map_goals_.data.resize ( msg_map_->data.size() );
    map_goals_.init ( msg_map_goals_.info, msg_map_goals_.data );
    std::copy ( msg_map_->data.begin(), msg_map_->data.end(), msg_map_goals_.data.begin() );
    map_goals_.erode ( distance_boundary_ );
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
    for ( tuw_multi_robot_msgs::RobotGoals &robot: robot_goals_array_.robots ) {
        tuw::Pose2D pw;
        int retries = 0;
        bool valid_pose = false;
        do {
            pw.set ( dis_x ( gen ), dis_y ( gen ), 0 );
            if ( map_goals_.isFree ( pw.position() ) ) {
                if ( ( pw.x() > map_goals_.min_x() + distance_to_map_border_ ) && ( pw.x() < map_goals_.max_x() - distance_to_map_border_ ) && ( pw.y() > map_goals_.min_y() + distance_to_map_border_ ) && ( pw.y() < map_goals_.max_y() - distance_to_map_border_) ) {
                    valid_pose = true;
                }
            }
            retries++;
        } while ( !valid_pose && ( retries < max_resample_ ) );
        total_retries += retries;
        if ( retries < max_resample_ ) {
            pw.theta() = dis_alpha ( gen );
            //std::cout << pw << std::endl; //Each call to dis(gen) generates a new random double
            robot.destinations.resize ( 1 );
            map_goals_.circle ( pw.position(), distance_between_robots_, map_goals_.SPACE_OCCUPIED, -1 );
            geometry_msgs::Pose &p = robot.destinations[0];
            pw.copyToROSPose ( p );
        } else {
            ROS_WARN ( "Max retries on finding new free space for goals for robot: %s", robot.robot_name.c_str() );
        }
        if ( max_retries < retries ) {
            max_retries =  retries;
        }
    }


    robot_goals_array_.header.frame_id = frame_id_;
    robot_goals_array_.header.stamp = ros::Time::now();
    pub_goals_.publish ( robot_goals_array_ );

    pub_map_goals_.publish ( msg_map_goals_ );
    ROS_INFO ( "Goal msg published: %i max retries and %i total retries on finding free space ", max_retries,  total_retries );
}


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "RandomGoalGenerator" );
    ros::NodeHandle n;
    RadomGoalGeneratorNode node ( n );
    ros::spin();
    return 0;
}

