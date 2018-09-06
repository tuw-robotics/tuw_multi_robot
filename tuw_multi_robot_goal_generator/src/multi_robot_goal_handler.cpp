#include "ros/ros.h"
#include <random>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include "tuw_multi_robot_goal_generator/multi_robot_goal_handler.h"
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tuw_geometry/utils.h"


GoalHandlerNode::GoalHandlerNode ( ros::NodeHandle & n, Mode mode )
    : n_ ( n ),  n_param_ ( "~" ) {
    n_param_.param<std::string> ( "file_name", file_name_, "/tmp/goals.txt" );
    
    n_param_.param<bool> ( "run_once", run_once_, "false" );
    if (mode == READ) {
        n_param_.param<double> ( "loop_rate", loop_rate_, 10.0 );
        n_param_.param<bool> ( "time_now", time_now_, "true" );
        pub_goals_ = n.advertise<tuw_multi_robot_msgs::RobotGoalsArray> ( "goals", 1 );
    }
    if (mode == WRITE) {
        sub_goals_ = n.subscribe ( "goals", 1, &GoalHandlerNode::callback, this );
    }
}


void GoalHandlerNode::callback ( const tuw_multi_robot_msgs::RobotGoalsArray& msg ) {
    ROS_INFO ( "GoalHandlerNode::callback" );
    msg_ = msg;

    std::string frame_id_;                 /// parameter
    std::ofstream file;
    file.open ( file_name_ );
    ROS_INFO ( "Write file %s", file_name_.c_str() );
    file << "tuw_multi_robot_msgs::RobotGoalsArray @" << boost::posix_time::to_iso_extended_string ( ros::Time::now().toBoost() ) << std::endl;
    file << "frame_id: " << msg_.header.frame_id << std::endl;
    file << std::setprecision ( std::numeric_limits<float>::digits10 + 1 );
    file << std::setw ( 12 ) << msg_.header.stamp.sec << ", " << std::setw ( 12 ) << msg_.header.stamp.nsec << std::endl;
    for ( size_t i = 0; i < msg_.goals.size(); i++ ) {
        const tuw_multi_robot_msgs::RobotGoals &robot = msg_.goals[i];
        file << robot.robot_name << std::endl;
        for ( size_t j = 0; j < robot.path_points.size(); j++ ) {
            const geometry_msgs::Pose p = robot.path_points[j];
            file << std::setw ( 12 ) << p.position.x << ", " << std::setw ( 12 ) << p.position.y << ", " << std::setw ( 12 ) << p.position.z << ", ";
            file << std::setw ( 12 ) << p.orientation.x << ", " << std::setw ( 12 ) << p.orientation.y << ", " << std::setw ( 12 ) << p.orientation.z << ", " << std::setw ( 12 ) << p.orientation.w << std::endl;
        }
    }
    file.close();
    if(run_once_) ros::shutdown();
}
void  GoalHandlerNode::publish () {
    ros::Rate loop_rate(loop_rate_);
    do {
        publishGoal(file_name_);
        ros::spinOnce();
        loop_rate.sleep();
    } while ( ros::ok() && (run_once_ == false));
}

void GoalHandlerNode::publishGoal ( const std::string &file_name ) {
    std::ifstream file ( file_name );
    if ( !file.good() ) {
        ROS_ERROR ( "File: %s does not exist!", file_name.c_str() );
        return;
    }
    if ( !file.is_open() ) {
        ROS_ERROR ( "Can't open file %s!", file_name.c_str() );
        return;
    }
    ROS_INFO ( "Reading file %s", file_name.c_str() );

    std::vector<std::string> columns;
    std::string line;
    // Titel
    if ( getline ( file,line ) ) {
        boost::erase_all ( line, " " );
        boost::split ( columns, line, boost::is_any_of ( "@ " ) );
        if ( ( columns.size() > 0 ) && boost::iequals ( columns[0],"tuw_multi_robot_msgs::RobotGoalsArray" ) ) {
            ROS_INFO ( "Start reading tuw_multi_robot_msgs::RobotGoalsArray" );
        } else {
            ROS_ERROR ( "Missing keyword tuw_multi_robot_msgs::RobotGoalsArray" );
            return;
        }
    } else {
        ROS_ERROR ( "No header in File: %s!", file_name.c_str() );
        return;
    }
    // Header
    if ( getline ( file,line ) ) {
        boost::erase_all ( line, " " );
        boost::split ( columns, line, boost::is_any_of ( ":" ) );
        if ( ( columns.size() == 2 ) && boost::iequals ( columns[0],"frame_id" ) ) {
            msg_.header.frame_id = columns[1];
        } else {
            ROS_ERROR ( "Missing keyword frame_id or frame_id value" );
        }
    } else {
        ROS_ERROR ( "No header in File: %s!", file_name.c_str() );
        return;
    }
    // Timestamp
    if ( getline ( file,line ) ) {
        boost::erase_all ( line, " " );
        boost::split ( columns, line, boost::is_any_of ( "," ) );
        if ( columns.size() == 2 ) {
            msg_.header.stamp.sec  = std::stol ( columns[0] );
            msg_.header.stamp.nsec = std::stol ( columns[1] );
        } else {
            ROS_INFO ( "Can't read timestamp" );
        }
    } else {
        ROS_ERROR ( "No Timestamp in File: %s!", file_name.c_str() );
        return;
    }
    // Goals
    tuw_multi_robot_msgs::RobotGoals::_path_points_type *path_points = NULL;
    while ( getline ( file,line ) ) {
        boost::erase_all ( line, " " );
        boost::split ( columns, line, boost::is_any_of ( "," ) );
        // Robot name
        if ( columns.size() == 1 ) {
            if ( columns[0].size() > 0 ) {
                tuw_multi_robot_msgs::RobotGoals robot;
                robot.robot_name = line;
                msg_.goals.push_back ( robot );
                path_points = &msg_.goals.back().path_points;
            } else {
                ROS_ERROR ( "robot name to short!" );
                return;
            }
        }
        if ( ( columns.size() == 7 ) && ( path_points != NULL ) ) {
            geometry_msgs::Pose p;
            p.position.x = std::stod ( columns[0] );
            p.position.y = std::stod ( columns[1] );
            p.position.z = std::stod ( columns[2] );
            p.orientation.x  = std::stod ( columns[3] );
            p.orientation.y = std::stod ( columns[4] );
            p.orientation.z = std::stod ( columns[5] );
            p.orientation.w = std::stod ( columns[6] );
            path_points->push_back ( p );
        }
        if ( ( columns.size() == 4 ) && ( path_points != NULL ) ) {
            geometry_msgs::Pose p;
            p.position.x = std::stod ( columns[0] );
            p.position.y = std::stod ( columns[1] );
            p.position.z = std::stod ( columns[2] );
            double alpha  = std::stod ( columns[3] );
            tuw::EulerYawToQuaternion ( alpha, p.orientation );
            path_points->push_back ( p );
        }
    }
    if(time_now_){
        msg_.header.stamp = ros::Time::now();
    }
    
    ros::Time timeout = ros::Time::now() + ros::Duration(10);
    int nr_of_subscribers = pub_goals_.getNumSubscribers();
    while (ros::Time::now() < timeout && (nr_of_subscribers <= 0)){
        ROS_INFO ( "NumSubscribers: %i", nr_of_subscribers );
        ros::Duration(0.1).sleep();
        nr_of_subscribers = pub_goals_.getNumSubscribers();
    }
    ROS_INFO ( "NumSubscribers: %i", nr_of_subscribers );
    pub_goals_.publish ( msg_ );
        
    file.close();
}
