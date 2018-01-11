#ifndef MULTI_SEGMENT_CONTROLLER_NODE_H
#define MULTI_SEGMENT_CONTROLLER_NODE_H


// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tuw_multi_robot_msgs/SegmentPath.h>

#include <simple_velocity_controller/segment_controller.h>
#include <memory>

namespace  velocity_controller
{
    class MultiSegmentControllerNode
    {
            //special class-member functions.
        public   : MultiSegmentControllerNode ( ros::NodeHandle& n );

            //ROS:
        public   : ros::NodeHandle                          n_;      ///< Node handler to the root node
        public   : ros::NodeHandle                          n_param_;///< Node handler to the current node
        public   : std::unique_ptr<ros::Rate>               rate_;

            // ROS Publishers
        private  : std::vector<ros::Publisher>              pubCmdVel_;


            // ROS Subscribers
        private  : std::vector<ros::Subscriber>             subOdom_;
        private  : std::vector<ros::Subscriber>             subPath_;
        private  : std::vector<ros::Subscriber>             subCtrl_;


            // ROS Topic names
        private  : std::string                              topic_cmdVel_;
        private  : std::string                              topic_odom_;
        private  : std::string                              topic_path_;
        private  : std::string                              topic_ctrl_;


        private  : float                                    max_vel_v_;
        private  : float                                    max_vel_w_;

        private  : float                                    goal_r_;
        private  : float                                    Kp_val_;
        private  : float                                    Ki_val_;
        private  : float                                    Kd_val_;


        private  : ros::Time                                last_update_;


        private:    std::vector<std::string>            robot_names_;
        private  : void subOdomCb ( const ros::MessageEvent<nav_msgs::Odometry const>& _event, int _topic );
        private  : void subPathCb ( const ros::MessageEvent<tuw_multi_robot_msgs::SegmentPath const>& _event, int _topic );
        private  : void subCtrlCb ( const ros::MessageEvent<std_msgs::String const>& _event, int _topic );

        private : std::vector<SegmentController>         controller;
    };

}

#endif // CONTROLLER_NODE_H

