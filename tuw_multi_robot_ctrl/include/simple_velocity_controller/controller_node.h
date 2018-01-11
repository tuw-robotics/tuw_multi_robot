#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H


// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>

#include <simple_velocity_controller/controller.h>
#include <memory>

namespace  velocity_controller
{
  class ControllerNode : public velocity_controller::Controller
  {
      //special class-member functions.
      public   : ControllerNode(ros::NodeHandle & n);
      
      //ROS:
      public   : ros::NodeHandle n_;      ///< Node handler to the root node
      public   : ros::NodeHandle n_param_;///< Node handler to the current node
      public   : std::unique_ptr<ros::Rate> rate_;
      
      // ROS Publishers
      private  : ros::Publisher  pubCmdVel_;
      
      
      // ROS Subscribers
      private  : ros::Subscriber              subOdom_; 
      private  : ros::Subscriber              subPath_; 
      private  : ros::Subscriber              subCtrl_; 
      
      
      // ROS Topic names
      private  : std::string				topic_cmdVel_;
      private  : std::string				topic_odom_;
      private  : std::string 				topic_path_;
	  private  : std::string 				topic_ctrl_;
	  
      
      private  : float						max_vel_v_;
      private  : float						max_vel_w_;
      
      private  : float						goal_r_;
      private  : float						Kp_val_;
      private  : float						Ki_val_;
      private  : float						Kd_val_;
	  
      
      private  : ros::Time					last_update_;
      
      
      private  : void subOdomCb( const nav_msgs::Odometry::ConstPtr& _odom );
      private  : void subPathCb( const nav_msgs::Path::ConstPtr& _path );
      private  : void subCtrlCb(const std_msgs::String _cmd);
  };

}

#endif // CONTROLLER_NODE_H
